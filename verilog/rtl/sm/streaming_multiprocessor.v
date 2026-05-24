//------------------------------------------------------------------------------
// Module: streaming_multiprocessor
// Description: Complete SM unit with 64 CUDA cores, 1 tensor core,
//              64KB register file, 32KB shared memory
//              Warp scheduler supporting 4 warps x 32 threads
//              Instruction dispatch and local memory controller
//------------------------------------------------------------------------------

module streaming_multiprocessor #(
    parameter NUM_CUDA_CORES   = 64,
    parameter NUM_TENSOR_CORES = 1,
    parameter WARPS_PER_SM     = 4,
    parameter THREADS_PER_WARP = 32,
    parameter REGFILE_SIZE     = 16384,   // 64KB / 4 bytes
    parameter SHARED_MEM_SIZE  = 32768,   // 32KB
    parameter DATA_WIDTH       = 32
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // Global control
    input  wire                      sm_enable,
    input  wire [7:0]                sm_id,

    // Instruction fetch interface
    input  wire [31:0]               instruction_in,
    input  wire                      instruction_valid,
    output wire                      instruction_ready,
    input  wire [31:0]               pc_in,
    output reg  [31:0]               pc_out,

    // Memory interface (to L1 cache / interconnect)
    output reg                       mem_req_valid,
    output reg  [31:0]               mem_req_addr,
    output reg  [DATA_WIDTH-1:0]     mem_req_data,
    output reg                       mem_req_we,
    output reg  [3:0]                mem_req_be,      // Byte enable
    input  wire                      mem_resp_valid,
    input  wire [DATA_WIDTH-1:0]     mem_resp_data,

    // Tensor core interface (to shared memory / register file)
    input  wire                      tensor_op_valid,
    input  wire [2:0]                tensor_op_mode,
    output wire                      tensor_busy,

    // Status outputs
    output reg                       sm_busy,
    output reg  [7:0]                active_warps,
    output reg  [15:0]               inst_count
);

    //--------------------------------------------------------------------------
    // Local parameters
    //--------------------------------------------------------------------------
    localparam TOTAL_THREADS = WARPS_PER_SM * THREADS_PER_WARP;  // 128
    localparam WARP_ID_W     = $clog2(WARPS_PER_SM);  // 2
    localparam THREAD_ID_W   = $clog2(THREADS_PER_WARP);  // 5
    localparam TOTAL_CORES   = NUM_CUDA_CORES;  // 64

    // Instruction opcodes (simplified PTX-like ISA)
    localparam OPCODE_NOP      = 6'b000000;
    localparam OPCODE_LD       = 6'b000001;
    localparam OPCODE_ST       = 6'b000010;
    localparam OPCODE_MOV      = 6'b000011;
    localparam OPCODE_ADD      = 6'b000100;
    localparam OPCODE_MUL      = 6'b000101;
    localparam OPCODE_FMA      = 6'b000110;
    localparam OPCODE_MMA      = 6'b000111;  // Matrix multiply-accumulate
    localparam OPCODE_BAR      = 6'b001000;  // Barrier
    localparam OPCODE_BRA      = 6'b001001;  // Branch
    localparam OPCODE_EXIT     = 6'b001010;

    //--------------------------------------------------------------------------
    // Warp state tracking
    //--------------------------------------------------------------------------
    reg [WARPS_PER_SM-1:0] warp_active;
    reg [WARPS_PER_SM-1:0] warp_waiting_barrier;
    reg [WARPS_PER_SM-1:0] warp_waiting_memory;
    reg [31:0]             warp_pc [0:WARPS_PER_SM-1];
    reg [THREADS_PER_WARP-1:0] thread_active [0:WARPS_PER_SM-1];

    // Current scheduled warp
    reg [WARP_ID_W-1:0]    current_warp;
    reg [WARP_ID_W-1:0]    next_warp;
    reg [2:0]              warp_schedule_cnt;

    // Instruction decode registers
    reg [31:0]             decoded_inst;
    reg                    decoded_valid;
    reg [5:0]              decoded_opcode;
    reg [5:0]              decoded_dest;
    reg [5:0]              decoded_src_a;
    reg [5:0]              decoded_src_b;
    reg [5:0]              decoded_src_c;
    reg [15:0]             decoded_imm;

    //--------------------------------------------------------------------------
    // Register file connections
    //--------------------------------------------------------------------------
    // 8 read ports, 4 write ports
    wire [7:0]             rf_read_en;
    wire [13:0]            rf_read_addr [0:7];
    wire [DATA_WIDTH-1:0]  rf_read_data [0:7];
    wire [3:0]             rf_write_en;
    wire [13:0]            rf_write_addr [0:3];
    wire [DATA_WIDTH-1:0]  rf_write_data [0:3];
    wire                   rf_bank_conflict;
    wire [7:0]             rf_read_stall;

    //--------------------------------------------------------------------------
    // CUDA core arrays
    // 64 cores organized as 2 groups of 32 (one per half-warp)
    //--------------------------------------------------------------------------
    wire [DATA_WIDTH-1:0]  core_result [0:TOTAL_CORES-1];
    wire [TOTAL_CORES-1:0] core_valid_out;
    wire [TOTAL_CORES-1:0] core_busy;
    wire [TOTAL_CORES-1:0] core_ready;

    reg [DATA_WIDTH-1:0]  core_op_a [0:TOTAL_CORES-1];
    reg [DATA_WIDTH-1:0]  core_op_b [0:TOTAL_CORES-1];
    reg [DATA_WIDTH-1:0]  core_op_c [0:TOTAL_CORES-1];
    reg [1:0]             core_opcode [0:TOTAL_CORES-1];
    reg [TOTAL_CORES-1:0] core_valid_in;

    //--------------------------------------------------------------------------
    // Shared memory connections
    //--------------------------------------------------------------------------
    wire [127:0]           sm_rdata0, sm_rdata1, sm_rdata2;
    wire                   sm_conflict;
    wire [2:0]             sm_conflict_ports;

    reg                    sm_en0, sm_en1, sm_en2;
    reg                    sm_we0, sm_we1;
    reg [7:0]              sm_addr0, sm_addr1, sm_addr2;
    reg [127:0]            sm_wdata0, sm_wdata1;

    //--------------------------------------------------------------------------
    // Tensor core connections
    //--------------------------------------------------------------------------
    wire [15:0]            tc_data_in;
    wire [31:0]            tc_data_out;
    wire                   tc_valid_out;
    wire                   tc_busy;
    wire [4:0]             tc_cycle;

    reg [15:0]             tc_data_in_reg;
    reg [7:0]              tc_row_addr;
    reg [7:0]              tc_col_addr;
    reg [2:0]              tc_op_mode;
    reg                    tc_valid_in;

    //--------------------------------------------------------------------------
    // Scoreboard for register dependencies
    //--------------------------------------------------------------------------
    reg [63:0]             scoreboard;  // Track destination registers in flight
    reg [5:0]              pending_dest [0:7];  // Up to 8 pending writes
    reg [3:0]              pending_count;

    //--------------------------------------------------------------------------
    // State machine
    //--------------------------------------------------------------------------
    localparam SM_IDLE     = 4'b0000;
    localparam SM_FETCH    = 4'b0001;
    localparam SM_DECODE   = 4'b0010;
    localparam SM_ISSUE    = 4'b0011;
    localparam SM_EXECUTE  = 4'b0100;
    localparam SM_WRITEBACK = 4'b0101;
    localparam SM_BARRIER  = 4'b0110;
    localparam SM_WAIT_MEM = 4'b0111;

    reg [3:0] sm_state;

    // Instruction format: [31:26] opcode, [25:20] dest, [19:14] src_a, [13:8] src_b, [7:0] imm
    // Or for 3-source ops: [31:26] opcode, [25:20] dest, [19:14] src_a, [13:8] src_b, [7:2] src_c

    //--------------------------------------------------------------------------
    // Register file instantiation
    //--------------------------------------------------------------------------
    register_file u_register_file (
        .clk             (clk),
        .rst_n           (rst_n),
        .read_en         (rf_read_en),
        .read_addr       (rf_read_addr),
        .read_data       (rf_read_data),
        .write_en        (rf_write_en),
        .write_addr      (rf_write_addr),
        .write_data      (rf_write_data),
        .bank_conflict   (rf_bank_conflict),
        .read_stall      (rf_read_stall)
    );

    // Connect RF read ports to decode logic
    assign rf_read_en[0] = (sm_state == SM_DECODE) && decoded_valid && 
                           (decoded_src_a != 6'd0);
    assign rf_read_en[1] = (sm_state == SM_DECODE) && decoded_valid && 
                           (decoded_src_b != 6'd0);
    assign rf_read_en[2] = (sm_state == SM_DECODE) && decoded_valid && 
                           (decoded_src_c != 6'd0);
    assign rf_read_en[7:3] = 5'd0;

    // Register addresses include warp ID offset
    assign rf_read_addr[0] = {current_warp, decoded_src_a[5:0]};
    assign rf_read_addr[1] = {current_warp, decoded_src_b[5:0]};
    assign rf_read_addr[2] = {current_warp, decoded_src_c[5:0]};
    assign rf_read_addr[3] = 14'd0;
    assign rf_read_addr[4] = 14'd0;
    assign rf_read_addr[5] = 14'd0;
    assign rf_read_addr[6] = 14'd0;
    assign rf_read_addr[7] = 14'd0;

    // Write ports
    assign rf_write_en = 4'b0000;  // Will be driven by writeback logic
    assign rf_write_addr[0] = 14'd0;
    assign rf_write_addr[1] = 14'd0;
    assign rf_write_addr[2] = 14'd0;
    assign rf_write_addr[3] = 14'd0;
    assign rf_write_data[0] = 32'd0;
    assign rf_write_data[1] = 32'd0;
    assign rf_write_data[2] = 32'd0;
    assign rf_write_data[3] = 32'd0;

    //--------------------------------------------------------------------------
    // Shared memory instantiation
    //--------------------------------------------------------------------------
    shared_memory u_shared_memory (
        .clk             (clk),
        .rst_n           (rst_n),
        .port0_en        (sm_en0),
        .port0_we        (sm_we0),
        .port0_addr      (sm_addr0),
        .port0_wdata     (sm_wdata0),
        .port0_rdata     (sm_rdata0),
        .port1_en        (sm_en1),
        .port1_we        (sm_we1),
        .port1_addr      (sm_addr1),
        .port1_wdata     (sm_wdata1),
        .port1_rdata     (sm_rdata1),
        .port2_en        (sm_en2),
        .port2_addr      (sm_addr2),
        .port2_rdata     (sm_rdata2),
        .bank_conflict   (sm_conflict),
        .conflict_ports  (sm_conflict_ports)
    );

    //--------------------------------------------------------------------------
    // Tensor core instantiation
    //--------------------------------------------------------------------------
    tensor_core u_tensor_core (
        .clk             (clk),
        .rst_n           (rst_n),
        .valid_in        (tc_valid_in),
        .ready_out       (),
        .op_mode         (tc_op_mode),
        .data_in         (tc_data_in_reg),
        .row_addr        (tc_row_addr),
        .col_addr        (tc_col_addr),
        .data_out        (tc_data_out),
        .valid_out       (tc_valid_out),
        .busy_out        (tc_busy),
        .cycle_counter   (tc_cycle)
    );
    assign tensor_busy = tc_busy;

    //--------------------------------------------------------------------------
    // CUDA core array instantiation
    //--------------------------------------------------------------------------
    genvar ci;
    generate
        for (ci = 0; ci < TOTAL_CORES; ci = ci + 1) begin : cuda_core_gen
            cuda_core #(
                .DATA_WIDTH(DATA_WIDTH)
            ) u_cuda_core (
                .clk           (clk),
                .rst_n         (rst_n),
                .valid_in      (core_valid_in[ci]),
                .ready_out     (core_ready[ci]),
                .op_code       (core_opcode[ci]),
                .operand_a     (core_op_a[ci]),
                .operand_b     (core_op_b[ci]),
                .operand_c     (core_op_c[ci]),
                .result_out    (core_result[ci]),
                .valid_out     (core_valid_out[ci]),
                .busy_out      (core_busy[ci]),
                .overflow_flag (),
                .underflow_flag(),
                .invalid_flag  ()
            );
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Instruction fetch
    //--------------------------------------------------------------------------
    assign instruction_ready = (sm_state == SM_IDLE) || (sm_state == SM_FETCH);

    //--------------------------------------------------------------------------
    // Warp scheduler - round-robin
    //--------------------------------------------------------------------------
    always @(*) begin
        next_warp = current_warp;
        if (warp_schedule_cnt == 3'd7) begin
            // Find next active warp
            case (current_warp)
                2'd0: next_warp = warp_active[1] ? 2'd1 : (warp_active[2] ? 2'd2 : (warp_active[3] ? 2'd3 : 2'd0));
                2'd1: next_warp = warp_active[2] ? 2'd2 : (warp_active[3] ? 2'd3 : (warp_active[0] ? 2'd0 : 2'd1));
                2'd2: next_warp = warp_active[3] ? 2'd3 : (warp_active[0] ? 2'd0 : (warp_active[1] ? 2'd1 : 2'd2));
                2'd3: next_warp = warp_active[0] ? 2'd0 : (warp_active[1] ? 2'd1 : (warp_active[2] ? 2'd2 : 2'd3));
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // Main state machine
    //--------------------------------------------------------------------------
    integer ti;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sm_state            <= SM_IDLE;
            current_warp        <= {WARP_ID_W{1'b0}};
            warp_schedule_cnt   <= 3'd0;
            warp_active         <= {WARPS_PER_SM{1'b0}};
            sm_busy             <= 1'b0;
            active_warps        <= 8'd0;
            inst_count          <= 16'd0;
            decoded_valid       <= 1'b0;
            pc_out              <= 32'd0;
            mem_req_valid       <= 1'b0;
            mem_req_addr        <= 32'd0;
            mem_req_data        <= {DATA_WIDTH{1'b0}};
            mem_req_we          <= 1'b0;
            mem_req_be          <= 4'd0;
            scoreboard          <= 64'd0;
            pending_count       <= 4'd0;
            tc_valid_in         <= 1'b0;
            sm_en0              <= 1'b0;
            sm_en1              <= 1'b0;
            sm_en2              <= 1'b0;
            sm_we0              <= 1'b0;
            sm_we1              <= 1'b0;

            for (ti = 0; ti < WARPS_PER_SM; ti = ti + 1) begin
                warp_pc[ti]             <= 32'd0;
                warp_waiting_barrier[ti]<= 1'b0;
                warp_waiting_memory[ti] <= 1'b0;
                thread_active[ti]       <= {THREADS_PER_WARP{1'b0}};
            end

            for (ti = 0; ti < TOTAL_CORES; ti = ti + 1) begin
                core_valid_in[ti] <= 1'b0;
                core_op_a[ti]     <= {DATA_WIDTH{1'b0}};
                core_op_b[ti]     <= {DATA_WIDTH{1'b0}};
                core_op_c[ti]     <= {DATA_WIDTH{1'b0}};
                core_opcode[ti]   <= 2'b00;
            end
        end else begin
            // Default: clear single-cycle signals
            mem_req_valid <= 1'b0;
            tc_valid_in   <= 1'b0;
            sm_en0        <= 1'b0;
            sm_en1        <= 1'b0;
            sm_en2        <= 1'b0;

            case (sm_state)
                SM_IDLE: begin
                    sm_busy <= 1'b0;
                    if (sm_enable && instruction_valid) begin
                        sm_busy <= 1'b1;
                        sm_state <= SM_FETCH;
                        // Initialize first warp
                        warp_active[0] <= 1'b1;
                        warp_pc[0]     <= pc_in;
                        for (ti = 0; ti < THREADS_PER_WARP; ti = ti + 1) begin
                            thread_active[0][ti] <= 1'b1;
                        end
                    end
                end

                SM_FETCH: begin
                    if (instruction_valid) begin
                        decoded_inst <= instruction_in;
                        decoded_valid <= 1'b1;
                        sm_state <= SM_DECODE;
                    end else begin
                        // No instruction - try next warp
                        warp_schedule_cnt <= warp_schedule_cnt + 1'b1;
                        current_warp <= next_warp;
                    end
                end

                SM_DECODE: begin
                    if (decoded_valid) begin
                        // Decode instruction fields
                        decoded_opcode <= decoded_inst[31:26];
                        decoded_dest   <= decoded_inst[25:20];
                        decoded_src_a  <= decoded_inst[19:14];
                        decoded_src_b  <= decoded_inst[13:8];
                        decoded_src_c  <= decoded_inst[7:2];
                        decoded_imm    <= decoded_inst[15:0];

                        // Check scoreboard for hazards
                        if (scoreboard[decoded_dest] ||
                            (decoded_src_a != 6'd0 && scoreboard[decoded_src_a]) ||
                            (decoded_src_b != 6'd0 && scoreboard[decoded_src_b]) ||
                            (decoded_src_c != 6'd0 && scoreboard[decoded_src_c])) begin
                            // Hazard detected - stall
                            sm_state <= SM_FETCH;
                        end else begin
                            sm_state <= SM_ISSUE;
                        end
                    end
                end

                SM_ISSUE: begin
                    inst_count <= inst_count + 1'b1;

                    case (decoded_opcode)
                        OPCODE_ADD, OPCODE_MUL, OPCODE_FMA: begin
                            // Issue to CUDA cores for all active threads
                            // Using first 32 cores for warp threads
                            for (ti = 0; ti < THREADS_PER_WARP; ti = ti + 1) begin
                                if (thread_active[current_warp][ti]) begin
                                    core_valid_in[ti] <= 1'b1;
                                    core_op_a[ti]     <= rf_read_data[0];
                                    core_op_b[ti]     <= rf_read_data[1];
                                    core_op_c[ti]     <= rf_read_data[2];
                                    case (decoded_opcode)
                                        OPCODE_ADD: core_opcode[ti] <= 2'b00;
                                        OPCODE_MUL: core_opcode[ti] <= 2'b01;
                                        OPCODE_FMA: core_opcode[ti] <= 2'b10;
                                        default:    core_opcode[ti] <= 2'b00;
                                    endcase
                                end
                            end
                            // Mark destination as pending
                            scoreboard[decoded_dest] <= 1'b1;
                            sm_state <= SM_EXECUTE;
                        end

                        OPCODE_LD: begin
                            // Load from memory
                            mem_req_valid <= 1'b1;
                            mem_req_addr  <= rf_read_data[0] + {{16{decoded_imm[15]}}, decoded_imm};
                            mem_req_we    <= 1'b0;
                            mem_req_be    <= 4'b1111;
                            warp_waiting_memory[current_warp] <= 1'b1;
                            sm_state <= SM_WAIT_MEM;
                        end

                        OPCODE_ST: begin
                            // Store to memory
                            mem_req_valid <= 1'b1;
                            mem_req_addr  <= rf_read_data[0] + {{16{decoded_imm[15]}}, decoded_imm};
                            mem_req_data  <= rf_read_data[1];
                            mem_req_we    <= 1'b1;
                            mem_req_be    <= 4'b1111;
                            sm_state <= SM_FETCH;
                            warp_pc[current_warp] <= warp_pc[current_warp] + 4;
                        end

                        OPCODE_MMA: begin
                            // Issue tensor core operation
                            tc_valid_in   <= tensor_op_valid;
                            tc_op_mode    <= tensor_op_mode;
                            sm_state      <= SM_EXECUTE;
                        end

                        OPCODE_BAR: begin
                            // Barrier synchronization
                            warp_waiting_barrier[current_warp] <= 1'b1;
                            sm_state <= SM_BARRIER;
                        end

                        OPCODE_BRA: begin
                            // Branch
                            warp_pc[current_warp] <= warp_pc[current_warp] + 
                                                      {{14{decoded_imm[15]}}, decoded_imm, 2'b00};
                            sm_state <= SM_FETCH;
                        end

                        OPCODE_EXIT: begin
                            warp_active[current_warp] <= 1'b0;
                            thread_active[current_warp] <= {THREADS_PER_WARP{1'b0}};
                            sm_state <= SM_FETCH;
                        end

                        default: begin
                            sm_state <= SM_FETCH;
                            warp_pc[current_warp] <= warp_pc[current_warp] + 4;
                        end
                    endcase
                end

                SM_EXECUTE: begin
                    // Clear core valid signals after one cycle
                    for (ti = 0; ti < TOTAL_CORES; ti = ti + 1) begin
                        core_valid_in[ti] <= 1'b0;
                    end
                    tc_valid_in <= 1'b0;

                    // Wait for execution to complete, then writeback
                    sm_state <= SM_WRITEBACK;
                end

                SM_WRITEBACK: begin
                    // Collect results and write back to register file
                    // Simplified: assume single-cycle writeback for ALU ops
                    // In reality, would need to match latency of CUDA cores
                    case (decoded_opcode)
                        OPCODE_ADD, OPCODE_MUL, OPCODE_FMA: begin
                            // Write first thread result (simplified)
                            // Full implementation would write all thread results
                            scoreboard[decoded_dest] <= 1'b0;
                        end
                        OPCODE_MMA: begin
                            if (!tc_busy) begin
                                sm_state <= SM_FETCH;
                                warp_pc[current_warp] <= warp_pc[current_warp] + 4;
                            end
                        end
                        default: begin
                            scoreboard[decoded_dest] <= 1'b0;
                        end
                    endcase

                    if (decoded_opcode != OPCODE_MMA) begin
                        sm_state <= SM_FETCH;
                        warp_pc[current_warp] <= warp_pc[current_warp] + 4;
                    end
                end

                SM_BARRIER: begin
                    // Wait for all warps to reach barrier
                    if (&warp_waiting_barrier && warp_active == {WARPS_PER_SM{1'b1}}) begin
                        warp_waiting_barrier <= {WARPS_PER_SM{1'b0}};
                        sm_state <= SM_FETCH;
                        warp_pc[current_warp] <= warp_pc[current_warp] + 4;
                    end else begin
                        // Try next warp
                        warp_schedule_cnt <= warp_schedule_cnt + 1'b1;
                        current_warp <= next_warp;
                        sm_state <= SM_FETCH;
                    end
                end

                SM_WAIT_MEM: begin
                    if (mem_resp_valid) begin
                        warp_waiting_memory[current_warp] <= 1'b0;
                        sm_state <= SM_WRITEBACK;
                        // Result would be written to register file
                    end else begin
                        // Try next warp while waiting
                        warp_schedule_cnt <= warp_schedule_cnt + 1'b1;
                        current_warp <= next_warp;
                        sm_state <= SM_FETCH;
                    end
                end

                default: sm_state <= SM_IDLE;
            endcase

            // Update active warps status
            active_warps <= {4'd0, warp_active};

            // Check if all warps completed
            if (warp_active == {WARPS_PER_SM{1'b0}} && sm_state != SM_IDLE) begin
                sm_state <= SM_IDLE;
                sm_busy  <= 1'b0;
            end
        end
    end

endmodule
