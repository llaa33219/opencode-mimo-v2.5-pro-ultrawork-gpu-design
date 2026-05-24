//------------------------------------------------------------------------------
// Module: tensor_core
// Description: 16x8 Matrix Multiply-Accumulate (MMA) unit
//              Input: FP16 (16-bit), Output: FP32 (32-bit)
//              Computes D = A x B + C where A is 16x16, B is 16x8, C/D are 16x8
//              Weight-stationary dataflow
//              Latency: 16 cycles for full matrix
//------------------------------------------------------------------------------

module tensor_core #(
    parameter FP16_WIDTH = 16,
    parameter FP32_WIDTH = 32,
    parameter MMA_M      = 16,   // Rows of A, Rows of C/D
    parameter MMA_K      = 16,   // Cols of A, Rows of B
    parameter MMA_N      = 8     // Cols of B, Cols of C/D
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // Control interface
    input  wire                      valid_in,
    output wire                      ready_out,
    input  wire [2:0]                op_mode,       // 000=LOAD_A, 001=LOAD_B, 010=LOAD_C, 011=EXECUTE, 100=STORE_D

    // Data interface - FP16 inputs, FP32 accumulator
    input  wire [FP16_WIDTH-1:0]     data_in,       // Sequential element input
    input  wire [7:0]                row_addr,      // Row address for loading
    input  wire [7:0]                col_addr,      // Column address for loading

    output reg  [FP32_WIDTH-1:0]     data_out,      // Sequential element output
    output reg                       valid_out,
    output reg                       busy_out,

    // Status
    output reg  [4:0]                cycle_counter   // Current execution cycle
);

    //--------------------------------------------------------------------------
    // Local parameters and opcodes
    //--------------------------------------------------------------------------
    localparam OP_LOAD_A = 3'b000;
    localparam OP_LOAD_B = 3'b001;
    localparam OP_LOAD_C = 3'b010;
    localparam OP_EXEC   = 3'b011;
    localparam OP_STORE_D = 3'b100;

    // Total elements
    localparam A_ELEMENTS = MMA_M * MMA_K;  // 256
    localparam B_ELEMENTS = MMA_K * MMA_N;  // 128
    localparam C_ELEMENTS = MMA_M * MMA_N;  // 128
    localparam D_ELEMENTS = MMA_M * MMA_N;  // 128

    // Address widths
    localparam A_ADDR_W = 8;  // log2(256)
    localparam B_ADDR_W = 7;  // log2(128)
    localparam C_ADDR_W = 7;  // log2(128)
    localparam D_ADDR_W = 7;  // log2(128)

    //--------------------------------------------------------------------------
    // Matrix storage arrays
    //--------------------------------------------------------------------------
    // Matrix A: 16x16 FP16 - weight stationary
    reg [FP16_WIDTH-1:0] matrix_a [0:A_ELEMENTS-1];

    // Matrix B: 16x8 FP16
    reg [FP16_WIDTH-1:0] matrix_b [0:B_ELEMENTS-1];

    // Matrix C: 16x8 FP32 accumulator
    reg [FP32_WIDTH-1:0] matrix_c [0:C_ELEMENTS-1];

    // Matrix D: 16x8 FP32 result
    reg [FP32_WIDTH-1:0] matrix_d [0:D_ELEMENTS-1];

    //--------------------------------------------------------------------------
    // State machine
    //--------------------------------------------------------------------------
    localparam STATE_IDLE    = 4'b0000;
    localparam STATE_LOAD_A  = 4'b0001;
    localparam STATE_LOAD_B  = 4'b0010;
    localparam STATE_LOAD_C  = 4'b0011;
    localparam STATE_EXEC    = 4'b0100;
    localparam STATE_STORE_D = 4'b0101;
    localparam STATE_DONE    = 4'b0110;

    reg [3:0] state_reg, state_next;
    reg [A_ADDR_W-1:0] load_cnt;
    reg [D_ADDR_W-1:0] store_cnt;

    // Execution state
    reg [3:0]  exec_row;       // Current row of D being computed (0-15)
    reg [2:0]  exec_col;       // Current col of D being computed (0-7)
    reg [4:0]  exec_k;         // Current K dimension (0-15)
    reg        exec_active;

    // FP16 to FP32 conversion for multiplication
    wire [FP32_WIDTH-1:0] fp16_to_fp32_a;
    wire [FP32_WIDTH-1:0] fp16_to_fp32_b;

    // FP32 MAC unit
    reg  [FP32_WIDTH-1:0] mac_accum;
    reg  [FP32_WIDTH-1:0] mac_product;
    wire [FP32_WIDTH-1:0] mac_sum;

    // Output buffering
    reg [D_ADDR_W-1:0] out_addr;

    assign ready_out = (state_reg == STATE_IDLE);

    //--------------------------------------------------------------------------
    // FP16 to FP32 conversion helper
    // FP16: [15] sign, [14:10] exp (5-bit, bias 15), [9:0] mant (10-bit)
    // FP32: [31] sign, [30:23] exp (8-bit, bias 127), [22:0] mant (23-bit)
    //--------------------------------------------------------------------------
    function [FP32_WIDTH-1:0] fp16_to_fp32;
        input [FP16_WIDTH-1:0] fp16;
        reg sign;
        reg [4:0] exp16;
        reg [9:0] mant16;
        reg [7:0] exp32;
        reg [22:0] mant32;
        begin
            sign   = fp16[15];
            exp16  = fp16[14:10];
            mant16 = fp16[9:0];

            if (exp16 == 5'b00000) begin
                // Zero or denormal - treat as zero for simplicity
                exp32  = 8'd0;
                mant32 = 23'd0;
            end else if (exp16 == 5'b11111) begin
                // Inf or NaN
                exp32  = 8'hFF;
                mant32 = {mant16, 13'd0};
            end else begin
                // Normal number
                exp32  = {3'd0, exp16} + 8'd112; // Adjust bias: 127 - 15 = 112
                mant32 = {mant16, 13'd0};
            end

            fp16_to_fp32 = {sign, exp32, mant32};
        end
    endfunction

    //--------------------------------------------------------------------------
    // Simplified FP32 multiplication (combinational)
    // Product = fp32_a * fp32_b
    //--------------------------------------------------------------------------
    function [FP32_WIDTH-1:0] fp32_mul;
        input [FP32_WIDTH-1:0] a;
        input [FP32_WIDTH-1:0] b;
        reg        sign;
        reg [7:0]  exp;
        reg [22:0] mant;
        reg [47:0] full_mant;
        reg [8:0]  exp_sum;
        begin
            sign = a[31] ^ b[31];
            exp_sum = {1'b0, a[30:23]} + {1'b0, b[30:23]} - 9'd127;
            full_mant = {1'b1, a[22:0]} * {1'b1, b[22:0]};

            if (a[30:23] == 8'hFF || b[30:23] == 8'hFF) begin
                // NaN or Inf propagation
                exp = 8'hFF;
                mant = 23'd0;
            end else if (a[30:0] == 0 || b[30:0] == 0) begin
                exp = 8'd0;
                mant = 23'd0;
            end else if (full_mant[47]) begin
                exp = exp_sum[7:0] + 8'd1;
                mant = full_mant[46:24];
            end else begin
                exp = exp_sum[7:0];
                mant = full_mant[45:23];
            end

            fp32_mul = {sign, exp, mant};
        end
    endfunction

    //--------------------------------------------------------------------------
    // Simplified FP32 addition (combinational)
    // Sum = a + b
    //--------------------------------------------------------------------------
    function [FP32_WIDTH-1:0] fp32_add;
        input [FP32_WIDTH-1:0] a;
        input [FP32_WIDTH-1:0] b;
        reg [7:0]  exp_diff;
        reg [23:0] mant_a, mant_b;
        reg [24:0] mant_sum;
        reg [7:0]  exp_max;
        reg        sign_out;
        begin
            if (a[30:23] >= b[30:23]) begin
                exp_max = a[30:23];
                exp_diff = a[30:23] - b[30:23];
                mant_a = {1'b1, a[22:0]};
                mant_b = {1'b1, b[22:0]} >> exp_diff;
                if (a[31] == b[31]) begin
                    mant_sum = mant_a + mant_b;
                    sign_out = a[31];
                end else begin
                    if (mant_a >= mant_b) begin
                        mant_sum = mant_a - mant_b;
                        sign_out = a[31];
                    end else begin
                        mant_sum = mant_b - mant_a;
                        sign_out = b[31];
                    end
                end
            end else begin
                exp_max = b[30:23];
                exp_diff = b[30:23] - a[30:23];
                mant_a = {1'b1, b[22:0]};
                mant_b = {1'b1, a[22:0]} >> exp_diff;
                if (a[31] == b[31]) begin
                    mant_sum = mant_a + mant_b;
                    sign_out = b[31];
                end else begin
                    if (mant_a >= mant_b) begin
                        mant_sum = mant_a - mant_b;
                        sign_out = b[31];
                    end else begin
                        mant_sum = mant_b - mant_a;
                        sign_out = a[31];
                    end
                end
            end

            if (mant_sum[24]) begin
                fp32_add = {sign_out, exp_max + 8'd1, mant_sum[23:1]};
            end else begin
                fp32_add = {sign_out, exp_max, mant_sum[22:0]};
            end
        end
    endfunction

    assign fp16_to_fp32_a = fp16_to_fp32(matrix_a[exec_row * MMA_K + exec_k]);
    assign fp16_to_fp32_b = fp16_to_fp32(matrix_b[exec_k * MMA_N + exec_col]);

    // MAC operation: accum + (A * B)
    always @(*) begin
        mac_product = fp32_mul(fp16_to_fp32_a, fp16_to_fp32_b);
        mac_sum     = fp32_add(mac_accum, mac_product);
    end

    //--------------------------------------------------------------------------
    // Sequential logic - State machine and execution
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg   <= STATE_IDLE;
            load_cnt    <= {A_ADDR_W{1'b0}};
            store_cnt   <= {D_ADDR_W{1'b0}};
            exec_row    <= 4'd0;
            exec_col    <= 3'd0;
            exec_k      <= 5'd0;
            exec_active <= 1'b0;
            mac_accum   <= {FP32_WIDTH{1'b0}};
            valid_out   <= 1'b0;
            busy_out    <= 1'b0;
            cycle_counter <= 5'd0;
            data_out    <= {FP32_WIDTH{1'b0}};
            out_addr    <= {D_ADDR_W{1'b0}};
        end else begin
            valid_out <= 1'b0;

            case (state_reg)
                STATE_IDLE: begin
                    busy_out <= 1'b0;
                    if (valid_in) begin
                        busy_out <= 1'b1;
                        case (op_mode)
                            OP_LOAD_A: begin
                                state_reg <= STATE_LOAD_A;
                                load_cnt  <= {A_ADDR_W{1'b0}};
                            end
                            OP_LOAD_B: begin
                                state_reg <= STATE_LOAD_B;
                                load_cnt  <= {B_ADDR_W{1'b0}};
                            end
                            OP_LOAD_C: begin
                                state_reg <= STATE_LOAD_C;
                                load_cnt  <= {C_ADDR_W{1'b0}};
                            end
                            OP_EXEC: begin
                                state_reg <= STATE_EXEC;
                                exec_row  <= 4'd0;
                                exec_col  <= 3'd0;
                                exec_k    <= 5'd0;
                                exec_active <= 1'b1;
                                cycle_counter <= 5'd0;
                            end
                            OP_STORE_D: begin
                                state_reg <= STATE_STORE_D;
                                store_cnt <= {D_ADDR_W{1'b0}};
                            end
                            default: state_reg <= STATE_IDLE;
                        endcase
                    end
                end

                STATE_LOAD_A: begin
                    busy_out <= 1'b1;
                    if (valid_in) begin
                        matrix_a[load_cnt] <= data_in;
                        if (load_cnt == A_ELEMENTS - 1) begin
                            state_reg <= STATE_IDLE;
                            busy_out  <= 1'b0;
                        end else begin
                            load_cnt <= load_cnt + 1'b1;
                        end
                    end
                end

                STATE_LOAD_B: begin
                    busy_out <= 1'b1;
                    if (valid_in) begin
                        matrix_b[load_cnt[B_ADDR_W-1:0]] <= data_in;
                        if (load_cnt[B_ADDR_W-1:0] == B_ELEMENTS - 1) begin
                            state_reg <= STATE_IDLE;
                            busy_out  <= 1'b0;
                        end else begin
                            load_cnt <= load_cnt + 1'b1;
                        end
                    end
                end

                STATE_LOAD_C: begin
                    busy_out <= 1'b1;
                    // Load C as 32-bit words (two 16-bit cycles)
                    // Simplified: assume data_in carries lower 16 bits first
                    if (valid_in) begin
                        if (load_cnt[0] == 1'b0) begin
                            // First half - store temporarily in matrix_d as buffer
                            matrix_d[load_cnt[C_ADDR_W:1]] <= {16'd0, data_in};
                        end else begin
                            // Second half - combine
                            matrix_c[load_cnt[C_ADDR_W:1]] <= {data_in, matrix_d[load_cnt[C_ADDR_W:1]][15:0]};
                        end
                        if (load_cnt == (C_ELEMENTS * 2) - 1) begin
                            state_reg <= STATE_IDLE;
                            busy_out  <= 1'b0;
                        end else begin
                            load_cnt <= load_cnt + 1'b1;
                        end
                    end
                end

                STATE_EXEC: begin
                    busy_out <= 1'b1;
                    cycle_counter <= cycle_counter + 1'b1;

                    if (exec_active) begin
                        // Initialize accumulator at start of K loop
                        if (exec_k == 5'd0) begin
                            mac_accum <= matrix_c[exec_row * MMA_N + exec_col];
                        end else begin
                            mac_accum <= mac_sum;
                        end

                        // Advance K
                        if (exec_k == MMA_K - 1) begin
                            // Store result and advance
                            matrix_d[exec_row * MMA_N + exec_col] <= mac_sum;
                            exec_k <= 5'd0;

                            if (exec_col == MMA_N - 1) begin
                                exec_col <= 3'd0;
                                if (exec_row == MMA_M - 1) begin
                                    exec_active <= 1'b0;
                                    state_reg   <= STATE_DONE;
                                end else begin
                                    exec_row <= exec_row + 1'b1;
                                end
                            end else begin
                                exec_col <= exec_col + 1'b1;
                            end
                        end else begin
                            exec_k <= exec_k + 1'b1;
                        end
                    end
                end

                STATE_STORE_D: begin
                    busy_out <= 1'b1;
                    data_out  <= matrix_d[store_cnt];
                    valid_out <= 1'b1;
                    out_addr  <= store_cnt;
                    if (store_cnt == D_ELEMENTS - 1) begin
                        state_reg <= STATE_IDLE;
                        busy_out  <= 1'b0;
                    end else begin
                        store_cnt <= store_cnt + 1'b1;
                    end
                end

                STATE_DONE: begin
                    busy_out <= 1'b0;
                    valid_out <= 1'b1;  // Signal completion
                    state_reg <= STATE_IDLE;
                end

                default: state_reg <= STATE_IDLE;
            endcase
        end
    end

endmodule
