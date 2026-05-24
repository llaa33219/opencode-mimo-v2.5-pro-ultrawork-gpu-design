//------------------------------------------------------------------------------
// Module: cache_controller
// Description: Manages L1-L2-DRAM cache hierarchy
//              Handles cache misses, writebacks, and prefetching
//              Implements MOESI coherence protocol
//------------------------------------------------------------------------------

module cache_controller #(
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 256,
    parameter NUM_SMS         = 4,
    parameter L1_LINE_SIZE    = 32,       // 32 bytes
    parameter L2_LINE_SIZE    = 64,       // 64 bytes
    parameter PREFETCH_DEPTH  = 8
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // L1 request interfaces (from SMs)
    input  wire [NUM_SMS-1:0]          l1_req_valid,
    input  wire [NUM_SMS-1:0]          l1_req_we,
    input  wire [ADDR_WIDTH-1:0]       l1_req_addr [0:NUM_SMS-1],
    input  wire [DATA_WIDTH-1:0]       l1_req_wdata [0:NUM_SMS-1],
    input  wire [(DATA_WIDTH/8)-1:0]   l1_req_be [0:NUM_SMS-1],
    output reg  [NUM_SMS-1:0]          l1_req_ready,

    // L1 response interfaces (to SMs)
    output reg  [NUM_SMS-1:0]          l1_resp_valid,
    output reg  [DATA_WIDTH-1:0]       l1_resp_rdata [0:NUM_SMS-1],

    // L2 interface
    output reg                         l2_req_valid,
    output reg                         l2_req_we,
    output reg  [ADDR_WIDTH-1:0]       l2_req_addr,
    output reg  [DATA_WIDTH-1:0]       l2_req_wdata,
    output reg  [(DATA_WIDTH/8)-1:0]   l2_req_be,
    input  wire                        l2_req_ready,

    input  wire                        l2_resp_valid,
    input  wire [DATA_WIDTH-1:0]       l2_resp_rdata,
    input  wire                        l2_resp_miss,

    // DRAM interface
    output reg                         dram_req_valid,
    output reg  [ADDR_WIDTH-1:0]       dram_req_addr,
    output reg                         dram_req_we,
    output reg  [L2_LINE_SIZE*8-1:0]   dram_req_wdata,
    input  wire                        dram_req_ready,

    input  wire                        dram_resp_valid,
    input  wire [L2_LINE_SIZE*8-1:0]   dram_resp_rdata,

    // Coherency interface (broadcast)
    output reg                         coh_broadcast_valid,
    output reg  [ADDR_WIDTH-1:0]       coh_broadcast_addr,
    output reg  [2:0]                  coh_broadcast_type,  // INVALIDATE, UPDATE, etc.
    input  wire [NUM_SMS-1:0]          coh_broadcast_ack
);

    //--------------------------------------------------------------------------
    // MOESI state encoding
    //--------------------------------------------------------------------------
    localparam MOESI_INVALID     = 3'b000;
    localparam MOESI_OWNED       = 3'b001;
    localparam MOESI_EXCLUSIVE   = 3'b010;
    localparam MOESI_SHARED      = 3'b011;
    localparam MOESI_MODIFIED    = 3'b100;

    // Coherence message types
    localparam COH_INVALIDATE    = 3'b000;
    localparam COH_READ_SHARED   = 3'b001;
    localparam COH_READ_EXCLUSIVE= 3'b010;
    localparam COH_WRITE_BACK    = 3'b011;

    //--------------------------------------------------------------------------
    // Directory tracking (simplified: track coherence state per SM)
    // For each cache line, track which SMs have which state
    // Using a direct-mapped structure for area efficiency
    //--------------------------------------------------------------------------
    localparam DIR_ENTRIES = 512;  // Simplified directory
    localparam DIR_ADDR_W  = 9;
    localparam DIR_TAG_W   = ADDR_WIDTH - DIR_ADDR_W - $clog2(L2_LINE_SIZE);

    reg [DIR_TAG_W-1:0] dir_tag [0:DIR_ENTRIES-1];
    reg [NUM_SMS-1:0]   dir_shared [0:DIR_ENTRIES-1];  // SMs in Shared state
    reg [NUM_SMS-1:0]   dir_owned  [0:DIR_ENTRIES-1];  // SM in Owned/Modified state
    reg                 dir_valid  [0:DIR_ENTRIES-1];

    //--------------------------------------------------------------------------
    // Request arbitration
    //--------------------------------------------------------------------------
    reg [NUM_SMS-1:0] req_pending;
    reg [$clog2(NUM_SMS)-1:0] selected_sm;
    reg [$clog2(NUM_SMS)-1:0] rr_arbiter;

    // Round-robin arbitration
    integer si;
    always @(*) begin
        selected_sm = rr_arbiter;
        for (si = 0; si < NUM_SMS; si = si + 1) begin
            if (l1_req_valid[si] && l1_req_ready[si]) begin
                selected_sm = si[$clog2(NUM_SMS)-1:0];
            end
        end
    end

    //--------------------------------------------------------------------------
    // Prefetcher
    //--------------------------------------------------------------------------
    reg [ADDR_WIDTH-1:0] prefetch_queue [0:PREFETCH_DEPTH-1];
    reg                  prefetch_valid [0:PREFETCH_DEPTH-1];
    reg [ADDR_WIDTH-1:0] last_access_addr;
    reg [ADDR_WIDTH-1:0] stride;
    reg                  stride_detected;

    // Prefetch address generation
    reg [ADDR_WIDTH-1:0] next_prefetch_addr;

    always @(*) begin
        if (stride_detected) begin
            next_prefetch_addr = last_access_addr + stride;
        end else begin
            next_prefetch_addr = last_access_addr + L2_LINE_SIZE;
        end
    end

    //--------------------------------------------------------------------------
    // State machine
    //--------------------------------------------------------------------------
    localparam STATE_IDLE          = 4'b0000;
    localparam STATE_ARBITRATE     = 4'b0001;
    localparam STATE_L2_LOOKUP     = 4'b0010;
    localparam STATE_L2_RESP       = 4'b0011;
    localparam STATE_COHERENCE     = 4'b0100;
    localparam STATE_DRAM_REQ      = 4'b0101;
    localparam STATE_DRAM_RESP     = 4'b0110;
    localparam STATE_WRITEBACK     = 4'b0111;
    localparam STATE_PREFETCH      = 4'b1000;
    localparam STATE_RESPOND       = 4'b1001;

    reg [3:0] state;
    reg [NUM_SMS-1:0] active_sm;
    reg [ADDR_WIDTH-1:0] current_addr;
    reg                  current_we;
    reg [DATA_WIDTH-1:0] current_wdata;
    reg [(DATA_WIDTH/8)-1:0] current_be;
    reg [2:0]            current_coh_state;

    //--------------------------------------------------------------------------
    // Directory lookup
    //--------------------------------------------------------------------------
    wire [DIR_ADDR_W-1:0] dir_index = current_addr[DIR_ADDR_W+$clog2(L2_LINE_SIZE)-1:$clog2(L2_LINE_SIZE)];
    wire [DIR_TAG_W-1:0]  dir_lookup_tag = current_addr[ADDR_WIDTH-1:ADDR_WIDTH-DIR_TAG_W];
    wire                  dir_hit = dir_valid[dir_index] && (dir_tag[dir_index] == dir_lookup_tag);
    wire [NUM_SMS-1:0]    dir_sm_shared = dir_shared[dir_index];
    wire [NUM_SMS-1:0]    dir_sm_owned  = dir_owned[dir_index];

    //--------------------------------------------------------------------------
    // Main control state machine
    //--------------------------------------------------------------------------
    integer pi, qi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= STATE_IDLE;
            rr_arbiter       <= {$clog2(NUM_SMS){1'b0}};
            l1_req_ready     <= {NUM_SMS{1'b1}};
            l1_resp_valid    <= {NUM_SMS{1'b0}};
            l2_req_valid     <= 1'b0;
            l2_req_we        <= 1'b0;
            l2_req_addr      <= {ADDR_WIDTH{1'b0}};
            l2_req_wdata     <= {DATA_WIDTH{1'b0}};
            l2_req_be        <= {(DATA_WIDTH/8){1'b0}};
            dram_req_valid   <= 1'b0;
            dram_req_addr    <= {ADDR_WIDTH{1'b0}};
            dram_req_we      <= 1'b0;
            dram_req_wdata   <= {L2_LINE_SIZE*8{1'b0}};
            coh_broadcast_valid <= 1'b0;
            coh_broadcast_addr  <= {ADDR_WIDTH{1'b0}};
            coh_broadcast_type  <= 3'd0;
            stride_detected     <= 1'b0;
            last_access_addr    <= {ADDR_WIDTH{1'b0}};
            stride              <= {ADDR_WIDTH{1'b0}};

            for (pi = 0; pi < NUM_SMS; pi = pi + 1) begin
                l1_resp_rdata[pi] <= {DATA_WIDTH{1'b0}};
            end

            for (qi = 0; qi < PREFETCH_DEPTH; qi = qi + 1) begin
                prefetch_valid[qi] <= 1'b0;
                prefetch_queue[qi] <= {ADDR_WIDTH{1'b0}};
            end

            // Initialize directory
            for (pi = 0; pi < DIR_ENTRIES; pi = pi + 1) begin
                dir_valid[pi]  <= 1'b0;
                dir_tag[pi]    <= {DIR_TAG_W{1'b0}};
                dir_shared[pi] <= {NUM_SMS{1'b0}};
                dir_owned[pi]  <= {NUM_SMS{1'b0}};
            end
        end else begin
            // Default: clear single-cycle signals
            l2_req_valid        <= 1'b0;
            dram_req_valid      <= 1'b0;
            coh_broadcast_valid <= 1'b0;
            l1_resp_valid       <= {NUM_SMS{1'b0}};

            case (state)
                STATE_IDLE: begin
                    // Check for pending L1 requests
                    if (|l1_req_valid) begin
                        state <= STATE_ARBITRATE;
                    end else if (prefetch_valid[0]) begin
                        // Issue prefetch if no demand requests
                        state <= STATE_PREFETCH;
                    end
                end

                STATE_ARBITRATE: begin
                    // Select requesting SM via round-robin
                    active_sm <= selected_sm;
                    current_addr  <= l1_req_addr[selected_sm];
                    current_we    <= l1_req_we[selected_sm];
                    current_wdata <= l1_req_wdata[selected_sm];
                    current_be    <= l1_req_be[selected_sm];

                    // Update stride detection
                    if (last_access_addr != {ADDR_WIDTH{1'b0}}) begin
                        if (stride == {ADDR_WIDTH{1'b0}}) begin
                            stride <= current_addr - last_access_addr;
                        end else if ((current_addr - last_access_addr) == stride) begin
                            stride_detected <= 1'b1;
                        end
                    end
                    last_access_addr <= current_addr;

                    // Update round-robin pointer
                    rr_arbiter <= selected_sm + 1'b1;

                    // Block this SM until response
                    l1_req_ready[selected_sm] <= 1'b0;

                    state <= STATE_L2_LOOKUP;
                end

                STATE_L2_LOOKUP: begin
                    // Issue request to L2 cache
                    l2_req_valid <= 1'b1;
                    l2_req_addr  <= current_addr;
                    l2_req_we    <= current_we;
                    l2_req_wdata <= current_wdata;
                    l2_req_be    <= current_be;

                    if (l2_req_ready) begin
                        state <= STATE_L2_RESP;
                    end
                end

                STATE_L2_RESP: begin
                    if (l2_resp_valid) begin
                        if (!l2_resp_miss) begin
                            // L2 hit - update directory and respond
                            if (current_we) begin
                                // Write: invalidate other sharers, set as owned
                                if (dir_hit && |dir_sm_shared) begin
                                    coh_broadcast_valid <= 1'b1;
                                    coh_broadcast_addr  <= current_addr;
                                    coh_broadcast_type  <= COH_INVALIDATE;
                                    state <= STATE_COHERENCE;
                                end else begin
                                    dir_owned[dir_index]  <= (1'b1 << active_sm);
                                    dir_shared[dir_index] <= {NUM_SMS{1'b0}};
                                    state <= STATE_RESPOND;
                                end
                            end else begin
                                // Read: add to sharers
                                if (dir_hit) begin
                                    dir_shared[dir_index] <= dir_shared[dir_index] | (1'b1 << active_sm);
                                end else begin
                                    dir_tag[dir_index]    <= dir_lookup_tag;
                                    dir_valid[dir_index]  <= 1'b1;
                                    dir_shared[dir_index] <= (1'b1 << active_sm);
                                    dir_owned[dir_index]  <= {NUM_SMS{1'b0}};
                                end
                                state <= STATE_RESPOND;
                            end
                        end else begin
                            // L2 miss - need DRAM access
                            // Check directory for ownership
                            if (dir_hit && |dir_sm_owned) begin
                                // Need to writeback from owning SM first
                                coh_broadcast_valid <= 1'b1;
                                coh_broadcast_addr  <= current_addr;
                                coh_broadcast_type  <= COH_WRITE_BACK;
                                state <= STATE_COHERENCE;
                            end else begin
                                state <= STATE_DRAM_REQ;
                            end
                        end
                    end
                end

                STATE_COHERENCE: begin
                    // Wait for coherence acknowledgments
                    if (&coh_broadcast_ack) begin
                        coh_broadcast_valid <= 1'b0;
                        if (coh_broadcast_type == COH_WRITE_BACK) begin
                            state <= STATE_DRAM_REQ;
                        end else begin
                            state <= STATE_RESPOND;
                        end
                    end
                end

                STATE_DRAM_REQ: begin
                    dram_req_valid <= 1'b1;
                    dram_req_addr  <= current_addr;
                    dram_req_we    <= current_we;
                    if (current_we) begin
                        dram_req_wdata <= {current_wdata, current_wdata};  // 256->512
                    end
                    if (dram_req_ready) begin
                        dram_req_valid <= 1'b0;
                        state <= STATE_DRAM_RESP;
                    end
                end

                STATE_DRAM_RESP: begin
                    if (dram_resp_valid) begin
                        // Update directory for new allocation
                        dir_tag[dir_index]    <= dir_lookup_tag;
                        dir_valid[dir_index]  <= 1'b1;
                        if (current_we) begin
                            dir_owned[dir_index]  <= (1'b1 << active_sm);
                            dir_shared[dir_index] <= {NUM_SMS{1'b0}};
                        end else begin
                            dir_shared[dir_index] <= (1'b1 << active_sm);
                            dir_owned[dir_index]  <= {NUM_SMS{1'b0}};
                        end
                        state <= STATE_RESPOND;
                    end
                end

                STATE_RESPOND: begin
                    // Send response back to requesting SM
                    l1_resp_valid[active_sm] <= 1'b1;
                    l1_resp_rdata[active_sm] <= l2_resp_rdata;
                    l1_req_ready[active_sm]  <= 1'b1;

                    // Queue prefetch
                    if (stride_detected) begin
                        // Shift prefetch queue and add new entry
                        for (pi = PREFETCH_DEPTH-1; pi > 0; pi = pi - 1) begin
                            prefetch_queue[pi] <= prefetch_queue[pi-1];
                            prefetch_valid[pi] <= prefetch_valid[pi-1];
                        end
                        prefetch_queue[0] <= next_prefetch_addr;
                        prefetch_valid[0] <= 1'b1;
                    end

                    state <= STATE_IDLE;
                end

                STATE_PREFETCH: begin
                    // Issue prefetch to L2
                    if (prefetch_valid[0]) begin
                        l2_req_valid <= 1'b1;
                        l2_req_addr  <= prefetch_queue[0];
                        l2_req_we    <= 1'b0;
                        if (l2_req_ready) begin
                            // Shift prefetch queue
                            for (pi = 0; pi < PREFETCH_DEPTH-1; pi = pi + 1) begin
                                prefetch_queue[pi] <= prefetch_queue[pi+1];
                                prefetch_valid[pi] <= prefetch_valid[pi+1];
                            end
                            prefetch_valid[PREFETCH_DEPTH-1] <= 1'b0;
                            state <= STATE_IDLE;
                        end
                    end else begin
                        state <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
