//------------------------------------------------------------------------------
// Module: dram_scheduler
// Description: FR-FCFS (First-Ready First-Come-First-Served) scheduling
//              Bank-aware request scheduling with priority support
//              Row buffer management for hit-first scheduling
//------------------------------------------------------------------------------

module dram_scheduler #(
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 256,
    parameter NUM_BANKS       = 64,
    parameter REQ_QUEUE_DEPTH = 32,
    parameter NUM_PRIORITIES  = 4,
    parameter PRIORITY_W      = 2
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Request input interface
    input  wire                        req_valid,
    input  wire [ADDR_WIDTH-1:0]       req_addr,
    input  wire                        req_we,
    input  wire [DATA_WIDTH-1:0]       req_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]   req_be,
    input  wire [PRIORITY_W-1:0]       req_priority,
    output reg                         req_ready,

    // Scheduled output interface (to GDDR6 controller)
    output reg                         sched_valid,
    output reg  [ADDR_WIDTH-1:0]       sched_addr,
    output reg                         sched_we,
    output reg  [DATA_WIDTH-1:0]       sched_wdata,
    output reg  [(DATA_WIDTH/8)-1:0]   sched_be,
    input  wire                        sched_ready,

    // Bank status from controller
    input  wire [NUM_BANKS-1:0]        bank_active,
    input  wire [13:0]                 bank_row    [0:NUM_BANKS-1],

    // Scheduling statistics
    output reg  [31:0]                 row_hits,
    output reg  [31:0]                 row_misses,
    output reg  [31:0]                 requests_queued
);

    //--------------------------------------------------------------------------
    // Address decomposition for bank/row extraction
    // Matches GDDR6 controller addressing
    //--------------------------------------------------------------------------
    localparam ROW_ADDR_W   = 14;
    localparam BANK_ADDR_W  = 6;   // log2(64)
    localparam COL_ADDR_W   = 6;

    wire [ROW_ADDR_W-1:0]   req_row  = req_addr[ADDR_WIDTH-1:ADDR_WIDTH-ROW_ADDR_W];
    wire [BANK_ADDR_W-1:0]  req_bank = req_addr[ADDR_WIDTH-ROW_ADDR_W-1:ADDR_WIDTH-ROW_ADDR_W-BANK_ADDR_W];

    //--------------------------------------------------------------------------
    // Request queue entry structure
    //--------------------------------------------------------------------------
    reg [ADDR_WIDTH-1:0]   queue_addr     [0:REQ_QUEUE_DEPTH-1];
    reg                    queue_we       [0:REQ_QUEUE_DEPTH-1];
    reg [DATA_WIDTH-1:0]   queue_wdata    [0:REQ_QUEUE_DEPTH-1];
    reg [(DATA_WIDTH/8)-1:0] queue_be     [0:REQ_QUEUE_DEPTH-1];
    reg [PRIORITY_W-1:0]   queue_priority [0:REQ_QUEUE_DEPTH-1];
    reg [BANK_ADDR_W-1:0]  queue_bank     [0:REQ_QUEUE_DEPTH-1];
    reg [ROW_ADDR_W-1:0]   queue_row      [0:REQ_QUEUE_DEPTH-1];
    reg                    queue_valid    [0:REQ_QUEUE_DEPTH-1];
    reg [31:0]             queue_age      [0:REQ_QUEUE_DEPTH-1];  // Age counter for fairness

    // Queue pointers
    reg [$clog2(REQ_QUEUE_DEPTH)-1:0] queue_head;
    reg [$clog2(REQ_QUEUE_DEPTH)-1:0] queue_tail;
    reg [$clog2(REQ_QUEUE_DEPTH):0]   queue_count;

    //--------------------------------------------------------------------------
    // Per-bank request tracking
    //--------------------------------------------------------------------------
    reg [REQ_QUEUE_DEPTH-1:0] bank_requests [0:NUM_BANKS-1];
    reg [$clog2(REQ_QUEUE_DEPTH)-1:0] bank_head [0:NUM_BANKS-1];
    reg [$clog2(REQ_QUEUE_DEPTH)-1:0] bank_tail [0:NUM_BANKS-1];
    reg [$clog2(REQ_QUEUE_DEPTH):0]   bank_count [0:NUM_BANKS-1];

    // Per-bank row buffer state (shadow copy)
    reg [ROW_ADDR_W-1:0] sched_bank_row [0:NUM_BANKS-1];
    reg                  sched_bank_open [0:NUM_BANKS-1];

    //--------------------------------------------------------------------------
    // FR-FCFS scheduling logic
    //--------------------------------------------------------------------------
    // First, find requests that are "ready" (bank not busy)
    // Then prioritize row hits over row misses
    // Within each group, use priority then age

    reg [REQ_QUEUE_DEPTH-1:0] ready_mask;
    reg [REQ_QUEUE_DEPTH-1:0] row_hit_mask;
    reg [REQ_QUEUE_DEPTH-1:0] priority_mask;

    reg [$clog2(REQ_QUEUE_DEPTH)-1:0] selected_idx;
    reg                               selected_valid;
    reg                               selected_is_hit;

    integer qi, bi;

    always @(*) begin
        // Initialize masks
        ready_mask    = {REQ_QUEUE_DEPTH{1'b0}};
        row_hit_mask  = {REQ_QUEUE_DEPTH{1'b0}};
        priority_mask = {REQ_QUEUE_DEPTH{1'b0}};
        selected_idx  = {$clog2(REQ_QUEUE_DEPTH){1'b0}};
        selected_valid = 1'b0;
        selected_is_hit = 1'b0;

        // Build ready mask: requests to banks that are not currently active
        for (qi = 0; qi < REQ_QUEUE_DEPTH; qi = qi + 1) begin
            if (queue_valid[qi] && !bank_active[queue_bank[qi]]) begin
                ready_mask[qi] = 1'b1;
            end
        end

        // Build row hit mask: ready requests that hit open rows
        for (qi = 0; qi < REQ_QUEUE_DEPTH; qi = qi + 1) begin
            if (ready_mask[qi] && sched_bank_open[queue_bank[qi]] &&
                sched_bank_row[queue_bank[qi]] == queue_row[qi]) begin
                row_hit_mask[qi] = 1'b1;
            end
        end

        // Select highest priority row-hit request
        for (qi = 0; qi < REQ_QUEUE_DEPTH; qi = qi + 1) begin
            if (row_hit_mask[qi]) begin
                if (!selected_valid || queue_priority[qi] > queue_priority[selected_idx] ||
                    (queue_priority[qi] == queue_priority[selected_idx] &&
                     queue_age[qi] > queue_age[selected_idx])) begin
                    selected_idx = qi[$clog2(REQ_QUEUE_DEPTH)-1:0];
                    selected_valid = 1'b1;
                    selected_is_hit = 1'b1;
                end
            end
        end

        // If no row hits, select highest priority ready request
        if (!selected_valid) begin
            for (qi = 0; qi < REQ_QUEUE_DEPTH; qi = qi + 1) begin
                if (ready_mask[qi]) begin
                    if (!selected_valid || queue_priority[qi] > queue_priority[selected_idx] ||
                        (queue_priority[qi] == queue_priority[selected_idx] &&
                         queue_age[qi] > queue_age[selected_idx])) begin
                        selected_idx = qi[$clog2(REQ_QUEUE_DEPTH)-1:0];
                        selected_valid = 1'b1;
                        selected_is_hit = 1'b0;
                    end
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Request enqueue
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            queue_head <= {$clog2(REQ_QUEUE_DEPTH){1'b0}};
            queue_tail <= {$clog2(REQ_QUEUE_DEPTH){1'b0}};
            queue_count <= {($clog2(REQ_QUEUE_DEPTH)+1){1'b0}};
            req_ready <= 1'b1;

            for (qi = 0; qi < REQ_QUEUE_DEPTH; qi = qi + 1) begin
                queue_valid[qi] <= 1'b0;
                queue_age[qi]   <= 32'd0;
            end

            for (bi = 0; bi < NUM_BANKS; bi = bi + 1) begin
                bank_count[bi] <= {($clog2(REQ_QUEUE_DEPTH)+1){1'b0}};
                sched_bank_open[bi] <= 1'b0;
                sched_bank_row[bi]  <= {ROW_ADDR_W{1'b0}};
            end

            row_hits   <= 32'd0;
            row_misses <= 32'd0;
            requests_queued <= 32'd0;
        end else begin
            // Update ages for all valid requests
            for (qi = 0; qi < REQ_QUEUE_DEPTH; qi = qi + 1) begin
                if (queue_valid[qi]) begin
                    queue_age[qi] <= queue_age[qi] + 1'b1;
                end
            end

            // Enqueue new request
            if (req_valid && req_ready) begin
                queue_addr[queue_tail]     <= req_addr;
                queue_we[queue_tail]       <= req_we;
                queue_wdata[queue_tail]    <= req_wdata;
                queue_be[queue_tail]       <= req_be;
                queue_priority[queue_tail] <= req_priority;
                queue_bank[queue_tail]     <= req_bank;
                queue_row[queue_tail]      <= req_row;
                queue_valid[queue_tail]    <= 1'b1;
                queue_age[queue_tail]      <= 32'd0;
                queue_tail <= queue_tail + 1'b1;
                queue_count <= queue_count + 1'b1;

                bank_count[req_bank] <= bank_count[req_bank] + 1'b1;
            end

            // Dequeue scheduled request
            if (sched_valid && sched_ready) begin
                queue_valid[selected_idx] <= 1'b0;
                queue_count <= queue_count - 1'b1;

                bank_count[queue_bank[selected_idx]] <=
                    bank_count[queue_bank[selected_idx]] - 1'b1;

                // Update row buffer state
                sched_bank_row[queue_bank[selected_idx]] <= queue_row[selected_idx];
                sched_bank_open[queue_bank[selected_idx]] <= 1'b1;

                // Update statistics
                if (selected_is_hit) begin
                    row_hits <= row_hits + 1'b1;
                end else begin
                    row_misses <= row_misses + 1'b1;
                end
            end

            // Update ready signal
            req_ready <= (queue_count < REQ_QUEUE_DEPTH - 2);

            // Update statistics
            requests_queued <= queue_count;
        end
    end

    //--------------------------------------------------------------------------
    // Scheduled output
    //--------------------------------------------------------------------------
    always @(*) begin
        sched_valid = selected_valid;
        if (selected_valid) begin
            sched_addr  = queue_addr[selected_idx];
            sched_we    = queue_we[selected_idx];
            sched_wdata = queue_wdata[selected_idx];
            sched_be    = queue_be[selected_idx];
        end else begin
            sched_addr  = {ADDR_WIDTH{1'b0}};
            sched_we    = 1'b0;
            sched_wdata = {DATA_WIDTH{1'b0}};
            sched_be    = {(DATA_WIDTH/8){1'b0}};
        end
    end

endmodule
