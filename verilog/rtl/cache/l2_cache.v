//------------------------------------------------------------------------------
// Module: l2_cache
// Description: 1MB L2 Cache, 16-way set associative
//              64-byte cache line
//              Write-back, write-allocate policy
//              4 banks for parallel access
//------------------------------------------------------------------------------

module l2_cache #(
    parameter CACHE_SIZE      = 1048576,   // 1MB
    parameter LINE_SIZE       = 64,        // 64 bytes per line
    parameter NUM_WAYS        = 16,        // 16-way set associative
    parameter NUM_BANKS       = 4,
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 256        // 256-bit interface
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Request interface
    input  wire                        req_valid,
    input  wire                        req_we,
    input  wire [ADDR_WIDTH-1:0]       req_addr,
    input  wire [DATA_WIDTH-1:0]       req_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]   req_be,        // Byte enable
    output reg                         req_ready,

    // Response interface
    output reg                         resp_valid,
    output reg  [DATA_WIDTH-1:0]       resp_rdata,
    output reg                         resp_miss,

    // Memory interface (to DRAM controller)
    output reg                         mem_req_valid,
    output reg  [ADDR_WIDTH-1:0]       mem_req_addr,
    output reg                         mem_req_we,
    output reg  [LINE_SIZE*8-1:0]      mem_req_wdata,
    input  wire                        mem_req_ready,

    input  wire                        mem_resp_valid,
    input  wire [LINE_SIZE*8-1:0]      mem_resp_rdata,
    output reg                         mem_resp_ready
);

    //--------------------------------------------------------------------------
    // Derived parameters
    //--------------------------------------------------------------------------
    localparam LINE_SIZE_BITS = LINE_SIZE * 8;          // 512 bits
    localparam NUM_LINES      = CACHE_SIZE / LINE_SIZE; // 16384 lines
    localparam NUM_SETS       = NUM_LINES / NUM_WAYS;   // 1024 sets
    localparam SET_ADDR_W     = 10;  // log2(1024)
    localparam WAY_W          = 4;   // log2(16)
    localparam BANK_W         = 2;   // log2(4)
    localparam BYTE_OFFSET_W  = 6;   // log2(64)
    localparam TAG_WIDTH      = ADDR_WIDTH - SET_ADDR_W - BYTE_OFFSET_W;  // 16 bits

    //--------------------------------------------------------------------------
    // Address decomposition
    // [31:16] Tag, [15:6] Set index, [5:0] Byte offset
    // Bank = set_index[1:0]
    //--------------------------------------------------------------------------
    wire [TAG_WIDTH-1:0]    req_tag    = req_addr[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
    wire [SET_ADDR_W-1:0]   req_set    = req_addr[SET_ADDR_W+BYTE_OFFSET_W-1:BYTE_OFFSET_W];
    wire [BANK_W-1:0]       req_bank   = req_set[BANK_W-1:0];
    wire [BYTE_OFFSET_W-1:0] req_offset = req_addr[BYTE_OFFSET_W-1:0];

    //--------------------------------------------------------------------------
    // Cache storage arrays (per bank)
    // Each bank contains: data, tag, valid, dirty, LRU
    //--------------------------------------------------------------------------
    // Data storage: 4 banks x 256 sets x 16 ways x 64 bytes
    reg [LINE_SIZE_BITS-1:0] cache_data [0:NUM_BANKS-1] [0:(NUM_SETS/NUM_BANKS)-1] [0:NUM_WAYS-1];

    // Tag storage
    reg [TAG_WIDTH-1:0] cache_tag [0:NUM_BANKS-1] [0:(NUM_SETS/NUM_BANKS)-1] [0:NUM_WAYS-1];

    // Valid bits
    reg [NUM_WAYS-1:0] cache_valid [0:NUM_BANKS-1] [0:(NUM_SETS/NUM_BANKS)-1];

    // Dirty bits
    reg [NUM_WAYS-1:0] cache_dirty [0:NUM_BANKS-1] [0:(NUM_SETS/NUM_BANKS)-1];

    // LRU counter (simple: which way was least recently used)
    reg [WAY_W-1:0] cache_lru [0:NUM_BANKS-1] [0:(NUM_SETS/NUM_BANKS)-1];

    // Bank-internal set address
    wire [(SET_ADDR_W-BANK_W)-1:0] bank_set = req_set[SET_ADDR_W-1:BANK_W];

    //--------------------------------------------------------------------------
    // Hit detection logic
    //--------------------------------------------------------------------------
    reg [NUM_WAYS-1:0] hit_way_vec;
    reg [WAY_W-1:0]    hit_way;
    reg                cache_hit;
    reg [WAY_W-1:0]    lru_way;

    integer w;

    always @(*) begin
        hit_way_vec = {NUM_WAYS{1'b0}};
        hit_way = {WAY_W{1'b0}};
        cache_hit = 1'b0;
        lru_way = cache_lru[req_bank][bank_set];

        for (w = 0; w < NUM_WAYS; w = w + 1) begin
            if (cache_valid[req_bank][bank_set][w] &&
                cache_tag[req_bank][bank_set][w] == req_tag) begin
                hit_way_vec[w] = 1'b1;
                hit_way = w[WAY_W-1:0];
                cache_hit = 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // State machine
    //--------------------------------------------------------------------------
    localparam STATE_IDLE        = 4'b0000;
    localparam STATE_LOOKUP      = 4'b0001;
    localparam STATE_HIT_READ    = 4'b0010;
    localparam STATE_HIT_WRITE   = 4'b0011;
    localparam STATE_MISS_ALLOC  = 4'b0100;
    localparam STATE_MISS_FILL   = 4'b0101;
    localparam STATE_WRITEBACK   = 4'b0110;
    localparam STATE_WRITEBACK_WAIT = 4'b0111;
    localparam STATE_UPDATE      = 4'b1000;

    reg [3:0] state;
    reg [LINE_SIZE_BITS-1:0] fill_buffer;
    reg [LINE_SIZE_BITS-1:0] wb_buffer;
    reg [WAY_W-1:0] alloc_way;
    reg [NUM_WAYS-1:0] valid_vec;

    //--------------------------------------------------------------------------
    // Data word extraction from cache line
    //--------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] read_data_word;

    always @(*) begin
        // Extract the appropriate 256-bit word from 512-bit line
        // based on the offset
        if (req_offset[5]) begin
            read_data_word = cache_data[req_bank][bank_set][hit_way][LINE_SIZE_BITS-1:DATA_WIDTH];
        end else begin
            read_data_word = cache_data[req_bank][bank_set][hit_way][DATA_WIDTH-1:0];
        end
    end

    //--------------------------------------------------------------------------
    // Find first invalid way for allocation
    //--------------------------------------------------------------------------
    integer fi;
    reg [WAY_W-1:0] first_invalid_way;
    reg             has_invalid;

    always @(*) begin
        has_invalid = 1'b0;
        first_invalid_way = {WAY_W{1'b0}};
        for (fi = 0; fi < NUM_WAYS; fi = fi + 1) begin
            if (!has_invalid && !cache_valid[req_bank][bank_set][fi]) begin
                has_invalid = 1'b1;
                first_invalid_way = fi[WAY_W-1:0];
            end
        end
    end

    //--------------------------------------------------------------------------
    // Main state machine
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= STATE_IDLE;
            req_ready     <= 1'b1;
            resp_valid    <= 1'b0;
            resp_rdata    <= {DATA_WIDTH{1'b0}};
            resp_miss     <= 1'b0;
            mem_req_valid <= 1'b0;
            mem_req_addr  <= {ADDR_WIDTH{1'b0}};
            mem_req_we    <= 1'b0;
            mem_req_wdata <= {LINE_SIZE_BITS{1'b0}};
            mem_resp_ready<= 1'b1;
            fill_buffer   <= {LINE_SIZE_BITS{1'b0}};
            wb_buffer     <= {LINE_SIZE_BITS{1'b0}};
            alloc_way     <= {WAY_W{1'b0}};

            // Initialize cache state
        end else begin
            case (state)
                STATE_IDLE: begin
                    req_ready  <= 1'b1;
                    resp_valid <= 1'b0;
                    resp_miss  <= 1'b0;
                    if (req_valid && req_ready) begin
                        state <= STATE_LOOKUP;
                        req_ready <= 1'b0;
                    end
                end

                STATE_LOOKUP: begin
                    if (cache_hit) begin
                        // Update LRU
                        cache_lru[req_bank][bank_set] <= hit_way;
                        if (req_we) begin
                            state <= STATE_HIT_WRITE;
                        end else begin
                            state <= STATE_HIT_READ;
                        end
                    end else begin
                        // Miss - need to allocate
                        resp_miss <= 1'b1;
                        state <= STATE_MISS_ALLOC;
                    end
                end

                STATE_HIT_READ: begin
                    resp_valid <= 1'b1;
                    resp_rdata <= read_data_word;
                    state <= STATE_IDLE;
                end

                STATE_HIT_WRITE: begin
                    // Update data in cache line
                    // Merge write data with existing line using byte enables
                    if (req_offset[5]) begin
                        cache_data[req_bank][bank_set][hit_way][LINE_SIZE_BITS-1:LINE_SIZE_BITS/2] <=
                            merge_write(cache_data[req_bank][bank_set][hit_way][LINE_SIZE_BITS-1:LINE_SIZE_BITS/2],
                                        req_wdata, req_be);
                    end else begin
                        cache_data[req_bank][bank_set][hit_way][LINE_SIZE_BITS/2-1:0] <=
                            merge_write(cache_data[req_bank][bank_set][hit_way][LINE_SIZE_BITS/2-1:0],
                                        req_wdata, req_be);
                    end
                    cache_dirty[req_bank][bank_set][hit_way] <= 1'b1;
                    resp_valid <= 1'b1;
                    state <= STATE_IDLE;
                end

                STATE_MISS_ALLOC: begin
                    // Choose allocation way
                    if (has_invalid) begin
                        alloc_way <= first_invalid_way;
                        state <= STATE_MISS_FILL;
                    end else begin
                        alloc_way <= lru_way;
                        // Check if victim is dirty
                        if (cache_dirty[req_bank][bank_set][lru_way]) begin
                            // Need writeback
                            wb_buffer <= cache_data[req_bank][bank_set][lru_way];
                            state <= STATE_WRITEBACK;
                        end else begin
                            state <= STATE_MISS_FILL;
                        end
                    end
                end

                STATE_WRITEBACK: begin
                    mem_req_valid <= 1'b1;
                    mem_req_addr  <= {cache_tag[req_bank][bank_set][alloc_way],
                                       req_set, {BYTE_OFFSET_W{1'b0}}};
                    mem_req_we    <= 1'b1;
                    mem_req_wdata <= wb_buffer;
                    if (mem_req_ready) begin
                        mem_req_valid <= 1'b0;
                        state <= STATE_MISS_FILL;
                    end
                end

                STATE_MISS_FILL: begin
                    mem_req_valid <= 1'b1;
                    mem_req_addr  <= {req_addr[ADDR_WIDTH-1:BYTE_OFFSET_W], {BYTE_OFFSET_W{1'b0}}};
                    mem_req_we    <= 1'b0;
                    if (mem_req_ready) begin
                        mem_req_valid <= 1'b0;
                        state <= STATE_UPDATE;
                    end
                end

                STATE_UPDATE: begin
                    if (mem_resp_valid) begin
                        // Fill the cache line
                        cache_data[req_bank][bank_set][alloc_way] <= mem_resp_rdata;
                        cache_tag[req_bank][bank_set][alloc_way]  <= req_tag;
                        cache_valid[req_bank][bank_set][alloc_way] <= 1'b1;
                        cache_dirty[req_bank][bank_set][alloc_way] <= req_we;
                        cache_lru[req_bank][bank_set] <= alloc_way;

                        // For write, merge data
                        if (req_we) begin
                            if (req_offset[5]) begin
                                cache_data[req_bank][bank_set][alloc_way][LINE_SIZE_BITS-1:LINE_SIZE_BITS/2] <=
                                    merge_write(mem_resp_rdata[LINE_SIZE_BITS-1:LINE_SIZE_BITS/2],
                                                req_wdata, req_be);
                            end else begin
                                cache_data[req_bank][bank_set][alloc_way][LINE_SIZE_BITS/2-1:0] <=
                                    merge_write(mem_resp_rdata[LINE_SIZE_BITS/2-1:0],
                                                req_wdata, req_be);
                            end
                            cache_dirty[req_bank][bank_set][alloc_way] <= 1'b1;
                        end

                        // Extract response data
                        if (req_offset[5]) begin
                            resp_rdata <= mem_resp_rdata[LINE_SIZE_BITS-1:LINE_SIZE_BITS/2];
                        end else begin
                            resp_rdata <= mem_resp_rdata[LINE_SIZE_BITS/2-1:0];
                        end

                        resp_valid <= 1'b1;
                        state <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // Merge write data function
    //--------------------------------------------------------------------------
    function [DATA_WIDTH-1:0] merge_write;
        input [DATA_WIDTH-1:0] original;
        input [DATA_WIDTH-1:0] new_data;
        input [(DATA_WIDTH/8)-1:0] be;
        integer i;
        begin
            merge_write = {DATA_WIDTH{1'b0}};
            for (i = 0; i < (DATA_WIDTH/8); i = i + 1) begin
                if (be[i]) begin
                    merge_write[i*8 +: 8] = new_data[i*8 +: 8];
                end else begin
                    merge_write[i*8 +: 8] = original[i*8 +: 8];
                end
            end
        end
    endfunction

    //--------------------------------------------------------------------------
    // Reset initialization
    //--------------------------------------------------------------------------
    integer bi, si, wi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (bi = 0; bi < NUM_BANKS; bi = bi + 1) begin
                for (si = 0; si < (NUM_SETS/NUM_BANKS); si = si + 1) begin
                    cache_valid[bi][si] <= {NUM_WAYS{1'b0}};
                    cache_dirty[bi][si] <= {NUM_WAYS{1'b0}};
                    cache_lru[bi][si]   <= {WAY_W{1'b0}};
                    for (wi = 0; wi < NUM_WAYS; wi = wi + 1) begin
                        cache_tag[bi][si][wi] <= {TAG_WIDTH{1'b0}};
                    end
                end
            end
        end
    end

endmodule
