//------------------------------------------------------------------------------
// Module: shared_memory
// Description: 32KB per-SM shared memory
//              32 banks for conflict-free access
//              128-bit wide access (4 x 32-bit words)
//              Single-cycle read latency
//------------------------------------------------------------------------------

module shared_memory #(
    parameter DATA_WIDTH     = 32,
    parameter TOTAL_SIZE     = 32768,    // 32KB
    parameter NUM_BANKS      = 32,
    parameter ACCESS_WIDTH   = 128,      // 4 x 32-bit words
    parameter WORDS_PER_ACCESS = 4
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Port 0: 128-bit read/write port
    input  wire                        port0_en,
    input  wire                        port0_we,
    input  wire [$clog2(TOTAL_SIZE/ACCESS_WIDTH)-1:0] port0_addr,
    input  wire [ACCESS_WIDTH-1:0]     port0_wdata,
    output reg  [ACCESS_WIDTH-1:0]     port0_rdata,

    // Port 1: 128-bit read/write port
    input  wire                        port1_en,
    input  wire                        port1_we,
    input  wire [$clog2(TOTAL_SIZE/ACCESS_WIDTH)-1:0] port1_addr,
    input  wire [ACCESS_WIDTH-1:0]     port1_wdata,
    output reg  [ACCESS_WIDTH-1:0]     port1_rdata,

    // Port 2: 128-bit read-only port (for broadcasts)
    input  wire                        port2_en,
    input  wire [$clog2(TOTAL_SIZE/ACCESS_WIDTH)-1:0] port2_addr,
    output reg  [ACCESS_WIDTH-1:0]     port2_rdata,

    // Bank conflict detection
    output reg                         bank_conflict,
    output reg  [2:0]                  conflict_ports
);

    //--------------------------------------------------------------------------
    // Local parameters
    //--------------------------------------------------------------------------
    localparam BANK_SIZE = TOTAL_SIZE / NUM_BANKS;  // 1024 bytes per bank
    localparam BANK_ADDR_W = $clog2(BANK_SIZE / (ACCESS_WIDTH / 8));  // log2(256) = 8
    localparam LINE_ADDR_W = $clog2(TOTAL_SIZE / ACCESS_WIDTH);  // log2(8192) = 13
    localparam BANK_SEL_W  = $clog2(NUM_BANKS);  // 5

    //--------------------------------------------------------------------------
    // Banked memory array
    // 32 independent banks, each holding 256 x 128-bit words
    //--------------------------------------------------------------------------
    reg [ACCESS_WIDTH-1:0] bank_mem [0:NUM_BANKS-1] [0:(BANK_SIZE/(ACCESS_WIDTH/8))-1];

    //--------------------------------------------------------------------------
    // Address decomposition for interleaved banking
    // Address [4:0]   = bank select (32 banks)
    // Address [12:5]  = bank internal address (256 entries)
    //--------------------------------------------------------------------------
    wire [BANK_SEL_W-1:0]  p0_bank = port0_addr[BANK_SEL_W-1:0];
    wire [BANK_ADDR_W-1:0] p0_baddr = port0_addr[LINE_ADDR_W-1:BANK_SEL_W];
    wire [BANK_SEL_W-1:0]  p1_bank = port1_addr[BANK_SEL_W-1:0];
    wire [BANK_ADDR_W-1:0] p1_baddr = port1_addr[LINE_ADDR_W-1:BANK_SEL_W];
    wire [BANK_SEL_W-1:0]  p2_bank = port2_addr[BANK_SEL_W-1:0];
    wire [BANK_ADDR_W-1:0] p2_baddr = port2_addr[LINE_ADDR_W-1:BANK_SEL_W];

    //--------------------------------------------------------------------------
    // Bank access control
    //--------------------------------------------------------------------------
    reg [NUM_BANKS-1:0] bank_rden;
    reg [NUM_BANKS-1:0] bank_wren;
    reg [BANK_ADDR_W-1:0] bank_raddr [0:NUM_BANKS-1];
    reg [BANK_ADDR_W-1:0] bank_waddr [0:NUM_BANKS-1];
    reg [ACCESS_WIDTH-1:0] bank_wdata [0:NUM_BANKS-1];
    wire [ACCESS_WIDTH-1:0] bank_rdata [0:NUM_BANKS-1];

    // Per-word write enable for byte-addressable writes
    reg [WORDS_PER_ACCESS-1:0] p0_wmask;
    reg [WORDS_PER_ACCESS-1:0] p1_wmask;

    integer b;

    always @(*) begin
        // Initialize
        for (b = 0; b < NUM_BANKS; b = b + 1) begin
            bank_rden[b] = 1'b0;
            bank_wren[b] = 1'b0;
            bank_raddr[b] = {BANK_ADDR_W{1'b0}};
            bank_waddr[b] = {BANK_ADDR_W{1'b0}};
            bank_wdata[b] = {ACCESS_WIDTH{1'b0}};
        end
        bank_conflict = 1'b0;
        conflict_ports = 3'd0;
        p0_wmask = {WORDS_PER_ACCESS{1'b1}};
        p1_wmask = {WORDS_PER_ACCESS{1'b1}};

        // Port 0 access (highest priority for writes)
        if (port0_en) begin
            if (port0_we) begin
                bank_wren[p0_bank] = 1'b1;
                bank_waddr[p0_bank] = p0_baddr;
                bank_wdata[p0_bank] = port0_wdata;
            end else begin
                bank_rden[p0_bank] = 1'b1;
                bank_raddr[p0_bank] = p0_baddr;
            end
        end

        // Port 1 access - check for conflict with port 0
        if (port1_en) begin
            if (p0_bank == p1_bank && port0_en) begin
                // Bank conflict detected
                bank_conflict = 1'b1;
                conflict_ports = 3'b011;
            end else begin
                if (port1_we) begin
                    bank_wren[p1_bank] = 1'b1;
                    bank_waddr[p1_bank] = p1_baddr;
                    bank_wdata[p1_bank] = port1_wdata;
                end else begin
                    bank_rden[p1_bank] = 1'b1;
                    bank_raddr[p1_bank] = p1_baddr;
                end
            end
        end

        // Port 2 read-only - check for conflict
        if (port2_en) begin
            if ((p0_bank == p2_bank && port0_en && !port0_we) ||
                (p1_bank == p2_bank && port1_en && !port1_we)) begin
                // Read conflict - port 2 loses (lowest priority)
                bank_conflict = 1'b1;
                conflict_ports = 3'b111;
            end else begin
                bank_rden[p2_bank] = 1'b1;
                bank_raddr[p2_bank] = p2_baddr;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Bank memory instances with per-bank read/write
    //--------------------------------------------------------------------------
    genvar bi;
    generate
        for (bi = 0; bi < NUM_BANKS; bi = bi + 1) begin : bank_mem_gen
            // Combinational read for single-cycle latency
            assign bank_rdata[bi] = bank_mem[bi][bank_raddr[bi]];

            // Sequential write
            always @(posedge clk) begin
                if (bank_wren[bi]) begin
                    bank_mem[bi][bank_waddr[bi]] <= bank_wdata[bi];
                end
            end
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Output registers
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            port0_rdata <= {ACCESS_WIDTH{1'b0}};
            port1_rdata <= {ACCESS_WIDTH{1'b0}};
            port2_rdata <= {ACCESS_WIDTH{1'b0}};
        end else begin
            // Port 0 output
            if (port0_en && !port0_we) begin
                port0_rdata <= bank_rdata[p0_bank];
            end

            // Port 1 output (only if no conflict)
            if (port1_en && !port1_we && !bank_conflict) begin
                port1_rdata <= bank_rdata[p1_bank];
            end else if (bank_conflict && conflict_ports[1:0] == 2'b11) begin
                port1_rdata <= {ACCESS_WIDTH{1'b0}};
            end

            // Port 2 output (only if no conflict)
            if (port2_en && !bank_conflict) begin
                port2_rdata <= bank_rdata[p2_bank];
            end else if (bank_conflict && conflict_ports == 3'b111) begin
                port2_rdata <= {ACCESS_WIDTH{1'b0}};
            end
        end
    end

    //--------------------------------------------------------------------------
    // Reset initialization
    //--------------------------------------------------------------------------
    integer bj, ej;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (bj = 0; bj < NUM_BANKS; bj = bj + 1) begin
                for (ej = 0; ej < (BANK_SIZE/(ACCESS_WIDTH/8)); ej = ej + 1) begin
                    bank_mem[bj][ej] <= {ACCESS_WIDTH{1'b0}};
                end
            end
        end
    end

endmodule
