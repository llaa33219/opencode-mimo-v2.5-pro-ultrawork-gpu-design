//------------------------------------------------------------------------------
// Module: register_file
// Description: 64KB per-SM register file (16384 x 32-bit entries)
//              8 read ports, 4 write ports
//              16-bank structure for simultaneous read/write
//              Supports conflict-free access when addresses map to different banks
//------------------------------------------------------------------------------

module register_file #(
    parameter DATA_WIDTH    = 32,
    parameter NUM_ENTRIES   = 16384,     // 64KB / 4 bytes = 16384 entries
    parameter ADDR_WIDTH    = 14,        // log2(16384)
    parameter NUM_BANKS     = 16,
    parameter BANK_ADDR_W   = 4,         // log2(16)
    parameter NUM_READ_PORTS = 8,
    parameter NUM_WRITE_PORTS = 4
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // Read port interfaces
    input  wire [NUM_READ_PORTS-1:0]  read_en,
    input  wire [ADDR_WIDTH-1:0]      read_addr [0:NUM_READ_PORTS-1],
    output reg  [DATA_WIDTH-1:0]      read_data [0:NUM_READ_PORTS-1],

    // Write port interfaces
    input  wire [NUM_WRITE_PORTS-1:0] write_en,
    input  wire [ADDR_WIDTH-1:0]      write_addr [0:NUM_WRITE_PORTS-1],
    input  wire [DATA_WIDTH-1:0]      write_data [0:NUM_WRITE_PORTS-1],

    // Bank conflict detection
    output reg                        bank_conflict,
    output reg  [NUM_READ_PORTS-1:0]  read_stall
);

    //--------------------------------------------------------------------------
    // Local parameters
    //--------------------------------------------------------------------------
    localparam ENTRIES_PER_BANK = NUM_ENTRIES / NUM_BANKS;  // 1024
    localparam BANK_ENTRY_W     = 10;                        // log2(1024)

    //--------------------------------------------------------------------------
    // Banked memory arrays
    // Each bank is an independent dual-port RAM (1 read + 1 write)
    //--------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] bank_mem [0:NUM_BANKS-1] [0:ENTRIES_PER_BANK-1];

    // Bank addresses extracted from full address
    wire [BANK_ADDR_W-1:0]   r_bank [0:NUM_READ_PORTS-1];
    wire [BANK_ENTRY_W-1:0]  r_entry [0:NUM_READ_PORTS-1];
    wire [BANK_ADDR_W-1:0]   w_bank [0:NUM_WRITE_PORTS-1];
    wire [BANK_ENTRY_W-1:0]  w_entry [0:NUM_WRITE_PORTS-1];

    // Bank read enable and data output from each bank
    reg  [NUM_BANKS-1:0]     bank_rden;
    reg  [BANK_ENTRY_W-1:0]  bank_raddr [0:NUM_BANKS-1];
    reg  [NUM_BANKS-1:0]     bank_wren;
    reg  [BANK_ENTRY_W-1:0]  bank_waddr [0:NUM_BANKS-1];
    reg  [DATA_WIDTH-1:0]    bank_wdata [0:NUM_BANKS-1];
    wire [DATA_WIDTH-1:0]    bank_rdata [0:NUM_BANKS-1];

    // Bank conflict detection
    reg [NUM_BANKS-1:0] bank_read_req [0:NUM_READ_PORTS-1];
    reg [NUM_BANKS-1:0] bank_write_req [0:NUM_WRITE_PORTS-1];
    reg [NUM_BANKS-1:0] bank_access_cnt;

    //--------------------------------------------------------------------------
    // Address decomposition
    //--------------------------------------------------------------------------
    genvar i, j;
    generate
        for (i = 0; i < NUM_READ_PORTS; i = i + 1) begin : read_addr_decode
            assign r_bank[i]  = read_addr[i][ADDR_WIDTH-1:ADDR_WIDTH-BANK_ADDR_W];
            assign r_entry[i] = read_addr[i][BANK_ENTRY_W-1:0];
        end
        for (i = 0; i < NUM_WRITE_PORTS; i = i + 1) begin : write_addr_decode
            assign w_bank[i]  = write_addr[i][ADDR_WIDTH-1:ADDR_WIDTH-BANK_ADDR_W];
            assign w_entry[i] = write_addr[i][BANK_ENTRY_W-1:0];
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Bank access arbitration
    // Each cycle, up to 1 read and 1 write can access each bank
    // Priority: write port 0 > write port 1 > write port 2 > write port 3
    // Read conflicts are detected and stalled
    //--------------------------------------------------------------------------
    integer b, p;

    always @(*) begin
        // Initialize
        for (b = 0; b < NUM_BANKS; b = b + 1) begin
            bank_rden[b]  = 1'b0;
            bank_raddr[b] = {BANK_ENTRY_W{1'b0}};
            bank_wren[b]  = 1'b0;
            bank_waddr[b] = {BANK_ENTRY_W{1'b0}};
            bank_wdata[b] = {DATA_WIDTH{1'b0}};
            bank_access_cnt[b] = 4'd0;
        end

        // Count accesses per bank for reads
        for (p = 0; p < NUM_READ_PORTS; p = p + 1) begin
            if (read_en[p]) begin
                bank_access_cnt[r_bank[p]] = bank_access_cnt[r_bank[p]] + 1'b1;
            end
        end

        // Count accesses per bank for writes
        for (p = 0; p < NUM_WRITE_PORTS; p = p + 1) begin
            if (write_en[p]) begin
                bank_access_cnt[w_bank[p]] = bank_access_cnt[w_bank[p]] + 1'b1;
            end
        end

        // Detect conflicts (more than 1 read to same bank, or read+write conflict)
        bank_conflict = 1'b0;
        for (b = 0; b < NUM_BANKS; b = b + 1) begin
            if (bank_access_cnt[b] > 4'd1) begin
                bank_conflict = 1'b1;
            end
        end

        // Determine which reads get stalled
        for (p = 0; p < NUM_READ_PORTS; p = p + 1) begin
            read_stall[p] = 1'b0;
        end

        // Grant write access - write port 0 has highest priority
        for (p = 0; p < NUM_WRITE_PORTS; p = p + 1) begin
            if (write_en[p]) begin
                if (!bank_wren[w_bank[p]]) begin
                    bank_wren[w_bank[p]]  = 1'b1;
                    bank_waddr[w_bank[p]] = w_entry[p];
                    bank_wdata[w_bank[p]] = write_data[p];
                end
                // else: write conflict - lower priority write is dropped
            end
        end

        // Grant read access - first reader to each bank wins
        for (p = 0; p < NUM_READ_PORTS; p = p + 1) begin
            if (read_en[p]) begin
                if (!bank_rden[r_bank[p]] && !bank_wren[r_bank[p]]) begin
                    bank_rden[r_bank[p]]  = 1'b1;
                    bank_raddr[r_bank[p]] = r_entry[p];
                end else begin
                    read_stall[p] = 1'b1;  // Bank busy
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Bank memory instances
    //--------------------------------------------------------------------------
    generate
        for (b = 0; b < NUM_BANKS; b = b + 1) begin : bank_inst
            // Bank read data output
            assign bank_rdata[b] = bank_mem[b][bank_raddr[b]];

            // Sequential write
            always @(posedge clk) begin
                if (bank_wren[b]) begin
                    bank_mem[b][bank_waddr[b]] <= bank_wdata[b];
                end
            end
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Read data output mux
    // Route bank read data back to requesting port
    //--------------------------------------------------------------------------
    integer rp;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rp = 0; rp < NUM_READ_PORTS; rp = rp + 1) begin
                read_data[rp] <= {DATA_WIDTH{1'b0}};
            end
        end else begin
            for (rp = 0; rp < NUM_READ_PORTS; rp = rp + 1) begin
                if (read_en[rp] && !read_stall[rp]) begin
                    read_data[rp] <= bank_rdata[r_bank[rp]];
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Reset initialization
    //--------------------------------------------------------------------------
    integer bi, ei;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (bi = 0; bi < NUM_BANKS; bi = bi + 1) begin
                for (ei = 0; ei < ENTRIES_PER_BANK; ei = ei + 1) begin
                    bank_mem[bi][ei] <= {DATA_WIDTH{1'b0}};
                end
            end
        end
    end

endmodule
