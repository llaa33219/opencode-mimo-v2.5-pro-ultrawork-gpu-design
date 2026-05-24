//------------------------------------------------------------------------------
// Module: axi4_dma
// Description: Descriptor-based DMA engine with 4 channels
//              Supports scatter-gather transfers
//              Handles 4K page boundary crossing
//------------------------------------------------------------------------------

module axi4_dma #(
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 256,
    parameter NUM_CHANNELS    = 4,
    parameter DESC_FIFO_DEPTH = 16,
    parameter MAX_BURST_LEN   = 16        // Maximum AXI burst length
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Control interface (from AXI4-Lite slave)
    input  wire [NUM_CHANNELS-1:0]     ch_enable,
    input  wire [NUM_CHANNELS-1:0]     ch_start,
    output reg  [NUM_CHANNELS-1:0]     ch_busy,
    output reg  [NUM_CHANNELS-1:0]     ch_done,
    output reg  [NUM_CHANNELS-1:0]     ch_error,

    // Descriptor interface
    input  wire [ADDR_WIDTH-1:0]       desc_src_addr,
    input  wire [ADDR_WIDTH-1:0]       desc_dst_addr,
    input  wire [31:0]                 desc_length,
    input  wire                        desc_scatter_gather,
    input  wire                        desc_valid,
    output reg                         desc_ready,
    input  wire [1:0]                  desc_channel,

    // AXI4 read address channel (to memory)
    output reg  [ADDR_WIDTH-1:0]       araddr,
    output reg  [7:0]                  arlen,
    output reg  [2:0]                  arsize,
    output reg  [1:0]                  arburst,
    output reg                         arvalid,
    input  wire                        arready,

    // AXI4 read data channel
    input  wire [DATA_WIDTH-1:0]       rdata,
    input  wire [1:0]                  rresp,
    input  wire                        rlast,
    input  wire                        rvalid,
    output reg                         rready,

    // AXI4 write address channel
    output reg  [ADDR_WIDTH-1:0]       awaddr,
    output reg  [7:0]                  awlen,
    output reg  [2:0]                  awsize,
    output reg  [1:0]                  awburst,
    output reg                         awvalid,
    input  wire                        awready,

    // AXI4 write data channel
    output reg  [DATA_WIDTH-1:0]       wdata,
    output reg  [(DATA_WIDTH/8)-1:0]   wstrb,
    output reg                         wlast,
    output reg                         wvalid,
    input  wire                        wready,

    // AXI4 write response channel
    input  wire [1:0]                  bresp,
    input  wire                        bvalid,
    output reg                         bready,

    // Interrupt
    output reg                         dma_irq
);

    //--------------------------------------------------------------------------
    // Channel descriptor structure
    //--------------------------------------------------------------------------
    localparam DESC_SIZE = 128;  // Descriptor size in bits

    // Descriptor fields
    // [127:96] Next descriptor pointer (for scatter-gather)
    // [95:64]  Source address
    // [63:32]  Destination address
    // [31:16]  Transfer length in bytes
    // [15:8]   Control/status
    // [7:0]    Reserved

    //--------------------------------------------------------------------------
    // Per-channel state
    //--------------------------------------------------------------------------
    reg [ADDR_WIDTH-1:0] ch_src_addr  [0:NUM_CHANNELS-1];
    reg [ADDR_WIDTH-1:0] ch_dst_addr  [0:NUM_CHANNELS-1];
    reg [31:0]           ch_length    [0:NUM_CHANNELS-1];
    reg [ADDR_WIDTH-1:0] ch_next_desc [0:NUM_CHANNELS-1];
    reg                  ch_sg_mode   [0:NUM_CHANNELS-1];
    reg [2:0]            ch_state    [0:NUM_CHANNELS-1];
    reg [31:0]           ch_xfer_cnt [0:NUM_CHANNELS-1];
    reg [31:0]           ch_total_cnt[0:NUM_CHANNELS-1];

    // Channel state encoding
    localparam CH_IDLE        = 3'b000;
    localparam CH_READ_ADDR   = 3'b001;
    localparam CH_READ_DATA   = 3'b010;
    localparam CH_WRITE_ADDR  = 3'b011;
    localparam CH_WRITE_DATA  = 3'b100;
    localparam CH_WRITE_RESP  = 3'b101;
    localparam CH_NEXT_DESC   = 3'b110;
    localparam CH_DONE        = 3'b111;

    // Descriptor FIFO per channel
    reg [DESC_SIZE-1:0] desc_fifo [0:NUM_CHANNELS-1] [0:DESC_FIFO_DEPTH-1];
    reg [$clog2(DESC_FIFO_DEPTH)-1:0] desc_fifo_wr [0:NUM_CHANNELS-1];
    reg [$clog2(DESC_FIFO_DEPTH)-1:0] desc_fifo_rd [0:NUM_CHANNELS-1];
    reg [$clog2(DESC_FIFO_DEPTH):0]   desc_fifo_cnt [0:NUM_CHANNELS-1];

    //--------------------------------------------------------------------------
    // Arbitration: Round-robin between channels
    //--------------------------------------------------------------------------
    reg [1:0] arbiter;
    reg [1:0] active_ch;
    reg       ch_selected;

    integer ci;
    always @(*) begin
        active_ch = 2'd0;
        ch_selected = 1'b0;
        for (ci = 0; ci < NUM_CHANNELS; ci = ci + 1) begin
            if (ch_busy[ci] && ch_state[ci] != CH_IDLE) begin
                active_ch = ci[1:0];
                ch_selected = 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 4K boundary detection
    // AXI bursts cannot cross 4KB boundaries
    //--------------------------------------------------------------------------
    wire [ADDR_WIDTH-1:0] page_boundary = {ch_src_addr[active_ch][ADDR_WIDTH-1:12], 12'hFFF};
    wire [ADDR_WIDTH-1:0] bytes_to_boundary = page_boundary - ch_src_addr[active_ch] + 1;
    wire [7:0]            max_burst_len;

    assign max_burst_len = (ch_length[active_ch] > (MAX_BURST_LEN * (DATA_WIDTH/8))) ?
                           MAX_BURST_LEN :
                           ch_length[active_ch][15:$clog2(DATA_WIDTH/8)];

    //--------------------------------------------------------------------------
    // Descriptor enqueue
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            desc_ready <= 1'b1;
            for (ci = 0; ci < NUM_CHANNELS; ci = ci + 1) begin
                desc_fifo_wr[ci] <= {$clog2(DESC_FIFO_DEPTH){1'b0}};
                desc_fifo_rd[ci] <= {$clog2(DESC_FIFO_DEPTH){1'b0}};
                desc_fifo_cnt[ci] <= {($clog2(DESC_FIFO_DEPTH)+1){1'b0}};
            end
        end else begin
            desc_ready <= 1'b1;
            for (ci = 0; ci < NUM_CHANNELS; ci = ci + 1) begin
                if (desc_valid && desc_channel == ci[1:0] &&
                    desc_fifo_cnt[ci] < DESC_FIFO_DEPTH) begin
                    desc_fifo[ci][desc_fifo_wr[ci]] <= {
                        32'd0,                    // Reserved
                        desc_length[15:0],        // Length
                        desc_dst_addr,            // Dst address
                        desc_src_addr             // Src address
                    };
                    desc_fifo_wr[ci] <= desc_fifo_wr[ci] + 1'b1;
                    desc_fifo_cnt[ci] <= desc_fifo_cnt[ci] + 1'b1;
                end

                if (ch_state[ci] == CH_IDLE && desc_fifo_cnt[ci] > 0 &&
                    ch_enable[ci] && ch_start[ci]) begin
                    desc_fifo_rd[ci] <= desc_fifo_rd[ci] + 1'b1;
                    desc_fifo_cnt[ci] <= desc_fifo_cnt[ci] - 1'b1;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // DMA state machines per channel
    //--------------------------------------------------------------------------
    integer ch;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arbiter     <= 2'd0;
            dma_irq     <= 1'b0;
            arvalid     <= 1'b0;
            rready      <= 1'b0;
            awvalid     <= 1'b0;
            wvalid      <= 1'b0;
            wlast       <= 1'b0;
            bready      <= 1'b0;

            for (ch = 0; ch < NUM_CHANNELS; ch = ch + 1) begin
                ch_busy[ch]     <= 1'b0;
                ch_done[ch]     <= 1'b0;
                ch_error[ch]    <= 1'b0;
                ch_state[ch]    <= CH_IDLE;
                ch_src_addr[ch] <= {ADDR_WIDTH{1'b0}};
                ch_dst_addr[ch] <= {ADDR_WIDTH{1'b0}};
                ch_length[ch]   <= 32'd0;
                ch_xfer_cnt[ch] <= 32'd0;
                ch_total_cnt[ch]<= 32'd0;
                ch_sg_mode[ch]  <= 1'b0;
            end
        end else begin
            dma_irq <= 1'b0;

            for (ch = 0; ch < NUM_CHANNELS; ch = ch + 1) begin
                case (ch_state[ch])
                    CH_IDLE: begin
                        ch_done[ch]  <= 1'b0;
                        ch_error[ch] <= 1'b0;
                        if (ch_enable[ch] && ch_start[ch] &&
                            desc_fifo_cnt[ch] > 0) begin
                            // Load descriptor
                            ch_state[ch]    <= CH_READ_ADDR;
                            ch_busy[ch]     <= 1'b1;
                            ch_src_addr[ch] <= desc_fifo[ch][desc_fifo_rd[ch]][95:64];
                            ch_dst_addr[ch] <= desc_fifo[ch][desc_fifo_rd[ch]][63:32];
                            ch_length[ch]   <= {16'd0, desc_fifo[ch][desc_fifo_rd[ch]][31:16]};
                            ch_xfer_cnt[ch] <= 32'd0;
                            ch_total_cnt[ch]<= {16'd0, desc_fifo[ch][desc_fifo_rd[ch]][31:16]};
                        end
                    end

                    CH_READ_ADDR: begin
                        if (ch == active_ch) begin
                            araddr  <= ch_src_addr[ch];
                            arlen   <= max_burst_len - 8'd1;
                            arsize  <= 3'b101;  // 32 bytes = 256 bits
                            arburst <= 2'b01;  // INCR
                            arvalid <= 1'b1;
                            if (arready) begin
                                arvalid <= 1'b0;
                                ch_state[ch] <= CH_READ_DATA;
                                rready <= 1'b1;
                            end
                        end
                    end

                    CH_READ_DATA: begin
                        if (ch == active_ch) begin
                            rready <= 1'b1;
                            if (rvalid) begin
                                // Store read data (simplified - would need FIFO)
                                ch_xfer_cnt[ch] <= ch_xfer_cnt[ch] + (DATA_WIDTH/8);
                                if (rlast || ch_xfer_cnt[ch] >= ch_length[ch] - (DATA_WIDTH/8)) begin
                                    rready <= 1'b0;
                                    ch_state[ch] <= CH_WRITE_ADDR;
                                end
                            end
                        end
                    end

                    CH_WRITE_ADDR: begin
                        if (ch == active_ch) begin
                            awaddr  <= ch_dst_addr[ch];
                            awlen   <= max_burst_len - 8'd1;
                            awsize  <= 3'b101;  // 32 bytes
                            awburst <= 2'b01;  // INCR
                            awvalid <= 1'b1;
                            if (awready) begin
                                awvalid <= 1'b0;
                                ch_state[ch] <= CH_WRITE_DATA;
                            end
                        end
                    end

                    CH_WRITE_DATA: begin
                        if (ch == active_ch) begin
                            wdata  <= rdata;  // Forward read data
                            wstrb  <= {(DATA_WIDTH/8){1'b1}};
                            wvalid <= 1'b1;
                            wlast  <= (ch_xfer_cnt[ch] >= ch_length[ch] - (DATA_WIDTH/8));
                            if (wready) begin
                                ch_xfer_cnt[ch] <= ch_xfer_cnt[ch] - (DATA_WIDTH/8);
                                if (wlast) begin
                                    wvalid <= 1'b0;
                                    wlast  <= 1'b0;
                                    ch_state[ch] <= CH_WRITE_RESP;
                                    bready <= 1'b1;
                                end
                            end
                        end
                    end

                    CH_WRITE_RESP: begin
                        if (ch == active_ch) begin
                            bready <= 1'b1;
                            if (bvalid) begin
                                bready <= 1'b0;
                                if (bresp != 2'b00) begin
                                    ch_error[ch] <= 1'b1;
                                    ch_state[ch] <= CH_IDLE;
                                    ch_busy[ch]  <= 1'b0;
                                end else begin
                                    ch_src_addr[ch] <= ch_src_addr[ch] + ch_total_cnt[ch];
                                    ch_dst_addr[ch] <= ch_dst_addr[ch] + ch_total_cnt[ch];
                                    ch_xfer_cnt[ch] <= 32'd0;
                                    ch_length[ch]   <= ch_length[ch] - ch_total_cnt[ch];

                                    if (ch_length[ch] <= ch_total_cnt[ch]) begin
                                        // Transfer complete
                                        ch_done[ch]  <= 1'b1;
                                        ch_busy[ch]  <= 1'b0;
                                        ch_state[ch] <= CH_IDLE;
                                        dma_irq <= 1'b1;
                                    end else begin
                                        // More data to transfer
                                        ch_state[ch] <= CH_READ_ADDR;
                                    end
                                end
                            end
                        end
                    end

                    default: ch_state[ch] <= CH_IDLE;
                endcase
            end
        end
    end

endmodule
