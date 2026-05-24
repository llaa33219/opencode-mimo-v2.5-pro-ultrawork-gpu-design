//------------------------------------------------------------------------------
// Module: axi4_lite_slave
// Description: AXI4-Lite slave interface for GPU control registers
//              32-bit address, 32-bit data
//              Supports configuration and status register access
//              Interrupt generation capability
//------------------------------------------------------------------------------

module axi4_lite_slave #(
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 32,
    parameter NUM_REGS        = 64        // 64 x 32-bit registers = 256 bytes
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // AXI4-Lite write address channel
    input  wire [ADDR_WIDTH-1:0]       awaddr,
    input  wire                        awvalid,
    output reg                         awready,

    // AXI4-Lite write data channel
    input  wire [DATA_WIDTH-1:0]       wdata,
    input  wire [(DATA_WIDTH/8)-1:0]   wstrb,
    input  wire                        wvalid,
    output reg                         wready,

    // AXI4-Lite write response channel
    output reg  [1:0]                  bresp,
    output reg                         bvalid,
    input  wire                        bready,

    // AXI4-Lite read address channel
    input  wire [ADDR_WIDTH-1:0]       araddr,
    input  wire                        arvalid,
    output reg                         arready,

    // AXI4-Lite read data channel
    output reg  [DATA_WIDTH-1:0]       rdata,
    output reg  [1:0]                  rresp,
    output reg                         rvalid,
    input  wire                        rready,

    // Register interface to GPU internals
    output reg  [DATA_WIDTH-1:0]       reg_wr_data,
    output reg  [5:0]                  reg_wr_addr,     // log2(64) = 6
    output reg                         reg_wr_en,
    output reg  [DATA_WIDTH-1:0]       reg_wr_mask,

    input  wire [DATA_WIDTH-1:0]       reg_rd_data [0:NUM_REGS-1],
    output reg  [5:0]                  reg_rd_addr,
    output reg                         reg_rd_en,

    // Interrupt output
    output reg                         irq_out,
    input  wire [31:0]                 irq_status,      // Per-source status
    input  wire [31:0]                 irq_enable       // Per-source enable
);

    //--------------------------------------------------------------------------
    // AXI4-Lite response codes
    //--------------------------------------------------------------------------
    localparam RESP_OKAY   = 2'b00;
    localparam RESP_EXOKAY = 2'b01;
    localparam RESP_SLVERR = 2'b10;
    localparam RESP_DECERR = 2'b11;

    //--------------------------------------------------------------------------
    // Register map offsets (in 32-bit words)
    //--------------------------------------------------------------------------
    localparam REG_DEVICE_ID      = 6'd0;   // 0x00: Device ID
    localparam REG_STATUS         = 6'd1;   // 0x04: GPU status
    localparam REG_CONTROL        = 6'd2;   // 0x08: GPU control
    localparam REG_SM_ENABLE      = 6'd3;   // 0x0C: SM enable mask
    localparam REG_CLOCK_GATE     = 6'd4;   // 0x10: Clock gating control
    localparam REG_DVFS_CTRL      = 6'd5;   // 0x14: DVFS control
    localparam REG_THERMAL_STATUS = 6'd6;   // 0x18: Thermal status
    localparam REG_IRQ_STATUS     = 6'd7;   // 0x1C: Interrupt status
    localparam REG_IRQ_ENABLE     = 6'd8;   // 0x20: Interrupt enable
    localparam REG_IRQ_CLEAR      = 6'd9;   // 0x24: Interrupt clear
    localparam REG_DMA_CTRL       = 6'd10;  // 0x28: DMA control
    localparam REG_DMA_STATUS     = 6'd11;  // 0x2C: DMA status
    localparam REG_FB_BASE        = 6'd12;  // 0x30: Framebuffer base
    localparam REG_FB_SIZE        = 6'd13;  // 0x34: Framebuffer size
    localparam REG_PERF_COUNT0    = 6'd14;  // 0x38: Performance counter 0
    localparam REG_PERF_COUNT1    = 6'd15;  // 0x3C: Performance counter 1

    // Device ID constant
    localparam DEVICE_ID_VALUE = 32'h4750_0001;  // "GP" + version 1

    //--------------------------------------------------------------------------
    // Internal register storage
    //--------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] ctrl_regs [0:NUM_REGS-1];

    // Write address and data latches
    reg [ADDR_WIDTH-1:0] awaddr_latch;
    reg [ADDR_WIDTH-1:0] araddr_latch;
    reg [DATA_WIDTH-1:0] wdata_latch;
    reg [(DATA_WIDTH/8)-1:0] wstrb_latch;

    // Interrupt state
    reg [31:0] irq_pending;
    reg [31:0] irq_enabled;

    //--------------------------------------------------------------------------
    // Write address channel
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awready <= 1'b0;
            awaddr_latch <= {ADDR_WIDTH{1'b0}};
        end else begin
            if (!awready && awvalid) begin
                awready <= 1'b1;
                awaddr_latch <= awaddr;
            end else begin
                awready <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Write data channel
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wready <= 1'b0;
            wdata_latch <= {DATA_WIDTH{1'b0}};
            wstrb_latch <= {(DATA_WIDTH/8){1'b0}};
        end else begin
            if (!wready && wvalid) begin
                wready <= 1'b1;
                wdata_latch <= wdata;
                wstrb_latch <= wstrb;
            end else begin
                wready <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Write response channel
    //--------------------------------------------------------------------------
    reg write_addr_done;
    reg write_data_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bvalid <= 1'b0;
            bresp  <= RESP_OKAY;
            write_addr_done <= 1'b0;
            write_data_done <= 1'b0;
            reg_wr_en <= 1'b0;
        end else begin
            reg_wr_en <= 1'b0;

            if (awvalid && awready) begin
                write_addr_done <= 1'b1;
            end
            if (wvalid && wready) begin
                write_data_done <= 1'b1;
            end

            if (write_addr_done && write_data_done && !bvalid) begin
                // Perform write
                bvalid <= 1'b1;
                bresp  <= RESP_OKAY;

                // Address decode: register address is awaddr[7:2]
                reg_wr_addr <= awaddr_latch[7:2];
                reg_wr_data <= wdata_latch;
                reg_wr_mask <= {{8{wstrb_latch[3]}}, {8{wstrb_latch[2]}},
                                {8{wstrb_latch[1]}}, {8{wstrb_latch[0]}}};
                reg_wr_en   <= 1'b1;

                // Write to control register
                if (awaddr_latch[7:2] < NUM_REGS) begin
                    // Apply write mask
                    if (wstrb_latch[0]) ctrl_regs[awaddr_latch[7:2]][7:0]   <= wdata_latch[7:0];
                    if (wstrb_latch[1]) ctrl_regs[awaddr_latch[7:2]][15:8]  <= wdata_latch[15:8];
                    if (wstrb_latch[2]) ctrl_regs[awaddr_latch[7:2]][23:16] <= wdata_latch[23:16];
                    if (wstrb_latch[3]) ctrl_regs[awaddr_latch[7:2]][31:24] <= wdata_latch[31:24];
                end else begin
                    bresp <= RESP_SLVERR;  // Address out of range
                end

                write_addr_done <= 1'b0;
                write_data_done <= 1'b0;
            end

            if (bvalid && bready) begin
                bvalid <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Read address channel
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arready <= 1'b0;
            araddr_latch <= {ADDR_WIDTH{1'b0}};
        end else begin
            if (!arready && arvalid) begin
                arready <= 1'b1;
                araddr_latch <= araddr;
            end else begin
                arready <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Read data channel
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rvalid <= 1'b0;
            rresp  <= RESP_OKAY;
            rdata  <= {DATA_WIDTH{1'b0}};
            reg_rd_en <= 1'b0;
        end else begin
            reg_rd_en <= 1'b0;

            if (arvalid && arready && !rvalid) begin
                rvalid <= 1'b1;
                rresp  <= RESP_OKAY;
                reg_rd_addr <= araddr_latch[7:2];
                reg_rd_en   <= 1'b1;

                // Register read with special handling
                case (araddr_latch[7:2])
                    REG_DEVICE_ID:      rdata <= DEVICE_ID_VALUE;
                    REG_STATUS:         rdata <= reg_rd_data[REG_STATUS];
                    REG_CONTROL:        rdata <= ctrl_regs[REG_CONTROL];
                    REG_SM_ENABLE:      rdata <= ctrl_regs[REG_SM_ENABLE];
                    REG_CLOCK_GATE:     rdata <= ctrl_regs[REG_CLOCK_GATE];
                    REG_DVFS_CTRL:      rdata <= ctrl_regs[REG_DVFS_CTRL];
                    REG_THERMAL_STATUS: rdata <= reg_rd_data[REG_THERMAL_STATUS];
                    REG_IRQ_STATUS:     rdata <= irq_pending;
                    REG_IRQ_ENABLE:     rdata <= irq_enabled;
                    REG_DMA_CTRL:       rdata <= ctrl_regs[REG_DMA_CTRL];
                    REG_DMA_STATUS:     rdata <= reg_rd_data[REG_DMA_STATUS];
                    REG_FB_BASE:        rdata <= ctrl_regs[REG_FB_BASE];
                    REG_FB_SIZE:        rdata <= ctrl_regs[REG_FB_SIZE];
                    REG_PERF_COUNT0:    rdata <= reg_rd_data[REG_PERF_COUNT0];
                    REG_PERF_COUNT1:    rdata <= reg_rd_data[REG_PERF_COUNT1];
                    default: begin
                        if (araddr_latch[7:2] < NUM_REGS) begin
                            rdata <= ctrl_regs[araddr_latch[7:2]];
                        end else begin
                            rdata <= 32'hDEAD_BEEF;
                            rresp <= RESP_SLVERR;
                        end
                    end
                endcase
            end

            if (rvalid && rready) begin
                rvalid <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Interrupt logic
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_pending <= 32'd0;
            irq_enabled <= 32'd0;
            irq_out     <= 1'b0;
        end else begin
            // Update pending interrupts
            irq_pending <= irq_pending | irq_status;

            // Handle interrupt enable register writes
            if (reg_wr_en && reg_wr_addr == REG_IRQ_ENABLE) begin
                irq_enabled <= reg_wr_data;
            end

            // Handle interrupt clear register writes
            if (reg_wr_en && reg_wr_addr == REG_IRQ_CLEAR) begin
                irq_pending <= irq_pending & ~reg_wr_data;
            end

            // Generate interrupt output
            irq_out <= |(irq_pending & irq_enabled);
        end
    end

    //--------------------------------------------------------------------------
    // Reset initialization
    //--------------------------------------------------------------------------
    integer ri;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ri = 0; ri < NUM_REGS; ri = ri + 1) begin
                ctrl_regs[ri] <= {DATA_WIDTH{1'b0}};
            end
        end
    end

endmodule
