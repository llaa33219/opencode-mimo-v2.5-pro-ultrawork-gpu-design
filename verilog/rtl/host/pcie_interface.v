//------------------------------------------------------------------------------
// Module: pcie_interface
// Description: Simplified PCIe Gen4 x8 interface
//              TLP (Transaction Layer Packet) handling
//              BAR (Base Address Register) decoding
//              MSI-X interrupt support
//------------------------------------------------------------------------------

module pcie_interface #(
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 256,       // PCIe Gen4 x8 = 256-bit (at 500MHz)
    parameter NUM_BARS        = 6,
    parameter NUM_MSIX_VEC    = 32
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        pcie_refclk,

    // PCIe PHY interface (simplified)
    input  wire [7:0]                  rx_data,         // Simplified 8-bit PHY
    input  wire                        rx_data_valid,
    output reg  [7:0]                  tx_data,
    output reg                         tx_data_valid,

    // Link status
    output reg                         link_up,
    output reg  [2:0]                  link_speed,      // 1=Gen1, 2=Gen2, 3=Gen3, 4=Gen4
    output reg  [3:0]                  link_width,      // 1, 2, 4, 8

    // TLP receive interface
    output reg                         tlp_rx_valid,
    output reg  [DATA_WIDTH-1:0]       tlp_rx_data,
    output reg  [15:0]                 tlp_rx_length,
    output reg  [2:0]                  tlp_rx_type,     // MEM_RD, MEM_WR, CFG, etc.
    output reg  [15:0]                 tlp_rx_req_id,
    output reg  [7:0]                  tlp_rx_tag,
    input  wire                        tlp_rx_ready,

    // TLP transmit interface
    input  wire                        tlp_tx_valid,
    input  wire [DATA_WIDTH-1:0]       tlp_tx_data,
    input  wire [15:0]                 tlp_tx_length,
    input  wire [2:0]                  tlp_tx_type,
    output reg                         tlp_tx_ready,

    // BAR decoded address output (to AXI4-Lite slave)
    output reg                         bar_hit_valid,
    output reg  [2:0]                  bar_hit_num,
    output reg  [ADDR_WIDTH-1:0]       bar_addr,
    output reg  [DATA_WIDTH-1:0]       bar_wdata,
    output reg                         bar_we,
    input  wire [DATA_WIDTH-1:0]       bar_rdata,
    input  wire                        bar_rvalid,

    // MSI-X interrupt interface
    input  wire [NUM_MSIX_VEC-1:0]     msix_request,
    output reg  [NUM_MSIX_VEC-1:0]     msix_pending,

    // Configuration space (simplified)
    input  wire [31:0]                 cfg_bar [0:NUM_BARS-1],
    input  wire [15:0]                 cfg_vendor_id,
    input  wire [15:0]                 cfg_device_id
);

    //--------------------------------------------------------------------------
    // TLP type encoding
    //--------------------------------------------------------------------------
    localparam TLP_MEM_RD   = 3'b000;
    localparam TLP_MEM_WR   = 3'b001;
    localparam TLP_CFG_RD   = 3'b010;
    localparam TLP_CFG_WR   = 3'b011;
    localparam TLP_CPL      = 3'b100;
    localparam TLP_CPL_D    = 3'b101;
    localparam TLP_MSG      = 3'b110;
    localparam TLP_MSG_D    = 3'b111;

    //--------------------------------------------------------------------------
    // PCIe link training state machine
    //--------------------------------------------------------------------------
    localparam LTSSM_DETECT     = 4'b0000;
    localparam LTSSM_POLLING    = 4'b0001;
    localparam LTSSM_CONFIG     = 4'b0010;
    localparam LTSSM_L0         = 4'b0011;
    localparam LTSSM_RECOVERY   = 4'b0100;
    localparam LTSSM_L0S        = 4'b0101;
    localparam LTSSM_L1         = 4'b0110;
    localparam LTSSM_L2         = 4'b0111;
    localparam LTSSM_DISABLED   = 4'b1000;
    localparam LTSSM_LOOPBACK   = 4'b1001;
    localparam LTSSM_HOT_RESET  = 4'b1010;

    reg [3:0] ltssm_state;
    reg [15:0] ltssm_timer;
    reg [3:0]  ltssm_lane_num;
    reg [7:0]  ltssm_ts_count;

    //--------------------------------------------------------------------------
    // BAR decoding
    //--------------------------------------------------------------------------
    reg [31:0] bar_mask [0:NUM_BARS-1];
    reg [31:0] bar_size [0:NUM_BARS-1];
    reg        bar_enabled [0:NUM_BARS-1];
    reg        bar_64bit [0:NUM_BARS-1];
    reg        bar_prefetchable [0:NUM_BARS-1];

    // Detect BAR hit
    reg [NUM_BARS-1:0] bar_hit_vec;
    integer bi;

    always @(*) begin
        bar_hit_vec = {NUM_BARS{1'b0}};
        for (bi = 0; bi < NUM_BARS; bi = bi + 1) begin
            if (bar_enabled[bi]) begin
                if ((tlp_rx_data[31:0] & bar_mask[bi]) == (cfg_bar[bi] & bar_mask[bi])) begin
                    bar_hit_vec[bi] = 1'b1;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // TLP receive logic
    //--------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] rx_tlp_buffer;
    reg [7:0]            rx_byte_cnt;
    reg                  rx_tlp_active;
    reg [3:0]            rx_dword_cnt;

    // TLP format and type from header
    wire [2:0] rx_tlp_fmt  = rx_tlp_buffer[30:28];
    wire [4:0] rx_tlp_type = rx_tlp_buffer[27:23];
    wire [9:0] rx_tlp_len  = rx_tlp_buffer[9:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ltssm_state   <= LTSSM_DETECT;
            ltssm_timer   <= 16'd0;
            link_up       <= 1'b0;
            link_speed    <= 3'd0;
            link_width    <= 4'd0;
            rx_tlp_active <= 1'b0;
            rx_byte_cnt   <= 8'd0;
            rx_dword_cnt  <= 4'd0;
            tlp_rx_valid  <= 1'b0;
            bar_hit_valid <= 1'b0;
            tlp_tx_ready  <= 1'b1;
        end else begin
            // Default: clear single-cycle signals
            tlp_rx_valid  <= 1'b0;
            bar_hit_valid <= 1'b0;

            // LTSSM state machine (simplified)
            case (ltssm_state)
                LTSSM_DETECT: begin
                    ltssm_timer <= ltssm_timer + 1'b1;
                    if (ltssm_timer >= 16'd100) begin
                        ltssm_state <= LTSSM_POLLING;
                        ltssm_timer <= 16'd0;
                    end
                end

                LTSSM_POLLING: begin
                    ltssm_timer <= ltssm_timer + 1'b1;
                    if (ltssm_timer >= 16'd200) begin
                        ltssm_state <= LTSSM_CONFIG;
                        ltssm_timer <= 16'd0;
                    end
                end

                LTSSM_CONFIG: begin
                    ltssm_timer <= ltssm_timer + 1'b1;
                    ltssm_lane_num <= ltssm_lane_num + 1'b1;
                    if (ltssm_lane_num >= 4'd8) begin
                        ltssm_state <= LTSSM_L0;
                        link_up     <= 1'b1;
                        link_speed  <= 3'd4;  // Gen4
                        link_width  <= 4'd8;  // x8
                    end
                end

                LTSSM_L0: begin
                    link_up <= 1'b1;
                end

                default: ltssm_state <= LTSSM_DETECT;
            endcase

            // TLP reception
            if (link_up) begin
                if (rx_data_valid) begin
                    if (!rx_tlp_active) begin
                        // Start of TLP
                        rx_tlp_active <= 1'b1;
                        rx_tlp_buffer <= {rx_tlp_buffer[DATA_WIDTH-9:0], rx_data};
                        rx_byte_cnt   <= rx_byte_cnt + 1'b1;
                    end else begin
                        rx_tlp_buffer <= {rx_tlp_buffer[DATA_WIDTH-9:0], rx_data};
                        rx_byte_cnt   <= rx_byte_cnt + 1'b1;

                        // Check if we have a complete TLP header (12 bytes for 3 DW)
                        if (rx_byte_cnt >= 8'd11) begin
                            rx_tlp_active <= 1'b0;
                            rx_byte_cnt   <= 8'd0;

                            // Decode TLP type
                            case (rx_tlp_type[4:0])
                                5'b00000: tlp_rx_type <= TLP_MEM_RD;   // MRd
                                5'b00001: tlp_rx_type <= TLP_MEM_RD;   // MRdLk
                                5'b10000: tlp_rx_type <= TLP_MEM_WR;   // MWr
                                5'b00100: tlp_rx_type <= TLP_CFG_RD;   // CfgRd0
                                5'b00101: tlp_rx_type <= TLP_CFG_WR;   // CfgWr0
                                5'b01010: tlp_rx_type <= TLP_CPL;      // Cpl
                                5'b01011: tlp_rx_type <= TLP_CPL_D;    // CplD
                                default:  tlp_rx_type <= TLP_MSG;
                            endcase

                            tlp_rx_valid  <= 1'b1;
                            tlp_rx_data   <= rx_tlp_buffer;
                            tlp_rx_length <= {6'd0, rx_tlp_len};
                            tlp_rx_req_id <= rx_tlp_buffer[63:48];
                            tlp_rx_tag    <= rx_tlp_buffer[47:40];

                            // BAR decode
                            if (|bar_hit_vec) begin
                                bar_hit_valid <= 1'b1;
                                case (bar_hit_vec)
                                    6'b000001: bar_hit_num <= 3'd0;
                                    6'b000010: bar_hit_num <= 3'd1;
                                    6'b000100: bar_hit_num <= 3'd2;
                                    6'b001000: bar_hit_num <= 3'd3;
                                    6'b010000: bar_hit_num <= 3'd4;
                                    6'b100000: bar_hit_num <= 3'd5;
                                    default:   bar_hit_num <= 3'd0;
                                endcase
                                bar_addr <= rx_tlp_buffer[31:0] & ~bar_mask[0];
                                bar_we   <= (tlp_rx_type == TLP_MEM_WR);
                                if (tlp_rx_type == TLP_MEM_WR) begin
                                    bar_wdata <= rx_tlp_buffer[DATA_WIDTH-1:32];
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // TLP transmit logic
    //--------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] tx_tlp_buffer;
    reg [7:0]            tx_byte_cnt;
    reg                  tx_tlp_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data       <= 8'd0;
            tx_data_valid <= 1'b0;
            tx_tlp_active <= 1'b0;
            tx_byte_cnt   <= 8'd0;
        end else begin
            tx_data_valid <= 1'b0;

            if (link_up && tlp_tx_valid && tlp_tx_ready && !tx_tlp_active) begin
                tx_tlp_active <= 1'b1;
                tx_tlp_buffer <= tlp_tx_data;
                tx_byte_cnt   <= 8'd0;
            end

            if (tx_tlp_active) begin
                // Transmit one byte per cycle (simplified)
                tx_data       <= tx_tlp_buffer[7:0];
                tx_data_valid <= 1'b1;
                tx_tlp_buffer <= {8'd0, tx_tlp_buffer[DATA_WIDTH-1:8]};
                tx_byte_cnt   <= tx_byte_cnt + 1'b1;

                if (tx_byte_cnt >= tlp_tx_length[7:0]) begin
                    tx_tlp_active <= 1'b0;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // MSI-X interrupt handling
    //--------------------------------------------------------------------------
    reg [NUM_MSIX_VEC-1:0] msix_enabled;
    reg [31:0]             msix_table_addr;
    reg [31:0]             msix_pba_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msix_pending  <= {NUM_MSIX_VEC{1'b0}};
            msix_enabled  <= {NUM_MSIX_VEC{1'b0}};
        end else begin
            // Capture interrupt requests
            msix_pending <= msix_pending | msix_request;

            // Clear handled interrupts
            if (tlp_tx_valid && tlp_tx_type == TLP_MSG) begin
                // MSI-X message sent - clear pending
                msix_pending <= msix_pending & ~msix_request;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Configuration space initialization
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (bi = 0; bi < NUM_BARS; bi = bi + 1) begin
                bar_mask[bi] <= 32'hFFFF_F000;  // 4KB minimum
                bar_size[bi] <= 32'h0000_1000;  // 4KB
                bar_enabled[bi] <= 1'b1;
                bar_64bit[bi] <= 1'b0;
                bar_prefetchable[bi] <= 1'b0;
            end
        end
    end

endmodule
