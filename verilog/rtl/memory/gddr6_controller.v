//------------------------------------------------------------------------------
// Module: gddr6_controller
// Description: GDDR6 memory controller with 256-bit data bus
//              8 x 32-bit channels, 16 GT/s data rate
//              BL16 burst mode
//              Read latency: 32 cycles
//              16 bank groups x 4 banks = 64 banks total
//------------------------------------------------------------------------------

module gddr6_controller #(
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 256,       // 8 x 32-bit channels
    parameter NUM_CHANNELS    = 8,
    parameter NUM_BANK_GROUPS = 16,
    parameter BANKS_PER_GROUP = 4,
    parameter TOTAL_BANKS     = NUM_BANK_GROUPS * BANKS_PER_GROUP,  // 64
    parameter BURST_LENGTH    = 16,        // BL16
    parameter READ_LATENCY    = 32
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        clk_x2,        // 2x clock for DDR

    // PHY interface
    output reg  [NUM_CHANNELS-1:0]     phy_ck,        // Clock to DRAM
    output reg  [NUM_CHANNELS-1:0]     phy_cke,       // Clock enable
    output reg  [NUM_CHANNELS-1:0]     phy_cs_n,      // Chip select
    output reg  [NUM_CHANNELS-1:0]     phy_ca,        // Command/address (shared)
    output reg  [NUM_CHANNELS-1:0]     phy_dqm,       // Data mask
    inout  wire [DATA_WIDTH-1:0]       phy_dq,        // Bidirectional data
    output reg  [NUM_CHANNELS-1:0]     phy_dqs,       // Data strobe

    // Controller request interface
    input  wire                        req_valid,
    input  wire                        req_we,
    input  wire [ADDR_WIDTH-1:0]       req_addr,
    input  wire [DATA_WIDTH-1:0]       req_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]   req_be,
    output reg                         req_ready,

    // Controller response interface
    output reg                         resp_valid,
    output reg  [DATA_WIDTH-1:0]       resp_rdata,

    // Initialization status
    output reg                         init_done,

    // Status
    output reg  [TOTAL_BANKS-1:0]      bank_active,
    output reg  [15:0]                 refresh_counter
);

    //--------------------------------------------------------------------------
    // GDDR6 command encoding
    //--------------------------------------------------------------------------
    localparam CMD_NOP       = 4'b0000;
    localparam CMD_ACTIVE    = 4'b0001;  // Activate row
    localparam CMD_READ      = 4'b0010;  // Read burst
    localparam CMD_WRITE     = 4'b0011;  // Write burst
    localparam CMD_PRECHARGE = 4'b0100;  // Precharge bank
    localparam CMD_REFRESH   = 4'b0101;  // Refresh
    localparam CMD_MRS       = 4'b0110;  // Mode register set
    localparam CMD_ZQCS      = 4'b0111;  // ZQ calibration short

    //--------------------------------------------------------------------------
    // Timing parameters (in clock cycles)
    //--------------------------------------------------------------------------
    localparam T_RCD   = 14;   // RAS to CAS delay
    localparam T_CL    = 16;   // CAS latency
    localparam T_RP    = 14;   // Row precharge time
    localparam T_RAS   = 33;   // Row active time
    localparam T_RRD   = 4;    // Row to row delay
    localparam T_WR    = 12;   // Write recovery
    localparam T_RTP   = 4;    // Read to precharge
    localparam T_CCD   = 2;    // CAS to CAS delay
    localparam T_WTR   = 4;    // Write to read delay
    localparam T_RTW   = 6;    // Read to write delay
    localparam T_FAW   = 21;   // Four activate window
    localparam T_REF_I = 3900; // Refresh interval (~7.8us at 500MHz)

    //--------------------------------------------------------------------------
    // Address decomposition
    // [31:18] Row address (14 bits)
    // [17:12] Bank group (6 bits for 64 banks)  
    // [11:6]  Bank within group (4 bits)
    // [5:0]   Column address / burst offset (6 bits)
    //--------------------------------------------------------------------------
    localparam ROW_ADDR_W    = 14;
    localparam BANK_GROUP_W  = 6;   // log2(64) since we have 64 total banks
    localparam BANK_W        = 2;   // log2(4) within group
    localparam COL_ADDR_W    = 6;

    wire [ROW_ADDR_W-1:0]   row_addr   = req_addr[ADDR_WIDTH-1:ADDR_WIDTH-ROW_ADDR_W];
    wire [BANK_GROUP_W-1:0] bank_group = req_addr[ADDR_WIDTH-ROW_ADDR_W-1:ADDR_WIDTH-ROW_ADDR_W-BANK_GROUP_W];
    wire [BANK_W-1:0]       bank       = req_addr[ADDR_WIDTH-ROW_ADDR_W-BANK_GROUP_W-1:ADDR_WIDTH-ROW_ADDR_W-BANK_GROUP_W-BANK_W];
    wire [COL_ADDR_W-1:0]   col_addr   = req_addr[COL_ADDR_W-1:0];

    wire [5:0] bank_id = {bank_group, bank};  // 0-63

    //--------------------------------------------------------------------------
    // Bank state tracking
    //--------------------------------------------------------------------------
    reg [ROW_ADDR_W-1:0] bank_row [0:TOTAL_BANKS-1];
    reg                  bank_open [0:TOTAL_BANKS-1];
    reg [5:0]            bank_timer [0:TOTAL_BANKS-1];  // Time since last activate
    reg [5:0]            bank_act_count;                // Activates in FAW window

    // Command FIFO
    localparam CMD_FIFO_DEPTH = 16;
    reg [ADDR_WIDTH-1:0] cmd_fifo_addr [0:CMD_FIFO_DEPTH-1];
    reg                  cmd_fifo_we   [0:CMD_FIFO_DEPTH-1];
    reg                  cmd_fifo_valid [0:CMD_FIFO_DEPTH-1];
    reg [DATA_WIDTH-1:0] cmd_fifo_wdata [0:CMD_FIFO_DEPTH-1];
    reg [(DATA_WIDTH/8)-1:0] cmd_fifo_be [0:CMD_FIFO_DEPTH-1];
    reg [3:0]            cmd_fifo_rd_ptr;
    reg [3:0]            cmd_fifo_wr_ptr;
    reg [4:0]            cmd_fifo_count;

    //--------------------------------------------------------------------------
    // Data path
    //--------------------------------------------------------------------------
    // Write data FIFO
    localparam WR_FIFO_DEPTH = 16;
    reg [DATA_WIDTH-1:0] wr_fifo_data [0:WR_FIFO_DEPTH-1];
    reg [(DATA_WIDTH/8)-1:0] wr_fifo_be [0:WR_FIFO_DEPTH-1];
    reg [3:0]            wr_fifo_rd_ptr;
    reg [3:0]            wr_fifo_wr_ptr;

    // Read data pipeline (delay line for READ_LATENCY)
    reg [DATA_WIDTH-1:0] rd_pipe [0:READ_LATENCY-1];
    reg                  rd_pipe_valid [0:READ_LATENCY-1];

    //--------------------------------------------------------------------------
    // Initialization state machine
    //--------------------------------------------------------------------------
    localparam INIT_RESET    = 4'b0000;
    localparam INIT_WAIT     = 4'b0001;
    localparam INIT_ZQ       = 4'b0010;
    localparam INIT_MRS      = 4'b0011;
    localparam INIT_CALIB    = 4'b0100;
    localparam INIT_DONE     = 4'b0101;

    reg [3:0]  init_state;
    reg [15:0] init_counter;
    reg [2:0]  init_mrs_num;

    //--------------------------------------------------------------------------
    // Main state machine
    //--------------------------------------------------------------------------
    localparam CTRL_IDLE      = 3'b000;
    localparam CTRL_ACTIVATE  = 3'b001;
    localparam CTRL_READ      = 3'b010;
    localparam CTRL_WRITE     = 3'b011;
    localparam CTRL_PRECHARGE = 3'b100;
    localparam CTRL_REFRESH   = 3'b101;

    reg [2:0] ctrl_state;
    reg [ADDR_WIDTH-1:0] current_addr;
    reg                  current_we;
    reg [5:0]           current_bank;
    reg [ROW_ADDR_W-1:0] current_row;
    reg [COL_ADDR_W-1:0] current_col;

    // Timing counters
    reg [5:0]  t_rcd_counter [0:TOTAL_BANKS-1];
    reg [5:0]  t_ras_counter [0:TOTAL_BANKS-1];
    reg [5:0]  t_rp_counter  [0:TOTAL_BANKS-1];
    reg [5:0]  t_rtw_counter;
    reg [5:0]  t_wtr_counter;
    reg [5:0]  t_ccd_counter;

    integer b, p;

    //--------------------------------------------------------------------------
    // Initialization sequence
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_state   <= INIT_RESET;
            init_counter <= 16'd0;
            init_mrs_num <= 3'd0;
            init_done    <= 1'b0;
            phy_ck       <= {NUM_CHANNELS{1'b0}};
            phy_cke      <= {NUM_CHANNELS{1'b0}};
            phy_cs_n     <= {NUM_CHANNELS{1'b1}};
            phy_ca       <= {NUM_CHANNELS{1'b0}};
            phy_dqm      <= {NUM_CHANNELS{1'b0}};
            phy_dqs      <= {NUM_CHANNELS{1'b0}};
        end else begin
            case (init_state)
                INIT_RESET: begin
                    phy_cke  <= {NUM_CHANNELS{1'b0}};
                    phy_cs_n <= {NUM_CHANNELS{1'b1}};
                    init_counter <= init_counter + 1'b1;
                    if (init_counter >= 16'd200) begin  // 200us reset pulse (simplified)
                        init_state   <= INIT_WAIT;
                        init_counter <= 16'd0;
                    end
                end

                INIT_WAIT: begin
                    phy_cke <= {NUM_CHANNELS{1'b1}};
                    init_counter <= init_counter + 1'b1;
                    if (init_counter >= 16'd100) begin  // Wait tXPR
                        init_state   <= INIT_ZQ;
                        init_counter <= 16'd0;
                    end
                end

                INIT_ZQ: begin
                    // Issue ZQ calibration
                    phy_cs_n <= {NUM_CHANNELS{1'b0}};
                    phy_ca   <= {NUM_CHANNELS{CMD_ZQCS[0]}};
                    init_counter <= init_counter + 1'b1;
                    if (init_counter >= 16'd128) begin  // Wait tZQCS
                        init_state   <= INIT_MRS;
                        init_counter <= 16'd0;
                    end
                end

                INIT_MRS: begin
                    // Program mode registers
                    phy_cs_n <= {NUM_CHANNELS{1'b0}};
                    case (init_mrs_num)
                        3'd0: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR0
                        3'd1: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR1
                        3'd2: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR2
                        3'd3: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR3
                        3'd4: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR4
                        3'd5: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR5
                        3'd6: phy_ca <= {NUM_CHANNELS{CMD_MRS[0]}};  // MR6
                        default: phy_ca <= {NUM_CHANNELS{CMD_NOP[0]}};
                    endcase
                    init_counter <= init_counter + 1'b1;
                    if (init_counter >= 16'd10) begin
                        init_counter <= 16'd0;
                        init_mrs_num <= init_mrs_num + 1'b1;
                        if (init_mrs_num >= 3'd6) begin
                            init_state <= INIT_CALIB;
                        end
                    end
                end

                INIT_CALIB: begin
                    // Read/write leveling calibration (simplified)
                    phy_cs_n <= {NUM_CHANNELS{1'b1}};
                    init_counter <= init_counter + 1'b1;
                    if (init_counter >= 16'd512) begin
                        init_state   <= INIT_DONE;
                        init_counter <= 16'd0;
                    end
                end

                INIT_DONE: begin
                    init_done <= 1'b1;
                    phy_ck    <= {NUM_CHANNELS{1'b1}};
                end

                default: init_state <= INIT_RESET;
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // Command FIFO management
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_fifo_rd_ptr <= 4'd0;
            cmd_fifo_wr_ptr <= 4'd0;
            cmd_fifo_count  <= 5'd0;
            for (b = 0; b < CMD_FIFO_DEPTH; b = b + 1) begin
                cmd_fifo_valid[b] <= 1'b0;
            end
        end else begin
            // Write to FIFO
            if (req_valid && req_ready && init_done) begin
                cmd_fifo_addr[cmd_fifo_wr_ptr]  <= req_addr;
                cmd_fifo_we[cmd_fifo_wr_ptr]    <= req_we;
                cmd_fifo_wdata[cmd_fifo_wr_ptr] <= req_wdata;
                cmd_fifo_be[cmd_fifo_wr_ptr]    <= req_be;
                cmd_fifo_valid[cmd_fifo_wr_ptr] <= 1'b1;
                cmd_fifo_wr_ptr <= cmd_fifo_wr_ptr + 1'b1;
                cmd_fifo_count  <= cmd_fifo_count + 1'b1;
            end

            // Read from FIFO
            if (ctrl_state == CTRL_IDLE && cmd_fifo_count > 0 &&
                cmd_fifo_valid[cmd_fifo_rd_ptr]) begin
                cmd_fifo_valid[cmd_fifo_rd_ptr] <= 1'b0;
                cmd_fifo_rd_ptr <= cmd_fifo_rd_ptr + 1'b1;
                cmd_fifo_count  <= cmd_fifo_count - 1'b1;
            end
        end
    end

    // Ready when FIFO has space
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_ready <= 1'b0;
        end else begin
            req_ready <= init_done && (cmd_fifo_count < CMD_FIFO_DEPTH - 2);
        end
    end

    //--------------------------------------------------------------------------
    // Main controller state machine
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_state    <= CTRL_IDLE;
            current_addr  <= {ADDR_WIDTH{1'b0}};
            current_we    <= 1'b0;
            current_bank  <= 6'd0;
            current_row   <= {ROW_ADDR_W{1'b0}};
            current_col   <= {COL_ADDR_W{1'b0}};
            resp_valid    <= 1'b0;
            resp_rdata    <= {DATA_WIDTH{1'b0}};
            bank_active   <= {TOTAL_BANKS{1'b0}};
            refresh_counter <= 16'd0;
            t_rtw_counter <= 6'd0;
            t_wtr_counter <= 6'd0;
            t_ccd_counter <= 6'd0;

            for (b = 0; b < TOTAL_BANKS; b = b + 1) begin
                bank_row[b]    <= {ROW_ADDR_W{1'b0}};
                bank_open[b]   <= 1'b0;
                bank_timer[b]  <= 6'd0;
                t_rcd_counter[b] <= 6'd0;
                t_ras_counter[b] <= 6'd0;
                t_rp_counter[b]  <= 6'd0;
            end
        end else begin
            if (init_done) begin
                // Update refresh counter
                refresh_counter <= refresh_counter + 1'b1;

                // Update timing counters
                t_rtw_counter <= (t_rtw_counter > 0) ? t_rtw_counter - 1'b1 : 6'd0;
                t_wtr_counter <= (t_wtr_counter > 0) ? t_wtr_counter - 1'b1 : 6'd0;
                t_ccd_counter <= (t_ccd_counter > 0) ? t_ccd_counter - 1'b1 : 6'd0;

                for (b = 0; b < TOTAL_BANKS; b = b + 1) begin
                    if (t_rcd_counter[b] > 0) t_rcd_counter[b] <= t_rcd_counter[b] - 1'b1;
                    if (t_ras_counter[b] > 0) t_ras_counter[b] <= t_ras_counter[b] - 1'b1;
                    if (t_rp_counter[b] > 0)  t_rp_counter[b]  <= t_rp_counter[b] - 1'b1;
                end

                resp_valid <= 1'b0;

                case (ctrl_state)
                    CTRL_IDLE: begin
                        if (refresh_counter >= T_REF_I) begin
                            ctrl_state <= CTRL_REFRESH;
                            refresh_counter <= 16'd0;
                        end else if (cmd_fifo_count > 0 &&
                                   cmd_fifo_valid[cmd_fifo_rd_ptr]) begin
                            // Pop command
                            current_addr  <= cmd_fifo_addr[cmd_fifo_rd_ptr];
                            current_we    <= cmd_fifo_we[cmd_fifo_rd_ptr];
                            current_bank  <= {cmd_fifo_addr[cmd_fifo_rd_ptr][17:12],
                                                cmd_fifo_addr[cmd_fifo_rd_ptr][11:10]};
                            current_row   <= cmd_fifo_addr[cmd_fifo_rd_ptr][31:18];
                            current_col   <= cmd_fifo_addr[cmd_fifo_rd_ptr][5:0];

                            if (bank_open[current_bank]) begin
                                if (bank_row[current_bank] == current_row) begin
                                    // Row hit
                                    if (current_we && t_wtr_counter == 0) begin
                                        ctrl_state <= CTRL_WRITE;
                                    end else if (!current_we && t_rtw_counter == 0) begin
                                        ctrl_state <= CTRL_READ;
                                    end
                                end else begin
                                    // Row miss - need to precharge then activate
                                    ctrl_state <= CTRL_PRECHARGE;
                                end
                            end else begin
                                // Bank closed - activate row
                                ctrl_state <= CTRL_ACTIVATE;
                            end
                        end
                    end

                    CTRL_ACTIVATE: begin
                        // Issue activate command
                        phy_cs_n <= {NUM_CHANNELS{1'b0}};
                        phy_ca   <= {NUM_CHANNELS{CMD_ACTIVE[0]}};
                        bank_open[current_bank] <= 1'b1;
                        bank_row[current_bank]  <= current_row;
                        bank_active[current_bank] <= 1'b1;
                        t_rcd_counter[current_bank] <= T_RCD;
                        t_ras_counter[current_bank] <= T_RAS;

                        // Wait for tRCD
                        if (t_rcd_counter[current_bank] == 0) begin
                            if (current_we) begin
                                ctrl_state <= CTRL_WRITE;
                            end else begin
                                ctrl_state <= CTRL_READ;
                            end
                        end
                    end

                    CTRL_READ: begin
                        // Issue read command
                        phy_cs_n <= {NUM_CHANNELS{1'b0}};
                        phy_ca   <= {NUM_CHANNELS{CMD_READ[0]}};
                        t_rtw_counter <= T_RTW;
                        t_ccd_counter <= T_CCD;

                        // Push read into delay pipeline
                        rd_pipe[0]       <= {DATA_WIDTH{1'b0}};  // Placeholder
                        rd_pipe_valid[0] <= 1'b1;

                        // Shift pipeline
                        for (p = READ_LATENCY-1; p > 0; p = p - 1) begin
                            rd_pipe[p]       <= rd_pipe[p-1];
                            rd_pipe_valid[p] <= rd_pipe_valid[p-1];
                        end

                        // Check if data is ready
                        if (rd_pipe_valid[READ_LATENCY-1]) begin
                            resp_valid <= 1'b1;
                            resp_rdata <= rd_pipe[READ_LATENCY-1];
                        end

                        ctrl_state <= CTRL_IDLE;
                    end

                    CTRL_WRITE: begin
                        // Issue write command
                        phy_cs_n <= {NUM_CHANNELS{1'b0}};
                        phy_ca   <= {NUM_CHANNELS{CMD_WRITE[0]}};
                        phy_dqm  <= ~cmd_fifo_be[cmd_fifo_rd_ptr][NUM_CHANNELS-1:0];
                        t_wtr_counter <= T_WTR;
                        t_ccd_counter <= T_CCD;

                        ctrl_state <= CTRL_IDLE;
                    end

                    CTRL_PRECHARGE: begin
                        // Precharge bank
                        phy_cs_n <= {NUM_CHANNELS{1'b0}};
                        phy_ca   <= {NUM_CHANNELS{CMD_PRECHARGE[0]}};
                        bank_open[current_bank] <= 1'b0;
                        bank_active[current_bank] <= 1'b0;
                        t_rp_counter[current_bank] <= T_RP;

                        if (t_rp_counter[current_bank] == 0) begin
                            ctrl_state <= CTRL_ACTIVATE;
                        end
                    end

                    CTRL_REFRESH: begin
                        // Refresh all banks
                        phy_cs_n <= {NUM_CHANNELS{1'b0}};
                        phy_ca   <= {NUM_CHANNELS{CMD_REFRESH[0]}};

                        // Precharge all banks first
                        for (b = 0; b < TOTAL_BANKS; b = b + 1) begin
                            bank_open[b] <= 1'b0;
                            bank_active[b] <= 1'b0;
                        end

                        ctrl_state <= CTRL_IDLE;
                    end

                    default: ctrl_state <= CTRL_IDLE;
                endcase
            end
        end
    end

endmodule
