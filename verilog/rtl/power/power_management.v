//------------------------------------------------------------------------------
// Module: power_management
// Description: GPU power management unit
//              Clock gating control per domain
//              Power domain management
//              DVFS (Dynamic Voltage Frequency Scaling) interface
//              Thermal throttling
//------------------------------------------------------------------------------

module power_management #(
    parameter NUM_CLOCK_DOMAINS = 8,
    parameter NUM_POWER_DOMAINS = 6,
    parameter NUM_DVFS_LEVELS   = 8,
    parameter TEMP_SENSOR_W     = 10
)(
    input  wire                        clk,
    input  wire                        rst_n,

    // Control inputs (from AXI4-Lite registers)
    input  wire [NUM_CLOCK_DOMAINS-1:0] clk_gate_enable,
    input  wire [NUM_POWER_DOMAINS-1:0] pd_enable,
    input  wire [2:0]                   dvfs_level_req,
    input  wire                         dvfs_enable,
    input  wire                         thermal_throttle_enable,
    input  wire [TEMP_SENSOR_W-1:0]     thermal_threshold,

    // Clock gating outputs
    output reg  [NUM_CLOCK_DOMAINS-1:0] clk_gate_en,
    output reg  [NUM_CLOCK_DOMAINS-1:0] clk_gate_ack,

    // Power domain control
    output reg  [NUM_POWER_DOMAINS-1:0] pd_power_on,
    output reg  [NUM_POWER_DOMAINS-1:0] pd_iso_en,
    output reg  [NUM_POWER_DOMAINS-1:0] pd_retention,
    input  wire [NUM_POWER_DOMAINS-1:0] pd_power_ok,

    // DVFS interface
    output reg  [2:0]                   dvfs_current_level,
    output reg                          dvfs_req_valid,
    output reg  [7:0]                   dvfs_voltage_sel,    // Voltage selector
    output reg  [7:0]                   dvfs_freq_sel,       // Frequency selector
    input  wire                         dvfs_ack,

    // Thermal interface
    input  wire [TEMP_SENSOR_W-1:0]     temp_sensor_value,
    output reg                          thermal_throttle_active,
    output reg  [2:0]                   thermal_throttle_level,
    output reg                          thermal_shutdown,

    // Activity counters (from SMs)
    input  wire [7:0]                   sm_activity [0:3],   // 4 SMs
    input  wire [15:0]                  mem_activity,

    // Status outputs
    output reg  [TEMP_SENSOR_W-1:0]     current_temp,
    output reg  [NUM_DVFS_LEVELS-1:0]   dvfs_level_status
);

    //--------------------------------------------------------------------------
    // Clock domain definitions
    //--------------------------------------------------------------------------
    localparam CLK_DOMAIN_CORE     = 0;  // SM cores
    localparam CLK_DOMAIN_L2       = 1;  // L2 cache
    localparam CLK_DOMAIN_MEM      = 2;  // Memory controller
    localparam CLK_DOMAIN_DMA      = 3;  // DMA engine
    localparam CLK_DOMAIN_PCIE     = 4;  // PCIe interface
    localparam CLK_DOMAIN_HOST     = 5;  // Host interface
    localparam CLK_DOMAIN_PWR      = 6;  // Power management (always on)
    localparam CLK_DOMAIN_JTAG     = 7;  // JTAG/debug (always on)

    //--------------------------------------------------------------------------
    // Power domain definitions
    //--------------------------------------------------------------------------
    localparam PD_SM_CLUSTER  = 0;  // SM cluster (all 4 SMs)
    localparam PD_L2_CACHE    = 1;  // L2 cache
    localparam PD_MEMORY      = 2;  // GDDR6 controller
    localparam PD_DMA_PCIE    = 3;  // DMA and PCIe
    localparam PD_HOST        = 4;  // Host interface
    localparam PD_ALWAYS_ON   = 5;  // Always-on domain

    //--------------------------------------------------------------------------
    // DVFS levels (frequency/voltage pairs)
    // Level 0: Lowest power
    // Level 7: Maximum performance
    //--------------------------------------------------------------------------
    // Voltage table (encoded values for external regulator)
    reg [7:0] dvfs_voltage_table [0:NUM_DVFS_LEVELS-1];
    // Frequency table (encoded values for PLL)
    reg [7:0] dvfs_frequency_table [0:NUM_DVFS_LEVELS-1];

    // DVFS state machine
    localparam DVFS_IDLE      = 2'b00;
    localparam DVFS_REQ_UP    = 2'b01;
    localparam DVFS_REQ_DOWN  = 2'b10;
    localparam DVFS_WAIT_ACK  = 2'b11;

    reg [1:0] dvfs_state;
    reg [2:0] dvfs_target_level;
    reg [15:0] dvfs_timer;

    //--------------------------------------------------------------------------
    // Thermal management
    //--------------------------------------------------------------------------
    // Temperature thresholds (in sensor units)
    localparam TEMP_NOMINAL   = 10'd500;  // 50C (example)
    localparam TEMP_WARN      = 10'd750;  // 75C
    localparam TEMP_THROTTLE  = 10'd850;  // 85C
    localparam TEMP_SHUTDOWN  = 10'd950;  // 95C

    // Thermal state machine
    localparam THERM_NOMINAL    = 3'b000;
    localparam THERM_WARNING    = 3'b001;
    localparam THERM_THROTTLING = 3'b010;
    localparam THERM_SHUTDOWN   = 3'b011;

    reg [2:0] thermal_state;
    reg [15:0] thermal_timer;
    reg [TEMP_SENSOR_W-1:0] temp_avg;
    reg [TEMP_SENSOR_W-1:0] temp_history [0:7];  // Moving average window
    reg [2:0] temp_hist_idx;

    // Activity-based power estimation
    reg [15:0] total_activity;
    reg [2:0]  activity_dvfs_level;

    integer i, di;

    //--------------------------------------------------------------------------
    // Initialize DVFS tables
    //--------------------------------------------------------------------------
    initial begin
        // Voltage levels (arbitrary units for external regulator)
        dvfs_voltage_table[0] = 8'd50;   // 0.5V
        dvfs_voltage_table[1] = 8'd55;
        dvfs_voltage_table[2] = 8'd60;
        dvfs_voltage_table[3] = 8'd65;
        dvfs_voltage_table[4] = 8'd70;
        dvfs_voltage_table[5] = 8'd75;
        dvfs_voltage_table[6] = 8'd80;
        dvfs_voltage_table[7] = 8'd85;   // 0.85V

        // Frequency levels (arbitrary units for PLL)
        dvfs_frequency_table[0] = 8'd100;  // 100 MHz
        dvfs_frequency_table[1] = 8'd200;
        dvfs_frequency_table[2] = 8'd300;
        dvfs_frequency_table[3] = 8'd400;
        dvfs_frequency_table[4] = 8'd500;
        dvfs_frequency_table[5] = 8'd600;
        dvfs_frequency_table[6] = 8'd800;
        dvfs_frequency_table[7] = 8'd100;  // 1 GHz (placeholder)
    end

    //--------------------------------------------------------------------------
    // Clock gating control
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_gate_en  <= {NUM_CLOCK_DOMAINS{1'b0}};
            clk_gate_ack <= {NUM_CLOCK_DOMAINS{1'b0}};
        end else begin
            // Clock domains 6 and 7 (PWR, JTAG) are always on
            clk_gate_en[CLK_DOMAIN_PWR] <= 1'b1;
            clk_gate_en[CLK_DOMAIN_JTAG] <= 1'b1;
            clk_gate_ack[CLK_DOMAIN_PWR] <= 1'b1;
            clk_gate_ack[CLK_DOMAIN_JTAG] <= 1'b1;

            // Other domains controlled by software
            for (di = 0; di < NUM_CLOCK_DOMAINS - 2; di = di + 1) begin
                if (clk_gate_enable[di]) begin
                    clk_gate_en[di] <= 1'b1;
                    clk_gate_ack[di] <= 1'b1;
                end else if (!thermal_throttle_active) begin
                    clk_gate_en[di] <= 1'b0;
                    clk_gate_ack[di] <= 1'b0;
                end
            end

            // Thermal override: disable clock gates during throttling
            // to maintain minimum operation
            if (thermal_throttle_active) begin
                clk_gate_en[CLK_DOMAIN_CORE] <= 1'b1;
                clk_gate_en[CLK_DOMAIN_L2]   <= 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Power domain management
    //--------------------------------------------------------------------------
    localparam PD_STATE_OFF      = 3'b000;
    localparam PD_STATE_POWER_ON = 3'b001;
    localparam PD_STATE_ISO_ON   = 3'b010;
    localparam PD_STATE_ACTIVE   = 3'b011;
    localparam PD_STATE_RETENTION= 3'b100;
    localparam PD_STATE_ISO_OFF  = 3'b101;
    localparam PD_STATE_POWER_OFF= 3'b110;

    reg [2:0] pd_state [0:NUM_POWER_DOMAINS-1];
    reg [15:0] pd_timer [0:NUM_POWER_DOMAINS-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (di = 0; di < NUM_POWER_DOMAINS; di = di + 1) begin
                pd_power_on[di]  <= 1'b1;  // All on after reset
                pd_iso_en[di]    <= 1'b0;
                pd_retention[di] <= 1'b0;
                pd_state[di]     <= PD_STATE_ACTIVE;
                pd_timer[di]     <= 16'd0;
            end
            // Always-on domain
            pd_power_on[PD_ALWAYS_ON] <= 1'b1;
        end else begin
            for (di = 0; di < NUM_POWER_DOMAINS - 1; di = di + 1) begin
                case (pd_state[di])
                    PD_STATE_OFF: begin
                        if (pd_enable[di]) begin
                            pd_state[di] <= PD_STATE_POWER_ON;
                            pd_power_on[di] <= 1'b1;
                            pd_timer[di] <= 16'd0;
                        end
                    end

                    PD_STATE_POWER_ON: begin
                        pd_timer[di] <= pd_timer[di] + 1'b1;
                        if (pd_power_ok[di] && pd_timer[di] >= 16'd100) begin
                            pd_state[di] <= PD_STATE_ISO_ON;
                            pd_iso_en[di] <= 1'b0;
                        end
                    end

                    PD_STATE_ISO_ON: begin
                        pd_state[di] <= PD_STATE_ACTIVE;
                    end

                    PD_STATE_ACTIVE: begin
                        if (!pd_enable[di]) begin
                            pd_state[di] <= PD_STATE_RETENTION;
                            pd_retention[di] <= 1'b1;
                            pd_timer[di] <= 16'd0;
                        end
                    end

                    PD_STATE_RETENTION: begin
                        pd_timer[di] <= pd_timer[di] + 1'b1;
                        if (pd_timer[di] >= 16'd50) begin
                            pd_state[di] <= PD_STATE_ISO_OFF;
                            pd_iso_en[di] <= 1'b1;
                        end
                    end

                    PD_STATE_ISO_OFF: begin
                        pd_state[di] <= PD_STATE_POWER_OFF;
                        pd_power_on[di] <= 1'b0;
                        pd_retention[di] <= 1'b0;
                    end

                    PD_STATE_POWER_OFF: begin
                        pd_state[di] <= PD_STATE_OFF;
                    end

                    default: pd_state[di] <= PD_STATE_OFF;
                endcase
            end
        end
    end

    //--------------------------------------------------------------------------
    // DVFS control
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dvfs_state        <= DVFS_IDLE;
            dvfs_current_level<= 3'd7;  // Start at max
            dvfs_target_level <= 3'd7;
            dvfs_req_valid    <= 1'b0;
            dvfs_voltage_sel  <= 8'd85;
            dvfs_freq_sel     <= 8'd100;
            dvfs_timer        <= 16'd0;
            dvfs_level_status <= 8'b1000_0000;
        end else begin
            dvfs_req_valid <= 1'b0;

            // Activity-based DVFS estimation
            total_activity <= sm_activity[0] + sm_activity[1] + 
                              sm_activity[2] + sm_activity[3] + mem_activity[7:0];

            // Map activity to DVFS level
            if (total_activity < 16'd50) begin
                activity_dvfs_level <= 3'd1;
            end else if (total_activity < 16'd100) begin
                activity_dvfs_level <= 3'd2;
            end else if (total_activity < 16'd200) begin
                activity_dvfs_level <= 3'd3;
            end else if (total_activity < 16'd400) begin
                activity_dvfs_level <= 3'd4;
            end else if (total_activity < 16'd600) begin
                activity_dvfs_level <= 3'd5;
            end else if (total_activity < 16'd800) begin
                activity_dvfs_level <= 3'd6;
            end else begin
                activity_dvfs_level <= 3'd7;
            end

            case (dvfs_state)
                DVFS_IDLE: begin
                    if (dvfs_enable) begin
                        // Use requested level or activity-based level
                        if (thermal_throttle_active) begin
                            dvfs_target_level <= thermal_throttle_level;
                        end else begin
                            dvfs_target_level <= dvfs_level_req;
                        end

                        if (dvfs_target_level > dvfs_current_level) begin
                            dvfs_state <= DVFS_REQ_UP;
                        end else if (dvfs_target_level < dvfs_current_level) begin
                            dvfs_state <= DVFS_REQ_DOWN;
                        end
                    end
                end

                DVFS_REQ_UP: begin
                    // Request higher frequency - increase voltage first
                    dvfs_req_valid <= 1'b1;
                    dvfs_voltage_sel <= dvfs_voltage_table[dvfs_target_level];
                    dvfs_freq_sel    <= dvfs_frequency_table[dvfs_current_level + 1'b1];
                    dvfs_state <= DVFS_WAIT_ACK;
                end

                DVFS_REQ_DOWN: begin
                    // Request lower frequency - decrease frequency first
                    dvfs_req_valid <= 1'b1;
                    dvfs_voltage_sel <= dvfs_voltage_table[dvfs_current_level - 1'b1];
                    dvfs_freq_sel    <= dvfs_frequency_table[dvfs_target_level];
                    dvfs_state <= DVFS_WAIT_ACK;
                end

                DVFS_WAIT_ACK: begin
                    dvfs_timer <= dvfs_timer + 1'b1;
                    if (dvfs_ack) begin
                        if (dvfs_target_level > dvfs_current_level) begin
                            dvfs_current_level <= dvfs_current_level + 1'b1;
                        end else begin
                            dvfs_current_level <= dvfs_current_level - 1'b1;
                        end
                        dvfs_timer <= 16'd0;
                        dvfs_state <= DVFS_IDLE;
                    end else if (dvfs_timer >= 16'd1000) begin
                        // Timeout - retry
                        dvfs_timer <= 16'd0;
                        dvfs_state <= DVFS_IDLE;
                    end
                end

                default: dvfs_state <= DVFS_IDLE;
            endcase

            // Update status
            dvfs_level_status <= 8'd1 << dvfs_current_level;
        end
    end

    //--------------------------------------------------------------------------
    // Thermal management
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            thermal_state         <= THERM_NOMINAL;
            thermal_timer         <= 16'd0;
            current_temp          <= {TEMP_SENSOR_W{1'b0}};
            thermal_throttle_active <= 1'b0;
            thermal_throttle_level <= 3'd7;
            thermal_shutdown      <= 1'b0;
            temp_avg              <= {TEMP_SENSOR_W{1'b0}};
            temp_hist_idx         <= 3'd0;

            for (i = 0; i < 8; i = i + 1) begin
                temp_history[i] <= {TEMP_SENSOR_W{1'b0}};
            end
        end else begin
            thermal_timer <= thermal_timer + 1'b1;

            // Sample temperature every 1024 cycles
            if (thermal_timer[9:0] == 10'd0) begin
                // Update moving average
                temp_history[temp_hist_idx] <= temp_sensor_value;
                temp_hist_idx <= temp_hist_idx + 1'b1;

                // Compute average
                temp_avg <= (temp_history[0] + temp_history[1] + temp_history[2] + temp_history[3] +
                           temp_history[4] + temp_history[5] + temp_history[6] + temp_history[7]) >> 3;
                current_temp <= temp_avg;
            end

            // Thermal state machine
            case (thermal_state)
                THERM_NOMINAL: begin
                    thermal_throttle_active <= 1'b0;
                    thermal_shutdown <= 1'b0;
                    if (current_temp >= TEMP_WARN) begin
                        thermal_state <= THERM_WARNING;
                    end
                end

                THERM_WARNING: begin
                    if (current_temp >= TEMP_THROTTLE) begin
                        thermal_state <= THERM_THROTTLING;
                        thermal_throttle_active <= 1'b1;
                        // Reduce to 75% frequency
                        thermal_throttle_level <= 3'd5;
                    end else if (current_temp < TEMP_WARN - 10'd50) begin
                        thermal_state <= THERM_NOMINAL;
                    end
                end

                THERM_THROTTLING: begin
                    if (current_temp >= TEMP_SHUTDOWN) begin
                        thermal_state <= THERM_SHUTDOWN;
                        thermal_shutdown <= 1'b1;
                    end else if (current_temp <= TEMP_THROTTLE - 10'd100) begin
                        thermal_state <= THERM_WARNING;
                        thermal_throttle_active <= 1'b0;
                    end else if (current_temp >= TEMP_THROTTLE + 10'd50) begin
                        // More aggressive throttling
                        thermal_throttle_level <= 3'd3;
                    end
                end

                THERM_SHUTDOWN: begin
                    thermal_shutdown <= 1'b1;
                    // Disable all clock domains except always-on
                    clk_gate_en <= 8'b1100_0000;
                    // Power off non-essential domains
                    pd_enable <= {NUM_POWER_DOMAINS{1'b0}};
                    pd_enable[PD_ALWAYS_ON] <= 1'b1;

                    if (current_temp < TEMP_SHUTDOWN - 10'd200) begin
                        // Safe to restart
                        thermal_state <= THERM_NOMINAL;
                        thermal_shutdown <= 1'b0;
                    end
                end

                default: thermal_state <= THERM_NOMINAL;
            endcase
        end
    end

endmodule
