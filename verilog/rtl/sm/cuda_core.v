//------------------------------------------------------------------------------
// Module: cuda_core
// Description: Single-precision FP32 MAC unit with 4-stage pipeline
//              Supports ADD, MUL, FMA operations per IEEE 754
//              Latency: 3 cycles for FMA (execute + 2 pipeline regs)
//------------------------------------------------------------------------------

module cuda_core #(
    parameter DATA_WIDTH = 32
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // Control interface
    input  wire                      valid_in,
    output wire                      ready_out,
    input  wire [1:0]                op_code,      // 00=ADD, 01=MUL, 10=FMA, 11=reserved
    input  wire [DATA_WIDTH-1:0]     operand_a,    // Source A
    input  wire [DATA_WIDTH-1:0]     operand_b,    // Source B
    input  wire [DATA_WIDTH-1:0]     operand_c,    // Source C (for FMA)

    // Output interface
    output reg  [DATA_WIDTH-1:0]     result_out,
    output reg                       valid_out,
    output reg                       busy_out,

    // Exception flags
    output reg                       overflow_flag,
    output reg                       underflow_flag,
    output reg                       invalid_flag
);

    //--------------------------------------------------------------------------
    // Parameter definitions
    //--------------------------------------------------------------------------
    localparam OP_ADD = 2'b00;
    localparam OP_MUL = 2'b01;
    localparam OP_FMA = 2'b10;

    //--------------------------------------------------------------------------
    // Internal signals for pipeline stages
    //--------------------------------------------------------------------------
    // Stage 1: Decode / Input register
    reg [DATA_WIDTH-1:0] s1_op_a, s1_op_b, s1_op_c;
    reg [1:0]            s1_opcode;
    reg                  s1_valid;

    // Stage 2: Execute - unpack and operate
    reg [DATA_WIDTH-1:0] s2_result;
    reg                  s2_valid;
    reg                  s2_overflow;
    reg                  s2_underflow;
    reg                  s2_invalid;

    // Stage 3: Writeback pipeline register
    reg [DATA_WIDTH-1:0] s3_result;
    reg                  s3_valid;
    reg                  s3_overflow;
    reg                  s3_underflow;
    reg                  s3_invalid;

    // IEEE 754 field extraction
    wire        sign_a = operand_a[31];
    wire        sign_b = operand_b[31];
    wire        sign_c = operand_c[31];
    wire [7:0]  exp_a  = operand_a[30:23];
    wire [7:0]  exp_b  = operand_b[30:23];
    wire [7:0]  exp_c  = operand_c[30:23];
    wire [22:0] mant_a = operand_a[22:0];
    wire [22:0] mant_b = operand_b[22:0];
    wire [22:0] mant_c = operand_c[22:0];

    // Special value detection
    wire a_is_zero = (exp_a == 8'd0) && (mant_a == 23'd0);
    wire b_is_zero = (exp_b == 8'd0) && (mant_b == 23'd0);
    wire c_is_zero = (exp_c == 8'd0) && (mant_c == 23'd0);
    wire a_is_inf  = (exp_a == 8'hFF) && (mant_a == 23'd0);
    wire b_is_inf  = (exp_b == 8'hFF) && (mant_b == 23'd0);
    wire c_is_inf  = (exp_c == 8'hFF) && (mant_c == 23'd0);
    wire a_is_nan  = (exp_a == 8'hFF) && (mant_a != 23'd0);
    wire b_is_nan  = (exp_b == 8'hFF) && (mant_b != 23'd0);
    wire c_is_nan  = (exp_c == 8'hFF) && (mant_c != 23'd0);

    // Ready signal - core is ready when not busy
    assign ready_out = ~busy_out;

    //--------------------------------------------------------------------------
    // Stage 1: Fetch / Input Latching
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid   <= 1'b0;
            s1_op_a    <= {DATA_WIDTH{1'b0}};
            s1_op_b    <= {DATA_WIDTH{1'b0}};
            s1_op_c    <= {DATA_WIDTH{1'b0}};
            s1_opcode  <= 2'b00;
        end else begin
            if (valid_in && ready_out) begin
                s1_valid  <= 1'b1;
                s1_op_a   <= operand_a;
                s1_op_b   <= operand_b;
                s1_op_c   <= operand_c;
                s1_opcode <= op_code;
            end else begin
                s1_valid <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Stage 2: Execute - Combinational FP operations
    // Simplified FP32 implementation using integer arithmetic
    //--------------------------------------------------------------------------
    // Internal execute signals
    reg [DATA_WIDTH-1:0] exec_result;
    reg                  exec_overflow;
    reg                  exec_underflow;
    reg                  exec_invalid;

    // For multiplication: compute sign, exponent, mantissa
    wire        mul_sign = s1_op_a[31] ^ s1_op_b[31];
    wire [8:0]  mul_exp_tmp = {1'b0, s1_op_a[30:23]} + {1'b0, s1_op_b[30:23]} - 9'd127;
    wire [47:0] mul_mant = ({1'b1, s1_op_a[22:0]} * {1'b1, s1_op_b[22:0]});
    wire        mul_mant_overflow = mul_mant[47];
    wire [22:0] mul_mant_norm = mul_mant_overflow ? mul_mant[46:24] : mul_mant[45:23];
    wire [7:0]  mul_exp_norm = mul_mant_overflow ? (mul_exp_tmp[7:0] + 8'd1) : mul_exp_tmp[7:0];

    // For addition: align mantissas
    wire [7:0]  add_exp_max;
    wire [7:0]  add_exp_diff;
    wire [23:0] add_mant_a, add_mant_b;
    wire [24:0] add_mant_sum;
    wire        add_sign;
    wire [7:0]  add_exp_out;
    wire [22:0] add_mant_out;

    reg [7:0]  add_exp_a, add_exp_b;
    reg [23:0] add_mant_a_reg, add_mant_b_reg;
    reg        add_sign_a, add_sign_b;

    // FMA: A*B + C
    wire [31:0] fma_mul_result = {mul_sign, mul_exp_norm, mul_mant_norm};

    always @(*) begin
        // Default addition setup
        if (s1_op_a[30:23] >= s1_op_b[30:23]) begin
            add_exp_a = s1_op_a[30:23];
            add_exp_b = s1_op_b[30:23];
            add_mant_a_reg = {1'b1, s1_op_a[22:0]};
            add_mant_b_reg = {1'b1, s1_op_b[22:0]} >> (s1_op_a[30:23] - s1_op_b[30:23]);
            add_sign_a = s1_op_a[31];
            add_sign_b = s1_op_b[31];
        end else begin
            add_exp_a = s1_op_b[30:23];
            add_exp_b = s1_op_a[30:23];
            add_mant_a_reg = {1'b1, s1_op_b[22:0]};
            add_mant_b_reg = {1'b1, s1_op_a[22:0]} >> (s1_op_b[30:23] - s1_op_a[30:23]);
            add_sign_a = s1_op_b[31];
            add_sign_b = s1_op_a[31];
        end
    end

    assign add_exp_max = add_exp_a;
    assign add_mant_sum = (add_sign_a == add_sign_b) ? 
                          (add_mant_a_reg + add_mant_b_reg) :
                          (add_mant_a_reg - add_mant_b_reg);
    assign add_sign = (add_mant_sum[24]) ? add_sign_a :
                      ((add_mant_a_reg >= add_mant_b_reg) ? add_sign_a : add_sign_b);
    assign add_exp_out = add_exp_max + add_mant_sum[24];
    assign add_mant_out = add_mant_sum[24] ? add_mant_sum[23:1] : add_mant_sum[22:0];

    // FMA addition: fma_mul_result + s1_op_c
    wire [7:0]  fma_exp_max;
    wire [23:0] fma_mant_mul, fma_mant_c;
    wire [24:0] fma_mant_sum;
    wire [7:0]  fma_exp_out;
    wire [22:0] fma_mant_out;
    wire        fma_sign_out;

    reg [7:0]  fma_exp_mul_reg, fma_exp_c_reg;
    reg [23:0] fma_mant_mul_reg, fma_mant_c_reg;
    reg        fma_sign_mul_reg, fma_sign_c_reg;

    always @(*) begin
        if (mul_exp_norm >= s1_op_c[30:23]) begin
            fma_exp_mul_reg = mul_exp_norm;
            fma_exp_c_reg   = s1_op_c[30:23];
            fma_mant_mul_reg = {1'b1, mul_mant_norm};
            fma_mant_c_reg   = {1'b1, s1_op_c[22:0]} >> (mul_exp_norm - s1_op_c[30:23]);
            fma_sign_mul_reg = mul_sign;
            fma_sign_c_reg   = s1_op_c[31];
        end else begin
            fma_exp_mul_reg = s1_op_c[30:23];
            fma_exp_c_reg   = mul_exp_norm;
            fma_mant_mul_reg = {1'b1, mul_mant_norm} >> (s1_op_c[30:23] - mul_exp_norm);
            fma_mant_c_reg   = {1'b1, s1_op_c[22:0]};
            fma_sign_mul_reg = s1_op_c[31];
            fma_sign_c_reg   = mul_sign;
        end
    end

    assign fma_exp_max = fma_exp_mul_reg;
    assign fma_mant_sum = (fma_sign_mul_reg == fma_sign_c_reg) ?
                          (fma_mant_mul_reg + fma_mant_c_reg) :
                          (fma_mant_mul_reg - fma_mant_c_reg);
    assign fma_sign_out = (fma_mant_sum[24]) ? fma_sign_mul_reg :
                          ((fma_mant_mul_reg >= fma_mant_c_reg) ? fma_sign_mul_reg : fma_sign_c_reg);
    assign fma_exp_out = fma_exp_max + fma_mant_sum[24];
    assign fma_mant_out = fma_mant_sum[24] ? fma_mant_sum[23:1] : fma_mant_sum[22:0];

    //--------------------------------------------------------------------------
    // Execute stage combinational logic
    //--------------------------------------------------------------------------
    always @(*) begin
        exec_result    = {DATA_WIDTH{1'b0}};
        exec_overflow  = 1'b0;
        exec_underflow = 1'b0;
        exec_invalid   = 1'b0;

        if (s1_valid) begin
            case (s1_opcode)
                OP_ADD: begin
                    // Check for special values
                    if ((s1_op_a[30:23] == 8'hFF && s1_op_a[22:0] != 0) ||
                        (s1_op_b[30:23] == 8'hFF && s1_op_b[22:0] != 0)) begin
                        exec_result = 32'h7FC00000; // NaN
                        exec_invalid = 1'b1;
                    end else if (s1_op_a[30:23] == 8'hFF && s1_op_b[30:23] == 8'hFF &&
                                 s1_op_a[31] != s1_op_b[31]) begin
                        exec_result = 32'h7FC00000; // Inf - Inf = NaN
                        exec_invalid = 1'b1;
                    end else if (s1_op_a[30:23] == 8'hFF) begin
                        exec_result = s1_op_a;
                    end else if (s1_op_b[30:23] == 8'hFF) begin
                        exec_result = s1_op_b;
                    end else if (s1_op_a[30:0] == 31'd0) begin
                        exec_result = s1_op_b;
                    end else if (s1_op_b[30:0] == 31'd0) begin
                        exec_result = s1_op_a;
                    end else begin
                        exec_result = {add_sign, add_exp_out, add_mant_out};
                        if (add_exp_out == 8'hFF) exec_overflow = 1'b1;
                        if (add_exp_out == 8'd0 && add_mant_out != 0) exec_underflow = 1'b1;
                    end
                end

                OP_MUL: begin
                    if ((s1_op_a[30:23] == 8'hFF && s1_op_a[22:0] != 0) ||
                        (s1_op_b[30:23] == 8'hFF && s1_op_b[22:0] != 0)) begin
                        exec_result = 32'h7FC00000; // NaN
                        exec_invalid = 1'b1;
                    end else if ((s1_op_a[30:23] == 8'hFF && s1_op_b[30:0] == 0) ||
                                (s1_op_b[30:23] == 8'hFF && s1_op_a[30:0] == 0)) begin
                        exec_result = 32'h7FC00000; // Inf * 0 = NaN
                        exec_invalid = 1'b1;
                    end else if (s1_op_a[30:23] == 8'hFF || s1_op_b[30:23] == 8'hFF) begin
                        exec_result = {mul_sign, 8'hFF, 23'd0}; // Inf
                    end else if (s1_op_a[30:0] == 0 || s1_op_b[30:0] == 0) begin
                        exec_result = {mul_sign, 31'd0}; // Zero
                    end else begin
                        exec_result = {mul_sign, mul_exp_norm, mul_mant_norm};
                        if (mul_exp_norm == 8'hFF) exec_overflow = 1'b1;
                        if (mul_exp_norm == 8'd0 && mul_mant_norm != 0) exec_underflow = 1'b1;
                    end
                end

                OP_FMA: begin
                    if ((s1_op_a[30:23] == 8'hFF && s1_op_a[22:0] != 0) ||
                        (s1_op_b[30:23] == 8'hFF && s1_op_b[22:0] != 0) ||
                        (s1_op_c[30:23] == 8'hFF && s1_op_c[22:0] != 0)) begin
                        exec_result = 32'h7FC00000; // NaN
                        exec_invalid = 1'b1;
                    end else if (mul_exp_tmp >= 9'd255 || fma_exp_out == 8'hFF) begin
                        exec_result = {fma_sign_out, 8'hFF, 23'd0}; // Inf
                        exec_overflow = 1'b1;
                    end else begin
                        exec_result = {fma_sign_out, fma_exp_out, fma_mant_out};
                        if (fma_exp_out == 8'd0 && fma_mant_out != 0) exec_underflow = 1'b1;
                    end
                end

                default: begin
                    exec_result = 32'h7FC00000; // NaN for undefined op
                    exec_invalid = 1'b1;
                end
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // Stage 2: Execute output register
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid     <= 1'b0;
            s2_result    <= {DATA_WIDTH{1'b0}};
            s2_overflow  <= 1'b0;
            s2_underflow <= 1'b0;
            s2_invalid   <= 1'b0;
        end else begin
            s2_valid     <= s1_valid;
            s2_result    <= exec_result;
            s2_overflow  <= exec_overflow;
            s2_underflow <= exec_underflow;
            s2_invalid   <= exec_invalid;
        end
    end

    //--------------------------------------------------------------------------
    // Stage 3: Writeback pipeline register
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_valid     <= 1'b0;
            s3_result    <= {DATA_WIDTH{1'b0}};
            s3_overflow  <= 1'b0;
            s3_underflow <= 1'b0;
            s3_invalid   <= 1'b0;
        end else begin
            s3_valid     <= s2_valid;
            s3_result    <= s2_result;
            s3_overflow  <= s2_overflow;
            s3_underflow <= s2_underflow;
            s3_invalid   <= s2_invalid;
        end
    end

    //--------------------------------------------------------------------------
    // Output assignments
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result_out     <= {DATA_WIDTH{1'b0}};
            valid_out      <= 1'b0;
            overflow_flag  <= 1'b0;
            underflow_flag <= 1'b0;
            invalid_flag   <= 1'b0;
            busy_out       <= 1'b0;
        end else begin
            result_out     <= s3_result;
            valid_out      <= s3_valid;
            overflow_flag  <= s3_overflow;
            underflow_flag <= s3_underflow;
            invalid_flag   <= s3_invalid;
            // Busy when any stage is valid or input is accepted
            busy_out <= valid_in | s1_valid | s2_valid | s3_valid;
        end
    end

endmodule
