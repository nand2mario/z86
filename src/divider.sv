// ============================================================================
// 32-bit ÷ 16-bit (or 16-bit ÷ 8-bit) divider with signed/unsigned support
// ----------------------------------------------------------------------------
//   width = 0 : 16-bit dividend / 8-bit divisor   (16 cycles)
//   width = 1 : 32-bit dividend / 16-bit divisor  (32 cycles)
//   start     : pulse to begin when busy = 0
//   busy      : high while engine is running
//   done      : one-cycle pulse when result is ready
//   dbz       : asserted with done if divisor = 0
// ============================================================================
module divider
(
    input  wire        clk,
    input  wire        rst_n,

    // control
    input  wire        start,
    input  wire        width,        // 0=16/8, 1=32/16
    input  wire        is_signed,    // 0=unsigned, 1=signed

    // operands
    input  wire [31:0] dividend,
    input  wire [15:0] divisor,

    // status
    output reg         busy,
    output reg         done,
    output reg         overflow,    // either divide by zero, or quotient cannot fit in 8/16 bits

    // results
    output reg [31:0]  quotient,    // lower 16 valid in 16/8 mode
    output reg [15:0]  remainder    // lower  8 valid in 16/8 mode
);

    // ---------------------------------------------------------------------
    // local parameters & helpers
    // ---------------------------------------------------------------------
    localparam IDLE = 1'b0, RUN = 1'b1;

    reg  state;
    reg  [5:0]  cnt;                // 1 … 32
    reg  is_signed_r;

    // remainder | quotient packed together  17 + 32 = 49 bits
    reg  [48:0] rq;

    // positive-magnitude divisor (17 bits)  {1’b0, |divisor|}
    reg  [16:0] D;

    // sign bookkeeping
    reg  q_neg, r_neg;

    // width-dependent views of operands
    wire sel32 = width;             // 1 → 32/16, 0 → 16/8
    reg sel32_r;

    wire [31:0] divd_eff = sel32 ? dividend
                                 : {dividend[15:0], 16'b0};
    wire [15:0] divs_eff = sel32 ? divisor :
                           is_signed ? { {8{divisor[7]}}, divisor[7:0]} : { 8'd0, divisor[7:0]};

    // sign bits seen by the selected width
    wire divd_neg = is_signed & (sel32 ? dividend[31] : dividend[15]);
    wire divs_neg = is_signed & (sel32 ? divisor [15] : divisor [7]);

    // absolute-value helpers
    function [31:0] abs32 (input [31:0] v);
        abs32 = v[31] ? (~v + 1'b1) : v;
    endfunction
    function [15:0] abs16 (input [15:0] v);
        abs16 = v[15] ? (~v + 1'b1) : v;
    endfunction

    // cycle count per mode
    wire [5:0] cycles = sel32 ? 6'd32 : 6'd16;

    // ---------------------------------------------------------------------
    // combinational “next-state” network for the RUN phase
    // ---------------------------------------------------------------------
    wire [48:0] rq_shift  = {rq[47:0], 1'b0};            // left shift
    wire [16:0] rem_trial = rq_shift[48:32];             // top 17 bits
    wire        ge        = (rem_trial >= D);            // trial ≥ divisor?

    // if (ge) subtract and set new quotient bit = 1
    wire [16:0] rem_next  = ge ? (rem_trial - D) : rem_trial;
    wire [31:0] quo_next  = {rq_shift[31:1], ge};        // replace LSB with ge
    wire [48:0] rq_next   = {rem_next, quo_next};

    // ---------------------------------------------------------------------
    // main FSM
    // ---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            overflow  <= 1'b0;
            quotient  <= 32'd0;
            remainder <= 16'd0;
        end
        else begin
            logic overflow_next;
            logic [31:0] quotient_next;
            // default pulse-clears
            done <= 1'b0;

            case (state)
            // -------------------------------------------------------------
            // IDLE – accept a new request
            // -------------------------------------------------------------
            IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    // divide-by-zero?
                    overflow_next <= (sel32 ? (divs_eff == 16'd0)
                                            : (divs_eff[7:0] == 8'd0));
                    overflow <= overflow_next;

                    if (overflow_next) begin
                        // finish immediately on DBZ
                        quotient  <= 32'd0;
                        remainder <= 16'd0;
                        done      <= 1'b1;
                    end else begin
                        // normal start
                        cnt   <= cycles;       // counts DOWN to 0
                        busy  <= 1'b1;
                        state <= RUN;

                        // take magnitudes
                        q_neg <= divd_neg ^ divs_neg;         // sign(quotient)
                        r_neg <= divd_neg;                    // sign(remainder)

                        rq    <= { 17'd0,                     // remainder=0
                                   is_signed ? abs32(divd_eff)
                                             : divd_eff };

                        D     <= {1'b0,
                                  is_signed ? abs16(divs_eff)
                                            : divs_eff };

                        is_signed_r <= is_signed;
                        sel32_r <= sel32;
                    end
                end
            end

            // -------------------------------------------------------------
            // RUN – one restore step per clock
            // -------------------------------------------------------------
            RUN: begin
                rq  <= rq_next;        // shift / subtract / set Q bit
                cnt <= cnt - 1'b1;

                // detect overflow by watching 8-th bit or 16-bit of quotient
                if (cnt <= 16 && (sel32_r & rq_next[16] | ~sel32_r & rq_next[8])) begin
                    overflow <= 1'b1;
                    busy <= 0;
                    done <= 1;
                    state <= IDLE;
                end else if (cnt == 6'd1) begin // this was the last iteration
                    // un-pack & sign-correct
                    quotient_next = q_neg ? (~rq_next[31:0] + 1'b1)
                                       :  rq_next[31:0];
                    quotient <= quotient_next;

                    remainder <= r_neg ? (~rq_next[47:32] + 1'b1)
                                       :  rq_next[47:32];

                    if (is_signed_r) begin
                        // overflow if quotient cannot fit into 8/16 signed number
                        if (sel32_r) begin
                            overflow <= rq_next[15] & (~q_neg | rq_next[15:0] != 16'h8000); // -32768 - 32767, otherwise overflow
                        end else begin
                            overflow <= rq_next[7] & (~q_neg | rq_next[7:0] != 8'h80);      // -128 - 127, otherwise overflow
                        end
                    end

                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= IDLE;
                end
            end
            endcase
        end
    end
endmodule
