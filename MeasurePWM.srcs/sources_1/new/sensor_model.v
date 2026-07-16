//----------------------------------------------------------------------
// sensor_model.v -- Optional AHRS corruption (rov_sensor.py) in front of um7_tx
//
// PASSTHROUGH=1: 1-cycle register copy of true plant state (clean HIL).
// PASSTHROUGH=0: full rov_sensor.py chain using constants in sensor_params.vh.
//
// TIMING (PASSTHROUGH=0)
// ----------------------
// All Q16.16 multiplies go through ONE shared pipelined unit (q16_mul.v).
// Multiplies are issued serially (~39 muls × 4 cycles ≈ 160 clocks ≪ 20 ms).
// No parallel 32x32 function-fmuls, no multicycle XDC required for u_sensor.
// Previous parallel fmul() design caused WNS≈-3.5 ns and HIL RPY spikes.
//
// Quaternion is NOT generated here: um7_tx builds it from measured RPY.
//----------------------------------------------------------------------
`include "sensor_params.vh"

module sensor_model #(
    parameter PASSTHROUGH = 1
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    output reg         done,

    input  wire signed [31:0] roll_in,
    input  wire signed [31:0] pitch_in,
    input  wire signed [31:0] yaw_in,
    input  wire signed [31:0] p_in,
    input  wire signed [31:0] q_in,
    input  wire signed [31:0] r_in,

    output reg  signed [31:0] roll_out,
    output reg  signed [31:0] pitch_out,
    output reg  signed [31:0] yaw_out,
    output reg  signed [31:0] p_out,
    output reg  signed [31:0] q_out,
    output reg  signed [31:0] r_out
);

    //==================================================================
    // PASSTHROUGH path — 1 cycle, bit-exact copy
    //==================================================================
    generate if (PASSTHROUGH != 0) begin : g_pass
        always @(posedge clk) begin
            if (rst) begin
                done       <= 1'b0;
                roll_out   <= 32'sd0;
                pitch_out  <= 32'sd0;
                yaw_out    <= 32'sd0;
                p_out      <= 32'sd0;
                q_out      <= 32'sd0;
                r_out      <= 32'sd0;
            end else begin
                done <= 1'b0;
                if (start) begin
                    roll_out  <= roll_in;
                    pitch_out <= pitch_in;
                    yaw_out   <= yaw_in;
                    p_out     <= p_in;
                    q_out     <= q_in;
                    r_out     <= r_in;
                    done      <= 1'b1;
                end
            end
        end
    end else begin : g_noise
    //==================================================================
    // Full noise model — one shared q16_mul, serial issue
    //==================================================================
        function signed [31:0] fclamp;
            input signed [31:0] v, lo, hi;
            begin
                if      (v < lo) fclamp = lo;
                else if (v > hi) fclamp = hi;
                else             fclamp = v;
            end
        endfunction

        function signed [31:0] fabs;
            input signed [31:0] v;
            begin fabs = v[31] ? -v : v; end
        endfunction

        function [31:0] xorshift_next;
            input [31:0] x;
            reg [31:0] y;
            begin
                y = x ^ (x << 13);
                y = y ^ (y >> 17);
                y = y ^ (y << 5);
                if (y == 32'd0) y = 32'h1;
                xorshift_next = y;
            end
        endfunction

        // Q16.16 * 0.25 with same round as fmul(x, 16384)
        function signed [31:0] q16_div4;
            input signed [31:0] x;
            reg signed [63:0] t;
            begin
                t = (x <<< 14) + 64'sd32768;
                q16_div4 = t[47:16];
            end
        endfunction

        // q * 11 without DSP
        function signed [31:0] mul11;
            input signed [31:0] q;
            begin mul11 = (q <<< 3) + (q <<< 1) + q; end
        endfunction

        // Precomputed amplitudes a = std * sqrt(3)
        localparam signed [31:0] AMP_RATE_NOISE  = 32'sd5;
        localparam signed [31:0] AMP_BIAS_DRIFT  = 32'sd2;
        localparam signed [31:0] AMP_ATT_ST_R    = 32'sd1982;
        localparam signed [31:0] AMP_ATT_ST_P    = 32'sd1982;
        localparam signed [31:0] AMP_ATT_ST_Y    = 32'sd5940;
        localparam signed [31:0] AMP_ATT_DY_R    = 32'sd5940;
        localparam signed [31:0] AMP_ATT_DY_P    = 32'sd5940;
        localparam signed [31:0] AMP_ATT_DY_Y    = 32'sd9901;
        localparam signed [31:0] INV_ATT_QUANT   = 32'sd5958;
        localparam signed [31:0] INV_FS          = 32'sd1877;

        // ---- shared multiplier ----
        reg  signed [31:0] mul_a, mul_b;
        reg                mul_start;
        wire signed [31:0] mul_p;
        wire               mul_valid;
        wire               mul_busy;

        q16_mul u_mul (
            .clk  (clk),
            .rst  (rst),
            .start(mul_start),
            .a    (mul_a),
            .b    (mul_b),
            .p    (mul_p),
            .valid(mul_valid),
            .busy (mul_busy)
        );

        reg [31:0] lfsr;
        reg signed [31:0] rate_bias_p, rate_bias_q, rate_bias_r;
        reg signed [31:0] att_nbuf_r [0:3];
        reg signed [31:0] att_nbuf_p [0:3];
        reg signed [31:0] att_nbuf_y [0:3];
        reg [1:0] att_nidx;

        reg signed [31:0] att_r, att_p, att_y;
        reg signed [31:0] rt_p, rt_q, rt_r;
        reg signed [31:0] t0, t1, t2;
        reg signed [31:0] n_p, n_q, n_r;
        reg signed [31:0] n_ar, n_ap, n_ay;
        reg signed [31:0] amp_r, amp_p, amp_y;
        reg signed [31:0] sum_r, sum_p, sum_y;
        reg signed [31:0] noise_tmp;
        reg               issued;

        // State encoding: each mul is one state that issues then waits for valid
        localparam [5:0]
            ST_IDLE    = 6'd0,
            // cross-axis row 0
            ST_CA00    = 6'd1,
            ST_CA01    = 6'd2,
            ST_CA02    = 6'd3,
            ST_CA0S    = 6'd4,
            // row 1
            ST_CA10    = 6'd5,
            ST_CA11    = 6'd6,
            ST_CA12    = 6'd7,
            ST_CA1S    = 6'd8,
            // row 2
            ST_CA20    = 6'd9,
            ST_CA21    = 6'd10,
            ST_CA22    = 6'd11,
            ST_CA2S    = 6'd12,
            // scale
            ST_SFP     = 6'd13,
            ST_SFQ     = 6'd14,
            ST_SFR     = 6'd15,
            // nonlinearity
            ST_NL0P    = 6'd16,
            ST_NL0Q    = 6'd17,
            ST_NL0R    = 6'd18,
            ST_NL1P    = 6'd19,
            ST_NL1Q    = 6'd20,
            ST_NL1R    = 6'd21,
            ST_NL2P    = 6'd22,
            ST_NL2Q    = 6'd23,
            ST_NL2R    = 6'd24,
            ST_NL3P    = 6'd25,
            ST_NL3Q    = 6'd26,
            ST_NL3R    = 6'd27,
            ST_NL4P    = 6'd28,
            ST_NL4Q    = 6'd29,
            ST_NL4R    = 6'd30,
            // bias noise
            ST_BP      = 6'd31,
            ST_BPA     = 6'd32,
            ST_BQ      = 6'd33,
            ST_BQA     = 6'd34,
            ST_BR      = 6'd35,
            ST_BRA     = 6'd36,
            // rate noise
            ST_RNP     = 6'd37,
            ST_RNPA    = 6'd38,
            ST_RNQ     = 6'd39,
            ST_RNQA    = 6'd40,
            ST_RNR     = 6'd41,
            ST_RNRA    = 6'd42,
            ST_RSAT    = 6'd43,
            ST_ABIAS   = 6'd44,
            ST_ASTD    = 6'd45,
            // attitude noise
            ST_ANR     = 6'd46,
            ST_ANRA    = 6'd47,
            ST_ANP     = 6'd48,
            ST_ANPA    = 6'd49,
            ST_ANY     = 6'd50,
            ST_ANYA    = 6'd51,
            ST_AMA     = 6'd52,
            ST_AADD    = 6'd53,
            // att quant
            ST_AQR     = 6'd54,
            ST_AQP     = 6'd55,
            ST_AQY     = 6'd56,
            ST_AQOUT   = 6'd57,
            ST_DONE    = 6'd58;

        reg [5:0] st;
        integer i;

        // Helper: build Q16.16 U[-1,1] sample from current lfsr low bits
        function signed [31:0] lfsr_u;
            input [31:0] l;
            reg signed [31:0] u;
            begin
                u = $signed({1'b0, l[15:0]}) - 32'sd32768;
                lfsr_u = u <<< 1;
            end
        endfunction

        always @(posedge clk) begin
            if (rst) begin
                st          <= ST_IDLE;
                done        <= 1'b0;
                mul_start   <= 1'b0;
                mul_a       <= 32'sd0;
                mul_b       <= 32'sd0;
                issued      <= 1'b0;
                lfsr        <= `SP_LFSR_SEED;
                rate_bias_p <= `SP_RATE_BIAS_P;
                rate_bias_q <= `SP_RATE_BIAS_Q;
                rate_bias_r <= `SP_RATE_BIAS_R;
                att_nidx    <= 2'd0;
                for (i = 0; i < 4; i = i + 1) begin
                    att_nbuf_r[i] <= 32'sd0;
                    att_nbuf_p[i] <= 32'sd0;
                    att_nbuf_y[i] <= 32'sd0;
                end
                roll_out  <= 32'sd0; pitch_out <= 32'sd0; yaw_out <= 32'sd0;
                p_out <= 32'sd0; q_out <= 32'sd0; r_out <= 32'sd0;
                att_r <= 0; att_p <= 0; att_y <= 0;
                rt_p <= 0; rt_q <= 0; rt_r <= 0;
                t0 <= 0; t1 <= 0; t2 <= 0;
                n_p <= 0; n_q <= 0; n_r <= 0;
                n_ar <= 0; n_ap <= 0; n_ay <= 0;
                amp_r <= 0; amp_p <= 0; amp_y <= 0;
                sum_r <= 0; sum_p <= 0; sum_y <= 0;
                noise_tmp <= 0;
            end else begin
                done      <= 1'b0;
                mul_start <= 1'b0;

                case (st)
                    ST_IDLE: if (start) begin
                        att_r  <= roll_in;  att_p  <= pitch_in; att_y <= yaw_in;
                        rt_p   <= p_in;     rt_q   <= q_in;     rt_r  <= r_in;
                        issued <= 1'b0;
                        st     <= ST_CA00;
                    end

                    //--------------------------------------------------
                    // Macro: issue mul once, on valid capture and advance
                    // Cross-axis row 0
                    ST_CA00: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_00; mul_b <= rt_p;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_CA01;
                        end
                    end
                    ST_CA01: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_01; mul_b <= rt_q;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_CA02;
                        end
                    end
                    ST_CA02: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_02; mul_b <= rt_r;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_CA0S;
                        end
                    end
                    ST_CA0S: begin n_p <= t0 + t1 + t2; st <= ST_CA10; end

                    ST_CA10: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_10; mul_b <= rt_p;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_CA11;
                        end
                    end
                    ST_CA11: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_11; mul_b <= rt_q;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_CA12;
                        end
                    end
                    ST_CA12: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_12; mul_b <= rt_r;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_CA1S;
                        end
                    end
                    ST_CA1S: begin n_q <= t0 + t1 + t2; st <= ST_CA20; end

                    ST_CA20: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_20; mul_b <= rt_p;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_CA21;
                        end
                    end
                    ST_CA21: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_21; mul_b <= rt_q;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_CA22;
                        end
                    end
                    ST_CA22: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= `SP_CA_22; mul_b <= rt_r;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_CA2S;
                        end
                    end
                    ST_CA2S: begin
                        rt_p <= n_p; rt_q <= n_q; rt_r <= t0 + t1 + t2;
                        st <= ST_SFP;
                    end

                    // Scale factors
                    ST_SFP: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= rt_p; mul_b <= `SP_RATE_SF_P;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            rt_p <= mul_p; issued <= 1'b0; st <= ST_SFQ;
                        end
                    end
                    ST_SFQ: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= rt_q; mul_b <= `SP_RATE_SF_Q;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            rt_q <= mul_p; issued <= 1'b0; st <= ST_SFR;
                        end
                    end
                    ST_SFR: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= rt_r; mul_b <= `SP_RATE_SF_R;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            rt_r <= mul_p; issued <= 1'b0; st <= ST_NL0P;
                        end
                    end

                    // Nonlinearity: nl = nl_fs * rate^3 / FS^2
                    ST_NL0P: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= rt_p; mul_b <= rt_p;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_NL0Q;
                        end
                    end
                    ST_NL0Q: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= rt_q; mul_b <= rt_q;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_NL0R;
                        end
                    end
                    ST_NL0R: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= rt_r; mul_b <= rt_r;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_NL1P;
                        end
                    end
                    ST_NL1P: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t0; mul_b <= rt_p;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_NL1Q;
                        end
                    end
                    ST_NL1Q: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t1; mul_b <= rt_q;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_NL1R;
                        end
                    end
                    ST_NL1R: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t2; mul_b <= rt_r;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_NL2P;
                        end
                    end
                    ST_NL2P: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t0; mul_b <= `SP_NL_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_NL2Q;
                        end
                    end
                    ST_NL2Q: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t1; mul_b <= `SP_NL_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_NL2R;
                        end
                    end
                    ST_NL2R: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t2; mul_b <= `SP_NL_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_NL3P;
                        end
                    end
                    ST_NL3P: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t0; mul_b <= INV_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_NL3Q;
                        end
                    end
                    ST_NL3Q: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t1; mul_b <= INV_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_NL3R;
                        end
                    end
                    ST_NL3R: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t2; mul_b <= INV_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_NL4P;
                        end
                    end
                    ST_NL4P: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t0; mul_b <= INV_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            n_p <= mul_p; issued <= 1'b0; st <= ST_NL4Q;
                        end
                    end
                    ST_NL4Q: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t1; mul_b <= INV_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            n_q <= mul_p; issued <= 1'b0; st <= ST_NL4R;
                        end
                    end
                    ST_NL4R: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= t2; mul_b <= INV_FS;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            n_r <= mul_p; issued <= 1'b0; st <= ST_BP;
                        end
                    end

                    // Bias walk + apply
                    ST_BP: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= AMP_BIAS_DRIFT;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            noise_tmp <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_BPA;
                        end
                    end
                    ST_BPA: begin
                        rate_bias_p <= rate_bias_p + noise_tmp;
                        rt_p <= rt_p + n_p + (rate_bias_p + noise_tmp) + `SP_BIAS_TEMP_P;
                        st <= ST_BQ;
                    end
                    ST_BQ: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= AMP_BIAS_DRIFT;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            noise_tmp <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_BQA;
                        end
                    end
                    ST_BQA: begin
                        rate_bias_q <= rate_bias_q + noise_tmp;
                        rt_q <= rt_q + n_q + (rate_bias_q + noise_tmp) + `SP_BIAS_TEMP_Q;
                        st <= ST_BR;
                    end
                    ST_BR: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= AMP_BIAS_DRIFT;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            noise_tmp <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_BRA;
                        end
                    end
                    ST_BRA: begin
                        rate_bias_r <= rate_bias_r + noise_tmp;
                        rt_r <= rt_r + n_r + (rate_bias_r + noise_tmp) + `SP_BIAS_TEMP_R;
                        st <= ST_RNP;
                    end

                    // Rate white noise
                    ST_RNP: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= AMP_RATE_NOISE;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            noise_tmp <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_RNPA;
                        end
                    end
                    ST_RNPA: begin rt_p <= rt_p + noise_tmp; st <= ST_RNQ; end
                    ST_RNQ: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= AMP_RATE_NOISE;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            noise_tmp <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_RNQA;
                        end
                    end
                    ST_RNQA: begin rt_q <= rt_q + noise_tmp; st <= ST_RNR; end
                    ST_RNR: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= AMP_RATE_NOISE;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            noise_tmp <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_RNRA;
                        end
                    end
                    ST_RNRA: begin rt_r <= rt_r + noise_tmp; st <= ST_RSAT; end

                    // Saturation (quant step=1 → identity)
                    ST_RSAT: begin
                        rt_p <= fclamp(rt_p, -`SP_RATE_SAT, `SP_RATE_SAT);
                        rt_q <= fclamp(rt_q, -`SP_RATE_SAT, `SP_RATE_SAT);
                        rt_r <= fclamp(rt_r, -`SP_RATE_SAT, `SP_RATE_SAT);
                        st   <= ST_ABIAS;
                    end

                    ST_ABIAS: begin
                        att_r <= att_r + `SP_ATT_BIAS_R;
                        att_p <= att_p + `SP_ATT_BIAS_P;
                        att_y <= att_y + `SP_ATT_BIAS_Y;
                        st    <= ST_ASTD;
                    end

                    ST_ASTD: begin
                        amp_r <= (fabs(rt_p) > `SP_DYN_RATE_THR) ? AMP_ATT_DY_R : AMP_ATT_ST_R;
                        amp_p <= (fabs(rt_q) > `SP_DYN_RATE_THR) ? AMP_ATT_DY_P : AMP_ATT_ST_P;
                        amp_y <= (fabs(rt_r) > `SP_DYN_RATE_THR) ? AMP_ATT_DY_Y : AMP_ATT_ST_Y;
                        st    <= ST_ANR;
                    end

                    // Attitude noise samples
                    ST_ANR: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= amp_r;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            n_ar <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_ANP;
                        end
                    end
                    ST_ANP: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= amp_p;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            n_ap <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_ANY;
                        end
                    end
                    ST_ANY: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= lfsr_u(lfsr); mul_b <= amp_y;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            n_ay <= mul_p;
                            lfsr <= xorshift_next(lfsr);
                            issued <= 1'b0; st <= ST_AMA;
                        end
                    end

                    // Moving average of 4
                    ST_AMA: begin
                        att_nbuf_r[att_nidx] <= n_ar;
                        att_nbuf_p[att_nidx] <= n_ap;
                        att_nbuf_y[att_nidx] <= n_ay;
                        case (att_nidx)
                            2'd0: begin
                                sum_r <= n_ar + att_nbuf_r[1] + att_nbuf_r[2] + att_nbuf_r[3];
                                sum_p <= n_ap + att_nbuf_p[1] + att_nbuf_p[2] + att_nbuf_p[3];
                                sum_y <= n_ay + att_nbuf_y[1] + att_nbuf_y[2] + att_nbuf_y[3];
                            end
                            2'd1: begin
                                sum_r <= att_nbuf_r[0] + n_ar + att_nbuf_r[2] + att_nbuf_r[3];
                                sum_p <= att_nbuf_p[0] + n_ap + att_nbuf_p[2] + att_nbuf_p[3];
                                sum_y <= att_nbuf_y[0] + n_ay + att_nbuf_y[2] + att_nbuf_y[3];
                            end
                            2'd2: begin
                                sum_r <= att_nbuf_r[0] + att_nbuf_r[1] + n_ar + att_nbuf_r[3];
                                sum_p <= att_nbuf_p[0] + att_nbuf_p[1] + n_ap + att_nbuf_p[3];
                                sum_y <= att_nbuf_y[0] + att_nbuf_y[1] + n_ay + att_nbuf_y[3];
                            end
                            default: begin
                                sum_r <= att_nbuf_r[0] + att_nbuf_r[1] + att_nbuf_r[2] + n_ar;
                                sum_p <= att_nbuf_p[0] + att_nbuf_p[1] + att_nbuf_p[2] + n_ap;
                                sum_y <= att_nbuf_y[0] + att_nbuf_y[1] + att_nbuf_y[2] + n_ay;
                            end
                        endcase
                        att_nidx <= att_nidx + 2'd1;
                        st <= ST_AADD;
                    end

                    ST_AADD: begin
                        att_r <= att_r + q16_div4(sum_r);
                        att_p <= att_p + q16_div4(sum_p);
                        att_y <= att_y + q16_div4(sum_y);
                        st    <= ST_AQR;
                    end

                    // Attitude quant: q = round(att * inv_step) via q16_mul, then *11
                    // q16_mul always uses +32768 round (matches fmul); good enough for 0.01°
                    ST_AQR: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= att_r; mul_b <= INV_ATT_QUANT;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t0 <= mul_p; issued <= 1'b0; st <= ST_AQP;
                        end
                    end
                    ST_AQP: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= att_p; mul_b <= INV_ATT_QUANT;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t1 <= mul_p; issued <= 1'b0; st <= ST_AQY;
                        end
                    end
                    ST_AQY: begin
                        if (!issued) begin
                            if (!mul_busy) begin
                                mul_a <= att_y; mul_b <= INV_ATT_QUANT;
                                mul_start <= 1'b1; issued <= 1'b1;
                            end
                        end else if (mul_valid) begin
                            t2 <= mul_p; issued <= 1'b0; st <= ST_AQOUT;
                        end
                    end
                    ST_AQOUT: begin
                        roll_out  <= mul11(t0);
                        pitch_out <= mul11(t1);
                        yaw_out   <= mul11(t2);
                        p_out     <= rt_p;
                        q_out     <= rt_q;
                        r_out     <= rt_r;
                        st        <= ST_DONE;
                    end

                    ST_DONE: begin
                        done <= 1'b1;
                        st   <= ST_IDLE;
                    end

                    default: st <= ST_IDLE;
                endcase
            end
        end
    end endgenerate

endmodule
