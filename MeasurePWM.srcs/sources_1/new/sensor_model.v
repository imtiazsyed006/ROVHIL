//----------------------------------------------------------------------
// sensor_model.v -- Optional AHRS corruption (rov_sensor.py) in front of um7_tx
//
// PASSTHROUGH=1 (DEFAULT): 1-cycle register copy of true plant state.
//   Use this for HIL so a grounded ROV at plant zero reports RPY=0 and
//   identity quaternion.  The previous always-on noise model made rest
//   look like several degrees of random attitude and occasional spikes.
//
// PASSTHROUGH=0: full rov_sensor.py chain (cross-axis, scale, bias walk,
//   white noise, sat, quant) using constants in sensor_params.vh.
//
// Quaternion is NOT generated here: um7_tx builds it from measured RPY.
//----------------------------------------------------------------------
`include "sensor_params.vh"

module sensor_model #(
    // 1 = clean passthrough (recommended for HIL / grounded start)
    // 0 = inject AHRS noise/bias per sensor_params.vh
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
    // Full noise model (only elaborated when PASSTHROUGH=0)
    //==================================================================
        function signed [31:0] fmul;
            input signed [31:0] a, b;
            reg signed [63:0] p;
            begin
                p    = a * b + 64'sd32768;
                fmul = p[47:16];
            end
        endfunction

        function signed [31:0] fclamp;
            input signed [31:0] v, lo, hi;
            begin
                if      (v < lo) fclamp = lo;
                else if (v > hi) fclamp = hi;
                else             fclamp = v;
            end
        endfunction

        function signed [31:0] fquant;
            input signed [31:0] v, step;
            reg signed [31:0] half, q;
            begin
                if (step <= 0) fquant = v;
                else begin
                    half = step >>> 1;
                    if (v >= 0) q = (v + half) / step;
                    else        q = (v - half) / step;
                    fquant = q * step;
                end
            end
        endfunction

        function signed [31:0] fabs;
            input signed [31:0] v;
            begin fabs = v[31] ? -v : v; end
        endfunction

        reg [31:0] lfsr;
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

        // Unbiased approx N(0,std): U[-a,a] with a = std*sqrt(3)
        function signed [31:0] noise_approx;
            input signed [31:0] std;
            reg signed [31:0] u, a;
            begin
                u = $signed({1'b0, lfsr[15:0]}) - 32'sd32768;
                u = u <<< 1;
                a = fmul(std, `SP_SQRT3);
                noise_approx = fmul(u, a);
            end
        endfunction

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
        reg signed [31:0] std_r, std_p, std_y;
        reg signed [31:0] sum_r, sum_p, sum_y;
        reg signed [31:0] inv_fs;
        reg signed [31:0] bp, bq, br;

        localparam [4:0]
            ST_IDLE=0, ST_CA0=1, ST_CA0A=2, ST_CA1=3, ST_CA1A=4,
            ST_CA2=5, ST_CA2A=6, ST_SF=7, ST_NL0=8, ST_NL1=9,
            ST_NL2=10, ST_NL3=19, ST_NL4=20, ST_BIAS=11, ST_RNOISE=12,
            ST_RSAT=13, ST_ABIAS=14, ST_ASTD=15, ST_ANOISE=16,
            ST_AQUANT=17, ST_DONE=18;

        reg [4:0] st;
        integer i;

        always @(posedge clk) begin
            if (rst) begin
                st <= ST_IDLE; done <= 0;
                lfsr <= `SP_LFSR_SEED;
                rate_bias_p <= `SP_RATE_BIAS_P;
                rate_bias_q <= `SP_RATE_BIAS_Q;
                rate_bias_r <= `SP_RATE_BIAS_R;
                att_nidx <= 0;
                for (i = 0; i < 4; i = i + 1) begin
                    att_nbuf_r[i] <= 0; att_nbuf_p[i] <= 0; att_nbuf_y[i] <= 0;
                end
                roll_out<=0; pitch_out<=0; yaw_out<=0;
                p_out<=0; q_out<=0; r_out<=0;
                inv_fs <= 32'sd1877;
            end else begin
                done <= 0;
                case (st)
                    ST_IDLE: if (start) begin
                        att_r<=roll_in; att_p<=pitch_in; att_y<=yaw_in;
                        rt_p<=p_in; rt_q<=q_in; rt_r<=r_in;
                        st<=ST_CA0;
                    end
                    ST_CA0: begin
                        t0<=fmul(`SP_CA_00,rt_p); t1<=fmul(`SP_CA_01,rt_q); t2<=fmul(`SP_CA_02,rt_r);
                        st<=ST_CA0A;
                    end
                    ST_CA0A: begin n_p<=t0+t1+t2; st<=ST_CA1; end
                    ST_CA1: begin
                        t0<=fmul(`SP_CA_10,rt_p); t1<=fmul(`SP_CA_11,rt_q); t2<=fmul(`SP_CA_12,rt_r);
                        st<=ST_CA1A;
                    end
                    ST_CA1A: begin n_q<=t0+t1+t2; st<=ST_CA2; end
                    ST_CA2: begin
                        t0<=fmul(`SP_CA_20,rt_p); t1<=fmul(`SP_CA_21,rt_q); t2<=fmul(`SP_CA_22,rt_r);
                        st<=ST_CA2A;
                    end
                    ST_CA2A: begin rt_p<=n_p; rt_q<=n_q; rt_r<=t0+t1+t2; st<=ST_SF; end
                    ST_SF: begin
                        rt_p<=fmul(rt_p,`SP_RATE_SF_P);
                        rt_q<=fmul(rt_q,`SP_RATE_SF_Q);
                        rt_r<=fmul(rt_r,`SP_RATE_SF_R);
                        st<=ST_NL0;
                    end
                    ST_NL0: begin t0<=fmul(rt_p,rt_p); t1<=fmul(rt_q,rt_q); t2<=fmul(rt_r,rt_r); st<=ST_NL1; end
                    ST_NL1: begin t0<=fmul(t0,rt_p); t1<=fmul(t1,rt_q); t2<=fmul(t2,rt_r); st<=ST_NL2; end
                    ST_NL2: begin t0<=fmul(t0,`SP_NL_FS); t1<=fmul(t1,`SP_NL_FS); t2<=fmul(t2,`SP_NL_FS); st<=ST_NL3; end
                    ST_NL3: begin t0<=fmul(t0,inv_fs); t1<=fmul(t1,inv_fs); t2<=fmul(t2,inv_fs); st<=ST_NL4; end
                    ST_NL4: begin n_p<=fmul(t0,inv_fs); n_q<=fmul(t1,inv_fs); n_r<=fmul(t2,inv_fs); st<=ST_BIAS; end
                    ST_BIAS: begin
                        lfsr=xorshift_next(lfsr); bp=rate_bias_p+noise_approx(`SP_BIAS_DRIFT_STD);
                        lfsr=xorshift_next(lfsr); bq=rate_bias_q+noise_approx(`SP_BIAS_DRIFT_STD);
                        lfsr=xorshift_next(lfsr); br=rate_bias_r+noise_approx(`SP_BIAS_DRIFT_STD);
                        rate_bias_p<=bp; rate_bias_q<=bq; rate_bias_r<=br;
                        rt_p<=rt_p+n_p+bp+`SP_BIAS_TEMP_P;
                        rt_q<=rt_q+n_q+bq+`SP_BIAS_TEMP_Q;
                        rt_r<=rt_r+n_r+br+`SP_BIAS_TEMP_R;
                        st<=ST_RNOISE;
                    end
                    ST_RNOISE: begin
                        lfsr=xorshift_next(lfsr); rt_p<=rt_p+noise_approx(`SP_RATE_NOISE_STD);
                        lfsr=xorshift_next(lfsr); rt_q<=rt_q+noise_approx(`SP_RATE_NOISE_STD);
                        lfsr=xorshift_next(lfsr); rt_r<=rt_r+noise_approx(`SP_RATE_NOISE_STD);
                        st<=ST_RSAT;
                    end
                    ST_RSAT: begin
                        rt_p<=fquant(fclamp(rt_p,-`SP_RATE_SAT,`SP_RATE_SAT),`SP_RATE_QUANT);
                        rt_q<=fquant(fclamp(rt_q,-`SP_RATE_SAT,`SP_RATE_SAT),`SP_RATE_QUANT);
                        rt_r<=fquant(fclamp(rt_r,-`SP_RATE_SAT,`SP_RATE_SAT),`SP_RATE_QUANT);
                        st<=ST_ABIAS;
                    end
                    ST_ABIAS: begin
                        att_r<=att_r+`SP_ATT_BIAS_R;
                        att_p<=att_p+`SP_ATT_BIAS_P;
                        att_y<=att_y+`SP_ATT_BIAS_Y;
                        st<=ST_ASTD;
                    end
                    ST_ASTD: begin
                        std_r<=(fabs(rt_p)>`SP_DYN_RATE_THR)?`SP_ATT_NOISE_DY_R:`SP_ATT_NOISE_ST_R;
                        std_p<=(fabs(rt_q)>`SP_DYN_RATE_THR)?`SP_ATT_NOISE_DY_P:`SP_ATT_NOISE_ST_P;
                        std_y<=(fabs(rt_r)>`SP_DYN_RATE_THR)?`SP_ATT_NOISE_DY_Y:`SP_ATT_NOISE_ST_Y;
                        st<=ST_ANOISE;
                    end
                    ST_ANOISE: begin
                        lfsr=xorshift_next(lfsr); n_ar=noise_approx(std_r);
                        lfsr=xorshift_next(lfsr); n_ap=noise_approx(std_p);
                        lfsr=xorshift_next(lfsr); n_ay=noise_approx(std_y);
                        att_nbuf_r[att_nidx]<=n_ar;
                        att_nbuf_p[att_nidx]<=n_ap;
                        att_nbuf_y[att_nidx]<=n_ay;
                        case (att_nidx)
                            2'd0: begin
                                sum_r=n_ar+att_nbuf_r[1]+att_nbuf_r[2]+att_nbuf_r[3];
                                sum_p=n_ap+att_nbuf_p[1]+att_nbuf_p[2]+att_nbuf_p[3];
                                sum_y=n_ay+att_nbuf_y[1]+att_nbuf_y[2]+att_nbuf_y[3];
                            end
                            2'd1: begin
                                sum_r=att_nbuf_r[0]+n_ar+att_nbuf_r[2]+att_nbuf_r[3];
                                sum_p=att_nbuf_p[0]+n_ap+att_nbuf_p[2]+att_nbuf_p[3];
                                sum_y=att_nbuf_y[0]+n_ay+att_nbuf_y[2]+att_nbuf_y[3];
                            end
                            2'd2: begin
                                sum_r=att_nbuf_r[0]+att_nbuf_r[1]+n_ar+att_nbuf_r[3];
                                sum_p=att_nbuf_p[0]+att_nbuf_p[1]+n_ap+att_nbuf_p[3];
                                sum_y=att_nbuf_y[0]+att_nbuf_y[1]+n_ay+att_nbuf_y[3];
                            end
                            default: begin
                                sum_r=att_nbuf_r[0]+att_nbuf_r[1]+att_nbuf_r[2]+n_ar;
                                sum_p=att_nbuf_p[0]+att_nbuf_p[1]+att_nbuf_p[2]+n_ap;
                                sum_y=att_nbuf_y[0]+att_nbuf_y[1]+att_nbuf_y[2]+n_ay;
                            end
                        endcase
                        att_nidx<=att_nidx+2'd1;
                        att_r<=att_r+fmul(sum_r,`SP_ONE_QUARTER);
                        att_p<=att_p+fmul(sum_p,`SP_ONE_QUARTER);
                        att_y<=att_y+fmul(sum_y,`SP_ONE_QUARTER);
                        st<=ST_AQUANT;
                    end
                    ST_AQUANT: begin
                        roll_out  <= fquant(att_r, `SP_ATT_QUANT);
                        pitch_out <= fquant(att_p, `SP_ATT_QUANT);
                        yaw_out   <= fquant(att_y, `SP_ATT_QUANT);
                        p_out<=rt_p; q_out<=rt_q; r_out<=rt_r;
                        st<=ST_DONE;
                    end
                    ST_DONE: begin done<=1; st<=ST_IDLE; end
                    default: st<=ST_IDLE;
                endcase
            end
        end
    end endgenerate

endmodule
