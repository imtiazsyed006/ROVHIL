//----------------------------------------------------------------------
// tau_alloc.v -- Compute generalized torques Mx, My, Mz from motor forces
//
//   Replicates Python wrench_to_net_wrench_t200_14v():
//     f_N[i]  = u[i] * (u[i] > 0 ? MAX_FWD_N : MAX_REV_N)      [Newtons]
//     tau     = B_moment_rows @ f_N                            [N*m]
//   (u -> force must use the max force of u's OWN sign because the T200
//    curve is asymmetric: 4.5238 kgf forward vs 3.5168 kgf reverse. The
//    9.80665 kgf->N factor is folded into the FP_MAX_*_N constants.)
//
//   3-stage pipeline (each stage isolated by registers):
//     S0: 8 u -> f_N scale multiplies (Q16.16)
//     S1: 24 Q16.16 products  B[i] * f_N[i]
//     S2: 3 axis sums of 8 terms each -> Mx, My, Mz in N*m
//
//   Total latency = 3 clocks.
//----------------------------------------------------------------------
`include "rov_params.vh"

module tau_alloc (
    input  wire                clk,
    input  wire                rst,

    input  wire [255:0]        u_flat,

    output wire signed [31:0]  tau_mx,
    output wire signed [31:0]  tau_my,
    output wire signed [31:0]  tau_mz

);

    //==================================================================
    // unpack the 8 normalized motor commands (signed Q16.16, [-1, 1])
    //==================================================================
    wire signed [31:0] u0, u1, u2, u3, u4, u5, u6, u7;
    assign u0 = u_flat[ 0*32 +: 32];
    assign u1 = u_flat[ 1*32 +: 32];
    assign u2 = u_flat[ 2*32 +: 32];
    assign u3 = u_flat[ 3*32 +: 32];
    assign u4 = u_flat[ 4*32 +: 32];
    assign u5 = u_flat[ 5*32 +: 32];
    assign u6 = u_flat[ 6*32 +: 32];
    assign u7 = u_flat[ 7*32 +: 32];

    // Q16.16 multiply helper (round-to-nearest).
    // 32x32 is multi-cycle at 100 MHz; tau_alloc pipelines stages so each
    // fmul is reg->DSP->reg with a spare cycle via the FSM stage delay.
    function signed [31:0] fmul;
        input signed [31:0] a, b;
        reg signed [63:0] p;
        begin
            p    = a * b + 64'sd32768;
            fmul = p[47:16];
        end
    endfunction

    // 64-bit accumulated Q32.32 sum -> Q16.16 with round-to-nearest
    function signed [31:0] sum_rnd;
        input signed [63:0] s;
        reg signed [63:0] r;
        begin
            r       = s + 64'sd32768;
            sum_rnd = r[47:16];
        end
    endfunction

    //==================================================================
    // Pipeline registers
    //==================================================================
    // Stage 0: normalized u -> thruster force in Newtons
    reg signed [31:0] f0, f1, f2, f3, f4, f5, f6, f7;

    // Stage 1: 24 Q16.16 products  (32x32 = DSP48 native, 64-bit target)
    reg signed [63:0] mx_p0, mx_p1, mx_p2, mx_p3, mx_p4, mx_p5, mx_p6, mx_p7;
    reg signed [63:0] my_p0, my_p1, my_p2, my_p3, my_p4, my_p5, my_p6, my_p7;
    reg signed [63:0] mz_p0, mz_p1, mz_p2, mz_p3, mz_p4, mz_p5, mz_p6, mz_p7;

    // Stage 2: 3 axis sums -> N*m (Q16.16, 32-bit)
    reg signed [31:0] tau_mx_r, tau_my_r, tau_mz_r;
    assign tau_mx = tau_mx_r;
    assign tau_my = tau_my_r;
    assign tau_mz = tau_mz_r;

    //==================================================================
    // Pipeline (3 stages, all registered)
    //==================================================================
    always @(posedge clk) begin
        if (rst) begin
            f0 <= 0; f1 <= 0; f2 <= 0; f3 <= 0;
            f4 <= 0; f5 <= 0; f6 <= 0; f7 <= 0;
            mx_p0 <= 0; mx_p1 <= 0; mx_p2 <= 0; mx_p3 <= 0;
            mx_p4 <= 0; mx_p5 <= 0; mx_p6 <= 0; mx_p7 <= 0;
            my_p0 <= 0; my_p1 <= 0; my_p2 <= 0; my_p3 <= 0;
            my_p4 <= 0; my_p5 <= 0; my_p6 <= 0; my_p7 <= 0;
            mz_p0 <= 0; mz_p1 <= 0; mz_p2 <= 0; mz_p3 <= 0;
            mz_p4 <= 0; mz_p5 <= 0; mz_p6 <= 0; mz_p7 <= 0;
            tau_mx_r <= 0; tau_my_r <= 0; tau_mz_r <= 0;
        end else begin
            // ---- Stage 0: u -> force in Newtons (asymmetric fwd/rev) ----
            f0 <= fmul(u0, u0[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f1 <= fmul(u1, u1[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f2 <= fmul(u2, u2[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f3 <= fmul(u3, u3[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f4 <= fmul(u4, u4[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f5 <= fmul(u5, u5[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f6 <= fmul(u6, u6[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);
            f7 <= fmul(u7, u7[31] ? `FP_MAX_REV_N : `FP_MAX_FWD_N);

            // ---- Stage 1: products (32x32, assigned to 64-bit regs) ----
            mx_p0 <= `FP_B_MX0 * f0;
            mx_p1 <= `FP_B_MX1 * f1;
            mx_p2 <= `FP_B_MX2 * f2;
            mx_p3 <= `FP_B_MX3 * f3;
            mx_p4 <= `FP_B_MX4 * f4;
            mx_p5 <= `FP_B_MX5 * f5;
            mx_p6 <= `FP_B_MX6 * f6;
            mx_p7 <= `FP_B_MX7 * f7;

            my_p0 <= `FP_B_MY0 * f0;
            my_p1 <= `FP_B_MY1 * f1;
            my_p2 <= `FP_B_MY2 * f2;
            my_p3 <= `FP_B_MY3 * f3;
            my_p4 <= `FP_B_MY4 * f4;
            my_p5 <= `FP_B_MY5 * f5;
            my_p6 <= `FP_B_MY6 * f6;
            my_p7 <= `FP_B_MY7 * f7;

            mz_p0 <= `FP_B_MZ0 * f0;
            mz_p1 <= `FP_B_MZ1 * f1;
            mz_p2 <= `FP_B_MZ2 * f2;
            mz_p3 <= `FP_B_MZ3 * f3;
            mz_p4 <= `FP_B_MZ4 * f4;
            mz_p5 <= `FP_B_MZ5 * f5;
            mz_p6 <= `FP_B_MZ6 * f6;
            mz_p7 <= `FP_B_MZ7 * f7;

            // ---- Stage 2: sum 8 terms per axis -> N*m (Q16.16) ----
            // Sum at full 64-bit precision, round once (per-term [47:16]
            // slicing would floor 8 times and bias tau ~-4 LSB).
            tau_mx_r <= sum_rnd(mx_p0 + mx_p1 + mx_p2 + mx_p3
                              + mx_p4 + mx_p5 + mx_p6 + mx_p7);

            tau_my_r <= sum_rnd(my_p0 + my_p1 + my_p2 + my_p3
                              + my_p4 + my_p5 + my_p6 + my_p7);

            tau_mz_r <= sum_rnd(mz_p0 + mz_p1 + mz_p2 + mz_p3
                              + mz_p4 + mz_p5 + mz_p6 + mz_p7);
        end
    end

endmodule
