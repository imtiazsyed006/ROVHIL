//----------------------------------------------------------------------
// tau_alloc.v -- Compute generalized torques Mx, My, Mz from motor forces
//
//   3-stage pipeline (each stage isolated by registers):
//     S1: 24 Q16.16 products  B[i] * u[i]  (registered, 32x32 = DSP48-friendly)
//     S2: 3 axis sums of 8 terms each      (registered)
//     S3: 3 kgf*m -> N*m final scale       (registered)
//
//   Total latency = 3 clocks.
//
//   Note: 32x32 signed multiply can overflow at extreme |thrust| > ~2.2 kgf.
//   This is acceptable for initial bring-up; a wider multiply path can be
//   added later once the PWM pipeline is verified.
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
    // unpack the 8 motor forces (signed Q16.16) from the flat bus
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

    //==================================================================
    // Pipeline registers
    //==================================================================
    // Stage 1: 24 Q16.16 products  (32x32 = DSP48 native, 64-bit target)
    reg signed [63:0] mx_p0, mx_p1, mx_p2, mx_p3, mx_p4, mx_p5, mx_p6, mx_p7;
    reg signed [63:0] my_p0, my_p1, my_p2, my_p3, my_p4, my_p5, my_p6, my_p7;
    reg signed [63:0] mz_p0, mz_p1, mz_p2, mz_p3, mz_p4, mz_p5, mz_p6, mz_p7;

    // Stage 2: 3 axis sums → N·m (Q16.16, 32-bit)
    //   Input u[] is normalized (Q16.16 [-65536,65536] = [-1,1]).
    //   B * u_sat gives wrench in N·m directly (MATLAB convention).
    //   The old ×9.80665 kgf·m→N·m is REMOVED.
    reg signed [31:0] tau_mx_r, tau_my_r, tau_mz_r;
    assign tau_mx = tau_mx_r;
    assign tau_my = tau_my_r;
    assign tau_mz = tau_mz_r;

    //==================================================================
    // Pipeline (2 stages, all registered)
    //==================================================================
    always @(posedge clk) begin
        if (rst) begin
            mx_p0 <= 0; mx_p1 <= 0; mx_p2 <= 0; mx_p3 <= 0;
            mx_p4 <= 0; mx_p5 <= 0; mx_p6 <= 0; mx_p7 <= 0;
            my_p0 <= 0; my_p1 <= 0; my_p2 <= 0; my_p3 <= 0;
            my_p4 <= 0; my_p5 <= 0; my_p6 <= 0; my_p7 <= 0;
            mz_p0 <= 0; mz_p1 <= 0; mz_p2 <= 0; mz_p3 <= 0;
            mz_p4 <= 0; mz_p5 <= 0; mz_p6 <= 0; mz_p7 <= 0;
            tau_mx_r <= 0; tau_my_r <= 0; tau_mz_r <= 0;
        end else begin
            // ---- Stage 1: products (32x32, assigned to 64-bit regs) ----
            mx_p0 <= `FP_B_MX0 * u0;
            mx_p1 <= `FP_B_MX1 * u1;
            mx_p2 <= `FP_B_MX2 * u2;
            mx_p3 <= `FP_B_MX3 * u3;
            mx_p4 <= `FP_B_MX4 * u4;
            mx_p5 <= `FP_B_MX5 * u5;
            mx_p6 <= `FP_B_MX6 * u6;
            mx_p7 <= `FP_B_MX7 * u7;

            my_p0 <= `FP_B_MY0 * u0;
            my_p1 <= `FP_B_MY1 * u1;
            my_p2 <= `FP_B_MY2 * u2;
            my_p3 <= `FP_B_MY3 * u3;
            my_p4 <= `FP_B_MY4 * u4;
            my_p5 <= `FP_B_MY5 * u5;
            my_p6 <= `FP_B_MY6 * u6;
            my_p7 <= `FP_B_MY7 * u7;

            mz_p0 <= `FP_B_MZ0 * u0;
            mz_p1 <= `FP_B_MZ1 * u1;
            mz_p2 <= `FP_B_MZ2 * u2;
            mz_p3 <= `FP_B_MZ3 * u3;
            mz_p4 <= `FP_B_MZ4 * u4;
            mz_p5 <= `FP_B_MZ5 * u5;
            mz_p6 <= `FP_B_MZ6 * u6;
            mz_p7 <= `FP_B_MZ7 * u7;

            // ---- Stage 2: sum 8 terms per axis → N·m (Q16.16) ----
            // B * u_sat gives N·m directly; no ×9.81 needed.
            tau_mx_r <= mx_p0[47:16] + mx_p1[47:16] + mx_p2[47:16]
                     + mx_p3[47:16] + mx_p4[47:16] + mx_p5[47:16]
                     + mx_p6[47:16] + mx_p7[47:16];

            tau_my_r <= my_p0[47:16] + my_p1[47:16] + my_p2[47:16]
                     + my_p3[47:16] + my_p4[47:16] + my_p5[47:16]
                     + my_p6[47:16] + my_p7[47:16];

            tau_mz_r <= mz_p0[47:16] + mz_p1[47:16] + mz_p2[47:16]
                     + mz_p3[47:16] + mz_p4[47:16] + mz_p5[47:16]
                     + mz_p6[47:16] + mz_p7[47:16];
        end
    end

endmodule