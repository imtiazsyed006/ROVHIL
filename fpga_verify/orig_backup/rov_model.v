//----------------------------------------------------------------------
// rov_model.v -- ROV 6-DOF rotational dynamics RK4 engine
// Q16.16 fixed-point. 8-stage pipelined datapath with CORDIC + divider.
// Each pipeline stage has at most 1 fmul depth for 100 MHz timing.
//----------------------------------------------------------------------
`include "rov_params.vh"

module rov_model (
    input  wire         clk,
    input  wire         rst,
    input  wire         start,
    output reg          done,

    input  wire [31:0]  tau0_roll, tau0_pitch, tau0_yaw,
    input  wire [31:0]  tau1_roll, tau1_pitch, tau1_yaw,

    output wire [31:0]  state_roll,
    output wire [31:0]  state_pitch,
    output wire [31:0]  state_yaw,
    output wire [31:0]  state_p,
    output wire [31:0]  state_q,
    output wire [31:0]  state_r,

    // ---- DEBUG TAP (read-only mirrors of internal pipeline registers) ----
    // Exposed so a 2nd UART can stream them when diagnosing the divergence.
    // These do not affect timing-critical paths (they're just extra reads on
    // existing registers). Remove once the bug is found.
    output wire [31:0]  dbg_st,        // FSM state
    output wire [1:0]   dbg_kphase,
    output wire [31:0]  dbg_rhs0, dbg_rhs1, dbg_rhs2,
    output wire [31:0]  dbg_n0,   dbg_n1,   dbg_n2,
    output wire [31:0]  dbg_det,
    output wire [31:0]  dbg_Pd,   dbg_Qd,   dbg_Rd
);

    //==================================================================
    // fixed-point arithmetic helpers
    //==================================================================
    function [31:0] fmul;
        input signed [31:0] a, b;
        reg signed [63:0] p;
        begin
            p   = a * b;
            fmul = p[47:16];
        end
    endfunction
    function [31:0] fclamp;
        input [31:0] v, lo, hi;
        begin
            if      ($signed(v) < $signed(lo)) fclamp = lo;
            else if ($signed(v) > $signed(hi)) fclamp = hi;
            else                                fclamp = v;
        end
    endfunction
    function [31:0] fwrap;
        input [31:0] v;
        begin
            if      ($signed(v) >  `FP_PI) fwrap = $signed(v) - `FP_TWO_PI;
            else if ($signed(v) < -`FP_PI) fwrap = $signed(v) + `FP_TWO_PI;
            else                           fwrap = v;
        end
    endfunction
    function [31:0] fabs;
        input [31:0] v;
        begin fabs = v[31] ? (~v + 1'b1) : v; end
    endfunction

    //==================================================================
    // CORDIC
    //==================================================================
    reg        cord_start;
    wire       cord_done;
    reg        cord_sincos;
    reg [31:0] cord_angle;
    wire [31:0] cord_sin, cord_cos, cord_atan2;

    cordic u_cordic (
        .clk(clk), .rst(rst),
        .start(cord_start), .done(cord_done),
        .op_sincos(cord_sincos),
        .angle_in(cord_angle),
        .sin_out(cord_sin), .cos_out(cord_cos),
        .atan2_y(32'd0), .atan2_x(32'd0),
        .atan2_out(cord_atan2)
    );

    //==================================================================
    // Serial divider
    //==================================================================
    reg        dvs;
    wire       dvd;
    reg [31:0] dvn;
    reg [31:0] dvd_en;
    wire [31:0] dvq;

    reg        dvb;
    reg [5:0]  dvc;
    reg        dvq_neg;
    reg [47:0] dv_div;
    reg [47:0] dv_rem;
    reg [31:0] dvqr;
    reg [31:0] dvden_abs;

    wire       dv_bit = dv_div[47];

    assign dvd = dvb && (dvc == 6'd48);
    assign dvq = dvq_neg ? {~dvqr + 1'b1} : dvqr;

    always @(posedge clk) begin
        if (rst) begin
            dvb <= 1'b0; dvc <= 6'd0; dv_div <= 48'd0; dv_rem <= 48'd0;
            dvqr <= 32'd0; dvq_neg <= 1'b0; dvden_abs <= 32'd0;
        end else begin
            if (dvs && !dvb) begin
                dvb       <= 1'b1;
                dvc       <= 6'd0;
                dvq_neg   <= (dvn[31] ^ dvd_en[31]);
                dvden_abs <= dvd_en[31] ? {~dvd_en + 1'b1} : dvd_en;
                dv_div    <= dvn[31] ? {(~dvn + 1'b1), 16'd0} : {dvn, 16'd0};
                dv_rem    <= 48'd0;
                dvqr      <= 32'd0;
            end else if (dvb) begin
                if (dvc < 6'd48) begin
                    dvc    <= dvc + 6'd1;
                    dv_div <= {dv_div[46:0], 1'b0};
                    if ({dv_rem[46:0], dv_bit} >= {16'd0, dvden_abs}) begin
                        dv_rem <= {dv_rem[46:0], dv_bit} - {16'd0, dvden_abs};
                        dvqr   <= {dvqr[30:0], 1'b1};
                    end else begin
                        dv_rem <= {dv_rem[46:0], dv_bit};
                        dvqr   <= {dvqr[30:0], 1'b0};
                    end
                end else begin
                    dvb <= 1'b0;
                end
            end
        end
    end

    //==================================================================
    // FSM states
    //==================================================================
    localparam [5:0]
        ST_IDLE     = 6'd0,
        ST_T_PHI    = 6'd1,   ST_W_PHI  = 6'd2,
        ST_T_THETA  = 6'd3,   ST_W_THETA= 6'd4,
        ST_DIV_TAN  = 6'd5,   ST_W_TAN  = 6'd6,
        ST_DIV_SOV  = 6'd7,   ST_W_SOV  = 6'd8,
        ST_DIV_COV  = 6'd9,   ST_W_COV  = 6'd10;

    localparam [5:0]
        ST_C1       = 6'd11,  // leaf fmul
        ST_C2       = 6'd12,  // Coriolis, dq, g
        ST_C3       = 6'd13,  // C*w
        ST_C3b      = 6'd14,  // RHS pre-sum, Euler pre-mul, t pre-mul
        ST_C4       = 6'd15,  // RHS final, t1-3 final, Euler sum
        ST_C5       = 6'd16,  // Euler clamp, det pre-mul, A/B/C pre-mul
        ST_C5b      = 6'd17,  // A/B/C sub
        ST_C6       = 6'd18,  // det final, n pre-mul
        ST_COMPUTE  = 6'd19;  // n final add/sub

    localparam [5:0]
        ST_DIV_PDOT = 6'd20,  ST_W_PDOT = 6'd21,
        ST_DIV_QDOT = 6'd22,  ST_W_QDOT = 6'd23,
        ST_DIV_RDOT = 6'd24,  ST_W_RDOT = 6'd25,
        ST_DERIV_DONE = 6'd26;

    localparam [5:0]
        ST_K1_SETUP = 6'd27,
        ST_K2_SETUP = 6'd28,
        ST_K3_SETUP = 6'd29,
        ST_K4_SETUP = 6'd30,
        ST_UPDATE_P1 = 6'd31,
        ST_UPDATE_P2 = 6'd32,
        ST_UPDATE_P3 = 6'd33,
        ST_UPDATE_P4 = 6'd34,
        ST_DONE_PULSE = 6'd35;

    reg [5:0] st;
    reg [1:0] k_phase;

    //==================================================================
    // State registers
    //==================================================================
    reg signed [31:0] sv_phi, sv_theta, sv_psi, sv_p, sv_q, sv_r;
    reg signed [31:0] x0_phi, x0_theta, x0_psi, x0_p, x0_q, x0_r;

    reg signed [31:0] K0_phi, K0_theta, K0_psi, K0_p, K0_q, K0_r;
    reg signed [31:0] K1_phi, K1_theta, K1_psi, K1_p, K1_q, K1_r;
    reg signed [31:0] K2_phi, K2_theta, K2_psi, K2_p, K2_q, K2_r;
    reg signed [31:0] K3_phi, K3_theta, K3_psi, K3_p, K3_q, K3_r;

    reg signed [31:0] tphi, ttheta, tpsi, tP, tQ, tR;
    reg signed [31:0] ttau_r, ttau_p, ttau_y;

    // Registered |p|,|q|,|r| -- precomputed (one cycle behind tP/tQ/tR) so
    // the quadratic-damping fmul(|w|*w) in ST_C1 does not drive a combinational
    // fabs incrementer directly into the DSP input. This was the single timing
    // hot spot (12-16 logic levels, WNS = -2.61 ns @ 100 MHz).
    reg signed [31:0] tP_abs, tQ_abs, tR_abs;

    reg signed [31:0] sph, cph, sth, cth;
    reg signed [31:0] tth, sov, cov;

    reg signed [31:0] l_phid, l_thd, l_psid, l_Pd, l_Qd, l_Rd;
    reg signed [31:0] l_det, l_n0, l_n1, l_n2;

    //==================================================================
    // Pipeline registers (8 stages, each <= 1 fmul or 1 add chain)
    //==================================================================
    // Stage 1: leaf fmul
    reg signed [31:0] r1_sph_tth, r1_cph_tth;
    reg signed [31:0] r1_cph_tQ,  r1_sph_tR;
    reg signed [31:0] r1_sov_tQ,  r1_cov_tR;
    reg signed [31:0] r1_cth_cph, r1_cth_sph;
    reg signed [31:0] r1_add0, r1_add1, r1_add2;        // diag added-mass * w  (K_dp*p, M_dq*q, N_dr*r)
    reg signed [31:0] r1_am01, r1_am02;                 // off-diag added-mass: K_dq*q, K_dr*r
    reg signed [31:0] r1_am10, r1_am12;                 // off-diag added-mass: M_dp*p, M_dr*r
    reg signed [31:0] r1_am20, r1_am21;                 // off-diag added-mass: N_dp*p, N_dq*q
    reg signed [31:0] r1_IxzP, r1_IzR;
    reg signed [31:0] r1_IyQ,  r1_IxP;
    reg signed [31:0] r1_dl0, r1_dl1, r1_dl2;            // diag linear damping (K_p*p, M_q*q, N_r*r)
    reg signed [31:0] r1_dl01, r1_dl02;                 // off-diag linear damp: K_q*q, K_r*r
    reg signed [31:0] r1_dl10, r1_dl12;                 // off-diag linear damp: M_p*p, M_r*r
    reg signed [31:0] r1_dl20, r1_dl21;                 // off-diag linear damp: N_p*p, N_q*q
    reg signed [31:0] r1_tg0, r1_tg1, r1_tg2;
    reg signed [31:0] r1_wap, r1_waq, r1_war;           // diag quad-damp operand |w|*w
    // Off-diagonal quad-damp operands |w|*w share r1_wap/waq/war (same values,
    // different scalar coefficients applied in ST_C2).

    // Stage 2: Coriolis, quad damping, restoring
    reg signed [31:0] r2_C45, r2_C46, r2_C56;
    reg signed [31:0] r2_dq0, r2_dq1, r2_dq2;            // diag quad-damp rows (K_pp|p|p, M_qq|q|q, N_rr|r|r)
    reg signed [31:0] r2_dq01, r2_dq02;                 // off-diag quad-damp: K_qq|q|q, K_rr|r|r
    reg signed [31:0] r2_dq10, r2_dq12;                 // off-diag quad-damp: M_pp|p|p, M_rr|r|r
    reg signed [31:0] r2_dq20, r2_dq21;                 // off-diag quad-damp: N_pp|p|p, N_qq|q|q
    reg signed [31:0] r2_g4, r2_g5, r2_g6;

    // Stage 3: C*w
    reg signed [31:0] r3_cw0, r3_cw1, r3_cw2;

    // Stage 3b: RHS pre-sum, Euler pre-mul, t pre-mul
    reg signed [31:0] r3b_neg0_0, r3b_neg1_0, r3b_neg2_0;
    reg signed [31:0] r3b_neg0_1, r3b_neg1_1, r3b_neg2_1;
    reg signed [31:0] r3b_eul0, r3b_eul1;
    reg signed [31:0] r3b_t1a, r3b_t1b, r3b_t2a, r3b_t2b, r3b_t3a, r3b_t3b;

    // Stage 4: RHS final, t1-3 final, Euler intermediate sum
    reg signed [31:0] r4_rhs0, r4_rhs1, r4_rhs2;
    reg signed [31:0] r4_t1, r4_t2, r4_t3;
    reg signed [31:0] r4_phid_sum;

    // Stage 5: Euler clamp, det pre-mul, A/B/C pre-mul
    reg signed [31:0] r5_d0, r5_d1, r5_d2;
    reg signed [31:0] r5_A0, r5_A1, r5_B0, r5_B1, r5_C0, r5_C1;

    // Stage 5b: A/B/C sub
    reg signed [31:0] r5b_A, r5b_B, r5b_C;

    // Stage 6: det final, Cramer n pre-mul
    reg signed [31:0] r6_det;
    reg signed [31:0] r6_n0_0, r6_n0_1, r6_n0_2;
    reg signed [31:0] r6_n1_0, r6_n1_1, r6_n1_2;
    reg signed [31:0] r6_n2_0, r6_n2_1, r6_n2_2;

    // Update pipeline registers (4 stages for RK4 state update)
    reg signed [31:0] up_phi_s0,   up_phi_s1,   up_phi_sum,   up_phi_dt;
    reg signed [31:0] up_theta_s0, up_theta_s1, up_theta_sum, up_theta_dt;
    reg signed [31:0] up_psi_s0,   up_psi_s1,   up_psi_sum,   up_psi_dt;
    reg signed [31:0] up_p_s0,     up_p_s1,     up_p_sum,     up_p_dt;
    reg signed [31:0] up_q_s0,     up_q_s1,     up_q_sum,     up_q_dt;
    reg signed [31:0] up_r_s0,     up_r_s1,     up_r_sum,     up_r_dt;

    // Inertia matrix M elements (const wire) -- FULL coupled matrix
    //   M[i,j] = -I_ij + added-mass[i,j]   (added-mass replaces 0 on diagonal)
    wire [31:0] w_M11 = `FP_I_X + `FP_K_dp;
    wire [31:0] w_M12 = -`FP_I_XY + `FP_K_dq;
    wire [31:0] w_M13 = -`FP_I_XZ + `FP_K_dr;
    wire [31:0] w_M21 = -`FP_I_XY + `FP_M_dp;
    wire [31:0] w_M22 = `FP_I_Y + `FP_M_dq;
    wire [31:0] w_M23 = -`FP_I_YZ + `FP_M_dr;
    wire [31:0] w_M31 = -`FP_I_XZ + `FP_N_dp;
    wire [31:0] w_M32 = -`FP_I_YZ + `FP_N_dq;
    wire [31:0] w_M33 = `FP_I_Z + `FP_N_dr;

    //==================================================================
    // FSM
    //==================================================================
    always @(posedge clk) begin
        if (rst) begin
            st      <= ST_IDLE;
            k_phase <= 2'd0;
            done    <= 1'b0;
            cord_start <= 1'b0;
            cord_sincos<= 1'b0;
            dvs     <= 1'b0;
            sv_phi  <= 0; sv_theta <= 0; sv_psi <= 0; sv_p <= 0; sv_q <= 0; sv_r <= 0;
            tP_abs  <= 0; tQ_abs   <= 0; tR_abs <= 0;
            l_phid  <= 0; l_thd <= 0; l_psid <= 0;
            l_det   <= 0; l_n0 <= 0; l_n1 <= 0; l_n2 <= 0;
            l_Pd    <= 0; l_Qd <= 0; l_Rd <= 0;
        end else begin
            done       <= 1'b0;
            cord_start <= 1'b0;
            dvs        <= 1'b0;

            case (st)

                // ---- IDLE ----
                ST_IDLE: if (start) begin
                    x0_phi  <= sv_phi;  x0_theta <= sv_theta; x0_psi <= sv_psi;
                    x0_p    <= sv_p;    x0_q     <= sv_q;     x0_r   <= sv_r;
                    k_phase <= 2'd0;
                    st      <= ST_K1_SETUP;
                end

                // ---- SETUP states ----
                ST_K1_SETUP: begin
                    tphi <= x0_phi; ttheta <= x0_theta; tpsi <= x0_psi;
                    tP   <= x0_p;   tQ     <= x0_q;     tR   <= x0_r;
                    ttau_r <= tau0_roll; ttau_p <= tau0_pitch; ttau_y <= tau0_yaw;
                    st <= ST_T_PHI;
                end
                ST_K2_SETUP: begin
                    tphi <= x0_phi + fmul(`FP_HALF_DT, K0_phi);
                    ttheta<=x0_theta+ fmul(`FP_HALF_DT, K0_theta);
                    tpsi <= x0_psi + fmul(`FP_HALF_DT, K0_psi);
                    tP   <= x0_p   + fmul(`FP_HALF_DT, K0_p);
                    tQ   <= x0_q   + fmul(`FP_HALF_DT, K0_q);
                    tR   <= x0_r   + fmul(`FP_HALF_DT, K0_r);
                    ttau_r <= (tau0_roll  + tau1_roll)  >>> 1;
                    ttau_p <= (tau0_pitch + tau1_pitch) >>> 1;
                    ttau_y <= (tau0_yaw   + tau1_yaw)   >>> 1;
                    st <= ST_T_PHI;
                end
                ST_K3_SETUP: begin
                    tphi <= x0_phi + fmul(`FP_HALF_DT, K1_phi);
                    ttheta<=x0_theta+ fmul(`FP_HALF_DT, K1_theta);
                    tpsi <= x0_psi + fmul(`FP_HALF_DT, K1_psi);
                    tP   <= x0_p   + fmul(`FP_HALF_DT, K1_p);
                    tQ   <= x0_q   + fmul(`FP_HALF_DT, K1_q);
                    tR   <= x0_r   + fmul(`FP_HALF_DT, K1_r);
                    ttau_r <= (tau0_roll  + tau1_roll)  >>> 1;
                    ttau_p <= (tau0_pitch + tau1_pitch) >>> 1;
                    ttau_y <= (tau0_yaw   + tau1_yaw)   >>> 1;
                    st <= ST_T_PHI;
                end
                ST_K4_SETUP: begin
                    tphi <= x0_phi + fmul(`FP_DT, K2_phi);
                    ttheta<=x0_theta+ fmul(`FP_DT, K2_theta);
                    tpsi <= x0_psi + fmul(`FP_DT, K2_psi);
                    tP   <= x0_p   + fmul(`FP_DT, K2_p);
                    tQ   <= x0_q   + fmul(`FP_DT, K2_q);
                    tR   <= x0_r   + fmul(`FP_DT, K2_r);
                    ttau_r <= tau1_roll; ttau_p <= tau1_pitch; ttau_y <= tau1_yaw;
                    st <= ST_T_PHI;
                end

                // ---- TRIG PHI ----
                // Latch |p|,|q|,|r| here (first state after every K_SETUP) so the
                // quadratic-damping fmul in ST_C1 sees a registered abs operand,
                // not a combinational ~v+1 incrementer sitting on the DSP input.
                ST_T_PHI:   begin
                    cord_start<=1; cord_sincos<=1; cord_angle<=tphi;
                    tP_abs <= fabs(tP);
                    tQ_abs <= fabs(tQ);
                    tR_abs <= fabs(tR);
                    st<=ST_W_PHI;
                end
                ST_W_PHI:   if (cord_done) begin sph<=cord_sin; cph<=cord_cos; st<=ST_T_THETA; end

                // ---- TRIG THETA ----
                ST_T_THETA: begin cord_start<=1; cord_sincos<=1; cord_angle<=ttheta; st<=ST_W_THETA; end
                ST_W_THETA: if (cord_done) begin sth<=cord_sin; cth<=cord_cos; st<=ST_DIV_TAN; end

                // ---- DIV: tan_theta = sin/cos ----
                ST_DIV_TAN: begin
                    dvs<=1; dvn<=sth;
                    dvd_en <= ($signed(cth)==0) ? 32'sd1 : cth;
                    st<=ST_W_TAN;
                end
                ST_W_TAN: if (dvd) begin tth<=dvq; st<=ST_DIV_SOV; end

                // ---- DIV: sin_phi/cos_theta ----
                ST_DIV_SOV: begin
                    dvs<=1; dvn<=sph;
                    dvd_en <= ($signed(cth)==0) ? 32'sd1 : cth;
                    st<=ST_W_SOV;
                end
                ST_W_SOV: if (dvd) begin sov<=dvq; st<=ST_DIV_COV; end

                // ---- DIV: cos_phi/cos_theta ----
                ST_DIV_COV: begin
                    dvs<=1; dvn<=cph;
                    dvd_en <= ($signed(cth)==0) ? 32'sd1 : cth;
                    st<=ST_W_COV;
                end
                ST_W_COV: if (dvd) begin cov<=dvq; st<=ST_C1; end

                //==========================================================
                // ST_C1: leaf fmul
                //==========================================================
                ST_C1: begin
                    r1_sph_tth <= fmul(sph, tth);
                    r1_cph_tth <= fmul(cph, tth);
                    r1_cph_tQ  <= fmul(cph, tQ);
                    r1_sph_tR  <= fmul(sph, tR);
                    r1_sov_tQ  <= fmul(sov, tQ);
                    r1_cov_tR  <= fmul(cov, tR);
                    r1_cth_cph <= fmul(cth, cph);
                    r1_cth_sph <= fmul(cth, sph);
                    // diag added-mass * w  (M_added diagonal: K_dp*p, M_dq*q, N_dr*r)
                    r1_add0 <= fmul(`FP_K_dp, tP);
                    r1_add1 <= fmul(`FP_M_dq, tQ);
                    r1_add2 <= fmul(`FP_N_dr, tR);
                    // off-diag added-mass * w  (M_added[i][j] * w[j])
                    r1_am01 <= fmul(`FP_K_dq, tQ);   // M_added[0][1] * q
                    r1_am02 <= fmul(`FP_K_dr, tR);   // M_added[0][2] * r
                    r1_am10 <= fmul(`FP_M_dp, tP);   // M_added[1][0] * p
                    r1_am12 <= fmul(`FP_M_dr, tR);   // M_added[1][2] * r
                    r1_am20 <= fmul(`FP_N_dp, tP);   // M_added[2][0] * p
                    r1_am21 <= fmul(`FP_N_dq, tQ);   // M_added[2][1] * q
                    r1_IxzP <= fmul(`FP_I_XZ, tP);
                    r1_IzR  <= fmul(`FP_I_Z,  tR);
                    r1_IyQ  <= fmul(`FP_I_Y,  tQ);
                    r1_IxP  <= fmul(`FP_I_X,  tP);
                    // diag linear damping (D_linear diagonal: K_p*p, M_q*q, N_r*r)
                    r1_dl0 <= fmul(`FP_K_p, tP);
                    r1_dl1 <= fmul(`FP_M_q, tQ);
                    r1_dl2 <= fmul(`FP_N_r, tR);
                    // off-diag linear damping (D_linear[i][j] * w[j])
                    r1_dl01 <= fmul(`FP_K_q, tQ);   // D_linear[0][1] * q
                    r1_dl02 <= fmul(`FP_K_r, tR);   // D_linear[0][2] * r
                    r1_dl10 <= fmul(`FP_M_p, tP);   // D_linear[1][0] * p
                    r1_dl12 <= fmul(`FP_M_r, tR);   // D_linear[1][2] * r
                    r1_dl20 <= fmul(`FP_N_p, tP);   // D_linear[2][0] * p
                    r1_dl21 <= fmul(`FP_N_q, tQ);   // D_linear[2][1] * q
                    r1_tg0 <= fmul(`FP_G_p, ttau_r);
                    r1_tg1 <= fmul(`FP_G_q, ttau_p);
                    r1_tg2 <= fmul(`FP_G_r, ttau_y);
                    r1_wap <= fmul(tP_abs, tP);
                    r1_waq <= fmul(tQ_abs, tQ);
                    r1_war <= fmul(tR_abs, tR);
                    st <= ST_C2;
                end

                //==========================================================
                // ST_C2: Coriolis, quad damping, restoring
                //
                // Full Coriolis vector b = -M_added @ w, where M_added is the full
                // 3x3 added-mass matrix (Python pqr_rotational_accel):
                //   b0 = -(K_dp*p + K_dq*q + K_dr*r) = -(r1_add0 + r1_am01 + r1_am02)
                //   b1 = -(M_dp*p + M_dq*q + M_dr*r) = -(r1_am10 + r1_add1 + r1_am12)
                //   b2 = -(N_dp*p + N_dq*q + N_dr*r) = -(r1_am20 + r1_am21 + r1_add2)
                //
                // Then (Python):
                //   C45 = -I_xy*q - I_xz*p + I_z*r - b2
                //   C46 =  I_yz*r + I_xy*p - I_y*q + b1
                //   C56 = -I_xz*r - I_xy*q + I_x*p - b0
                //
                // Quad damping (full D_quad rows): D_quad @ w_abs_w
                //   row0 = K_pp|p|p + K_qq|q|q + K_rr|r|r
                //   row1 = M_pp|p|p + M_qq|q|q + M_rr|r|r
                //   row2 = N_pp|p|p + N_qq|q|q + N_rr|r|r
                //==========================================================
                ST_C2: begin
                    // b1 = -(am10 + add1 + am12);  b2 = -(am20 + am21 + add2)
                    r2_C45 <= -r1_IxzP + r1_IzR
                              + r1_am20 + r1_am21 + r1_add2;          // ... - b2 = ... + (am20+am21+add2)
                    r2_C46 <= -r1_IyQ
                              - r1_am10 - r1_add1 - r1_am12;          // b1 = -(...); C46 += b1
                    // b0 = -(add0 + am01 + am02);  C56 += -b0 = +(add0+am01+am02)
                    r2_C56 <=  r1_IxP
                              + r1_add0 + r1_am01 + r1_am02;
                    // full quad-damping rows (D_quad @ |w|w)
                    r2_dq0  <= fmul(`FP_K_pp, r1_wap);
                    r2_dq01 <= fmul(`FP_K_qq, r1_waq);
                    r2_dq02 <= fmul(`FP_K_rr, r1_war);
                    r2_dq1  <= fmul(`FP_M_pp, r1_wap);
                    r2_dq10 <= fmul(`FP_M_qq, r1_waq);
                    r2_dq12 <= fmul(`FP_M_rr, r1_war);
                    r2_dq2  <= fmul(`FP_N_pp, r1_wap);
                    r2_dq20 <= fmul(`FP_N_qq, r1_waq);
                    r2_dq21 <= fmul(`FP_N_rr, r1_war);
                    r2_g4 <=  fmul(`FP_WY_B, r1_cth_cph) - fmul(`FP_WZ_B, r1_cth_sph);
                    r2_g5 <= -fmul(`FP_WZ_B, sth)         - fmul(`FP_WX_B, r1_cth_cph);
                    r2_g6 <=  fmul(`FP_WX_B, r1_cth_sph) + fmul(`FP_WY_B, sth);
                    st <= ST_C3;
                end

                //==========================================================
                // ST_C3: C*w
                //==========================================================
                ST_C3: begin
                    r3_cw0 <= fmul(r2_C45, tQ) + fmul(r2_C46, tR);
                    r3_cw1 <= -fmul(r2_C45, tP) + fmul(r2_C56, tR);
                    r3_cw2 <= -fmul(r2_C46, tP) - fmul(r2_C56, tQ);
                    st <= ST_C3b;
                end

                //==========================================================
                // ST_C3b: RHS pre-sum (full damping rows), Euler pre-mul, t1-3 pre-mul
                //   neg{i}_0 = (C@w)[i] + D_linear full row i
                //   neg{i}_1 = D_quad full row i + restoring[i]
                //   ST_C4: rhs[i] = tau[i] - (neg{i}_0 + neg{i}_1)
                //==========================================================
                ST_C3b: begin
                    r3b_neg0_0 <= r3_cw0 + r1_dl0 + r1_dl01 + r1_dl02;
                    r3b_neg0_1 <= r2_dq0 + r2_dq01 + r2_dq02 + r2_g4;
                    r3b_neg1_0 <= r3_cw1 + r1_dl1 + r1_dl10 + r1_dl12;
                    r3b_neg1_1 <= r2_dq1 + r2_dq10 + r2_dq12 + r2_g5;
                    r3b_neg2_0 <= r3_cw2 + r1_dl2 + r1_dl20 + r1_dl21;
                    r3b_neg2_1 <= r2_dq2 + r2_dq20 + r2_dq21 + r2_g6;
                    r3b_eul0 <= fmul(r1_sph_tth, tQ);
                    r3b_eul1 <= fmul(r1_cph_tth, tR);
                    r3b_t1a <= fmul(w_M22, w_M33);
                    r3b_t1b <= fmul(w_M23, w_M32);
                    r3b_t2a <= fmul(w_M21, w_M33);
                    r3b_t2b <= fmul(w_M23, w_M31);
                    r3b_t3a <= fmul(w_M21, w_M32);
                    r3b_t3b <= fmul(w_M22, w_M31);
                    st <= ST_C4;
                end

                //==========================================================
                // ST_C4: RHS final, t1-3 final, Euler sum, thd/psid
                //==========================================================
                ST_C4: begin
                    r4_rhs0 <= r1_tg0 - (r3b_neg0_0 + r3b_neg0_1);
                    r4_rhs1 <= r1_tg1 - (r3b_neg1_0 + r3b_neg1_1);
                    r4_rhs2 <= r1_tg2 - (r3b_neg2_0 + r3b_neg2_1);
                    r4_t1 <= r3b_t1a - r3b_t1b;
                    r4_t2 <= r3b_t2a - r3b_t2b;
                    r4_t3 <= r3b_t3a - r3b_t3b;
                    r4_phid_sum <= tP + r3b_eul0;
                    l_thd  <= fclamp(r1_cph_tQ - r1_sph_tR, -`FP_ACCEL_CLIP, `FP_ACCEL_CLIP);
                    l_psid <= fclamp(r1_sov_tQ + r1_cov_tR, -`FP_ACCEL_CLIP, `FP_ACCEL_CLIP);
                    st <= ST_C5;
                end

                //==========================================================
                // ST_C5: Euler clamp, det pre-mul, A/B/C pre-mul
                //==========================================================
                ST_C5: begin
                    l_phid <= fclamp(r4_phid_sum + r3b_eul1, -`FP_ACCEL_CLIP, `FP_ACCEL_CLIP);
                    r5_d0 <= fmul(w_M11, r4_t1);
                    r5_d1 <= fmul(w_M12, r4_t2);
                    r5_d2 <= fmul(w_M13, r4_t3);
                    r5_A0 <= fmul(r4_rhs1, w_M33);
                    r5_A1 <= fmul(w_M23, r4_rhs2);
                    r5_B0 <= fmul(r4_rhs1, w_M32);
                    r5_B1 <= fmul(w_M22, r4_rhs2);
                    r5_C0 <= fmul(w_M21, r4_rhs2);
                    r5_C1 <= fmul(r4_rhs1, w_M31);
                    st <= ST_C5b;
                end

                //==========================================================
                // ST_C5b: A/B/C sub (separate from fmul for timing)
                //   A = A0 - A1   B = B0 - B1   C = C0 - C1
                //==========================================================
                ST_C5b: begin
                    r5b_A <= r5_A0 - r5_A1;
                    r5b_B <= r5_B0 - r5_B1;
                    r5b_C <= r5_C0 - r5_C1;
                    st <= ST_C6;
                end

                //==========================================================
                // ST_C6: det final, Cramer n pre-mul
                //   det = d0 - d1 + d2
                //   n0_0 = rhs0*t1   n0_1 = M12*A   n0_2 = M13*B
                //   n1_0 = M11*A     n1_1 = rhs0*t2 n1_2 = M13*C
                //   n2_0 = rhs0*t3   n2_1 = M11*B   n2_2 = M12*C
                //==========================================================
                ST_C6: begin
                    r6_det <= r5_d0 - r5_d1 + r5_d2;
                    r6_n0_0 <= fmul(r4_rhs0, r4_t1);
                    r6_n0_1 <= fmul(w_M12, r5b_A);
                    r6_n0_2 <= fmul(w_M13, r5b_B);
                    r6_n1_0 <= fmul(w_M11, r5b_A);
                    r6_n1_1 <= fmul(r4_rhs0, r4_t2);
                    r6_n1_2 <= fmul(w_M13, r5b_C);
                    r6_n2_0 <= fmul(r4_rhs0, r4_t3);
                    r6_n2_1 <= fmul(w_M11, r5b_B);
                    r6_n2_2 <= fmul(w_M12, r5b_C);
                    st <= ST_COMPUTE;
                end

                //==========================================================
                // ST_COMPUTE: Cramer n final add/sub + det pass-through
                //   n0 = n0_0 - n0_1 + n0_2
                //   n1 = n1_0 - n1_1 + n1_2
                //   n2 = n2_0 - n2_1 - n2_2
                //==========================================================
                ST_COMPUTE: begin
                    l_det <= r6_det;
                    l_n0  <= r6_n0_0 - r6_n0_1 + r6_n0_2;
                    l_n1  <= r6_n1_0 - r6_n1_1 + r6_n1_2;
                    l_n2  <= r6_n2_0 - r6_n2_1 - r6_n2_2;
                    st <= ST_DIV_PDOT;
                end

                // ---- DIV: p_dot = n0 / det ----
                ST_DIV_PDOT: begin
                    dvs<=1; dvn<=l_n0;
                    dvd_en <= ($signed(l_det)==0) ? 32'sd1 : l_det;
                    st<=ST_W_PDOT;
                end
                ST_W_PDOT: if (dvd) begin
                    l_Pd <= fclamp(dvq, -`FP_ACCEL_CLIP, `FP_ACCEL_CLIP);
                    st <= ST_DIV_QDOT;
                end

                // ---- DIV: q_dot = n1 / det ----
                ST_DIV_QDOT: begin
                    dvs<=1; dvn<=l_n1;
                    dvd_en <= ($signed(l_det)==0) ? 32'sd1 : l_det;
                    st<=ST_W_QDOT;
                end
                ST_W_QDOT: if (dvd) begin
                    l_Qd <= fclamp(dvq, -`FP_ACCEL_CLIP, `FP_ACCEL_CLIP);
                    st <= ST_DIV_RDOT;
                end

                // ---- DIV: r_dot = n2 / det ----
                ST_DIV_RDOT: begin
                    dvs<=1; dvn<=l_n2;
                    dvd_en <= ($signed(l_det)==0) ? 32'sd1 : l_det;
                    st<=ST_W_RDOT;
                end
                ST_W_RDOT: if (dvd) begin
                    l_Rd <= fclamp(dvq, -`FP_ACCEL_CLIP, `FP_ACCEL_CLIP);
                    st <= ST_DERIV_DONE;
                end

                // ---- DERIV DONE: store k vector ----
                ST_DERIV_DONE: begin
                    case (k_phase)
                        2'd0: begin K0_phi<=l_phid; K0_theta<=l_thd; K0_psi<=l_psid; K0_p<=l_Pd; K0_q<=l_Qd; K0_r<=l_Rd; end
                        2'd1: begin K1_phi<=l_phid; K1_theta<=l_thd; K1_psi<=l_psid; K1_p<=l_Pd; K1_q<=l_Qd; K1_r<=l_Rd; end
                        2'd2: begin K2_phi<=l_phid; K2_theta<=l_thd; K2_psi<=l_psid; K2_p<=l_Pd; K2_q<=l_Qd; K2_r<=l_Rd; end
                        2'd3: begin K3_phi<=l_phid; K3_theta<=l_thd; K3_psi<=l_psid; K3_p<=l_Pd; K3_q<=l_Qd; K3_r<=l_Rd; end
                    endcase
                    case (k_phase)
                        2'd0: begin k_phase <= 2'd1; st <= ST_K2_SETUP; end
                        2'd1: begin k_phase <= 2'd2; st <= ST_K3_SETUP; end
                        2'd2: begin k_phase <= 2'd3; st <= ST_K4_SETUP; end
                        2'd3: st <= ST_UPDATE_P1;
                    endcase
                end

                // ---- UPDATE P1: K0+2K1, K3+2K2 (6 axes, parallel) ----
                ST_UPDATE_P1: begin
                    up_phi_s0   <= K0_phi   + {K1_phi[30:0],   1'b0};
                    up_phi_s1   <= K3_phi   + {K2_phi[30:0],   1'b0};
                    up_theta_s0 <= K0_theta + {K1_theta[30:0], 1'b0};
                    up_theta_s1 <= K3_theta + {K2_theta[30:0], 1'b0};
                    up_psi_s0   <= K0_psi   + {K1_psi[30:0],   1'b0};
                    up_psi_s1   <= K3_psi   + {K2_psi[30:0],   1'b0};
                    up_p_s0     <= K0_p     + {K1_p[30:0],     1'b0};
                    up_p_s1     <= K3_p     + {K2_p[30:0],     1'b0};
                    up_q_s0     <= K0_q     + {K1_q[30:0],     1'b0};
                    up_q_s1     <= K3_q     + {K2_q[30:0],     1'b0};
                    up_r_s0     <= K0_r     + {K1_r[30:0],     1'b0};
                    up_r_s1     <= K3_r     + {K2_r[30:0],     1'b0};
                    st <= ST_UPDATE_P2;
                end

                // ---- UPDATE P2: sum = s0 + s1 ----
                ST_UPDATE_P2: begin
                    up_phi_sum   <= up_phi_s0   + up_phi_s1;
                    up_theta_sum <= up_theta_s0 + up_theta_s1;
                    up_psi_sum   <= up_psi_s0   + up_psi_s1;
                    up_p_sum     <= up_p_s0     + up_p_s1;
                    up_q_sum     <= up_q_s0     + up_q_s1;
                    up_r_sum     <= up_r_s0     + up_r_s1;
                    st <= ST_UPDATE_P3;
                end

                // ---- UPDATE P3: dt_mult = dt/6 * sum ----
                ST_UPDATE_P3: begin
                    up_phi_dt   <= fmul(`FP_DT_OVER_6, up_phi_sum);
                    up_theta_dt <= fmul(`FP_DT_OVER_6, up_theta_sum);
                    up_psi_dt   <= fmul(`FP_DT_OVER_6, up_psi_sum);
                    up_p_dt     <= fmul(`FP_DT_OVER_6, up_p_sum);
                    up_q_dt     <= fmul(`FP_DT_OVER_6, up_q_sum);
                    up_r_dt     <= fmul(`FP_DT_OVER_6, up_r_sum);
                    st <= ST_UPDATE_P4;
                end

                // ---- UPDATE P4: sv = x0 + dt_mult, clamp/wrap ----
                ST_UPDATE_P4: begin
                    sv_phi   <= fwrap(x0_phi   + up_phi_dt);
                    sv_theta <= fclamp(x0_theta + up_theta_dt,
                                       -`FP_PITCH_LIMIT, `FP_PITCH_LIMIT);
                    sv_psi   <= fwrap(x0_psi   + up_psi_dt);
                    sv_p     <= fclamp(x0_p     + up_p_dt,
                                       -`FP_RATE_LIMIT, `FP_RATE_LIMIT);
                    sv_q     <= fclamp(x0_q     + up_q_dt,
                                       -`FP_RATE_LIMIT, `FP_RATE_LIMIT);
                    sv_r     <= fclamp(x0_r     + up_r_dt,
                                       -`FP_RATE_LIMIT, `FP_RATE_LIMIT);
                    st <= ST_DONE_PULSE;
                end

                ST_DONE_PULSE: begin
                    done <= 1'b1;
                    st   <= ST_IDLE;
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

    assign state_roll  = sv_phi;
    assign state_pitch = sv_theta;
    assign state_yaw   = sv_psi;
    assign state_p     = sv_p;
    assign state_q     = sv_q;
    assign state_r     = sv_r;

    // ---- DEBUG TAP assigns ----
    assign dbg_st     = {26'd0, st};
    assign dbg_kphase = k_phase;
    assign dbg_rhs0   = r4_rhs0;
    assign dbg_rhs1   = r4_rhs1;
    assign dbg_rhs2   = r4_rhs2;
    assign dbg_n0     = l_n0;
    assign dbg_n1     = l_n1;
    assign dbg_n2     = l_n2;
    assign dbg_det    = l_det;
    assign dbg_Pd     = l_Pd;
    assign dbg_Qd     = l_Qd;
    assign dbg_Rd     = l_Rd;

endmodule
