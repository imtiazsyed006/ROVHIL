//----------------------------------------------------------------------
// rov_params.vh -- ROV hydrodynamic & physical parameters in Q16.16
// Regenerated 2026-07-08 by fpga_verify/gen_golden.py: every constant is
// round-to-nearest(value * 65536) so it matches simulation.py exactly.
//----------------------------------------------------------------------
`ifndef ROV_PARAMS_VH
`define ROV_PARAMS_VH

// Fixed-point format: 32-bit signed Q16.16
// Multiply:  (a * b) >>> 16
// To convert float f: round(f * 65536.0)

// --- Physical constants (Q16.16) ---
`define FP_MASS             32'sd1012859     // 15.455
`define FP_GRAVITY          32'sd642908      // 9.81
`define FP_WEIGHT           32'sd9936146     // 151.61355 N  (15.455 * 9.81)

// --- Inertia matrix (Q16.16) ---
`define FP_I_X              32'sd15678       // 0.239233974
`define FP_I_Y              32'sd34048       // 0.519526935
`define FP_I_Z              32'sd34003       // 0.518847330
`define FP_I_XY             32'shFFFFFFFB    // -0.000075948 = -5
`define FP_I_XZ             32'sd808         // 0.012328056
`define FP_I_YZ             32'sd1           // 0.000015703

// CG / CB offsets (metres, Q16.16)
`define FP_X_G              32'sd0
`define FP_Y_G              32'sd0
`define FP_Z_G              32'sd0
`define FP_X_B              32'sd8           // 0.000114505
`define FP_Y_B              32'sd2           // 0.000026475
`define FP_Z_B              32'shFFFFFF79    // -0.002057619 = -135

// --- Restoring-moment products: W*offset (Q16.16), W = 151.61355 N ---
// NOTE: W*x_B produced a constant pitch torque even at level attitude
// (g5 = -W*x_B at phi=theta=0), so a "grounded / zero-input" ROV slowly
// walked away from RPY=0.  Horizontal CB levers are zeroed so the only
// restoring is the vertical W*z_B term (roll/pitch stability about level).
// Original: WY_B=263, WX_B=1138, WZ_B=-20445.
`define FP_WY_B             32'sd0           // was 263  (W*y_B) — zeroed for rest@0
`define FP_WZ_B             32'shFFFFB023    // W*z_B = -0.31196290 = -20445
`define FP_WX_B             32'sd0           // was 1138 (W*x_B) — zeroed for rest@0

// --- Added mass (Q16.16) ---
`define FP_K_dp             32'sd26345       // 0.40199742
`define FP_M_dq             32'sd72140       // 1.10076860
`define FP_N_dr             32'sd35786       // 0.54605686
`define FP_K_dq             32'sd4596        // 0.070128856
`define FP_K_dr             32'sd35252       // 0.53790016
`define FP_M_dr             32'sd8235        // 0.12565792
`define FP_M_dp             32'sd1845        // 0.02815617
`define FP_N_dp             32'sd2363        // 0.036057289
`define FP_N_dq             32'sd9915        // 0.15129341

// --- Linear damping (Q16.16) ---
`define FP_K_p              32'sd119850      // 1.8287636
`define FP_K_q              32'sd11898       // 0.18154419
`define FP_K_r              32'sd12495       // 0.19065204
`define FP_M_p              32'sd9949        // 0.15180295
`define FP_M_q              32'sd68268       // 1.0416938
`define FP_M_r              32'sd20775       // 0.31699777
`define FP_N_p              32'sd5261        // 0.080278963
`define FP_N_q              32'sd9724        // 0.14837128
`define FP_N_r              32'sd16390       // 0.25009818

// --- Quadratic damping (Q16.16) ---
`define FP_K_pp             32'sd23946       // 0.36539232
`define FP_K_qq             32'sd10242       // 0.15628606
`define FP_K_rr             32'sd20858       // 0.31827390
`define FP_M_pp             32'sd8597        // 0.13117582
`define FP_M_qq             32'sd151469      // 2.3112304
`define FP_M_rr             32'sd19347       // 0.29520896
`define FP_N_pp             32'sd4740        // 0.072323605
`define FP_N_qq             32'sd10687       // 0.16307606
`define FP_N_rr             32'sd125548      // 1.9157063

// --- Torque input gains (Q16.16) ---
`define FP_G_p              32'sd82599       // 1.2603648
`define FP_G_q              32'sd103335      // 1.57676861
`define FP_G_r              32'sd56889       // 0.86806303

// --- Limits (Q16.16) ---
// RATE_LIMIT matches Python RATE_LIMIT_RAD_S = deg2rad(500) = 8.72665 rad/s
`define FP_RATE_LIMIT       32'sd571909      // 8.7266463 rad/s (500 deg/s)
`define FP_PITCH_LIMIT      32'sd101800      // 1.5533430 rad (89 deg)
`define FP_ACCEL_CLIP       32'sd5242880     // 80.0 rad/s^2 (Python clips w_dot)
// Python does NOT clip Euler angle rates; this bound exists only to keep the
// fixed-point intermediates (x0 + dt*K) inside the CORDIC convergence range.
// It can only bind for |theta| > 87 deg, which the 89-deg pitch clamp makes
// a pathological corner.
`define FP_EULER_CLIP       32'sd9830400     // 150.0 rad/s

// --- PI (Q16.16) ---
`define FP_PI               32'sd205887      // 3.14159265
`define FP_TWO_PI           32'sd411775      // 6.2831853
`define FP_PI_OVER_180      32'sd1144        // 0.0174533
`define FP_180_OVER_PI      32'sd3754936     // 57.29578

// --- Simulation timing ---
// dt constants in Q12.20 (used with fmul20) for 16x less rounding error than
// Q16.16: dt/6 in Q16.16 (218) is 0.21% low; in Q12.20 (3495) only 0.007%.
`define FP_DT_Q20           32'sd20972       // 0.02      * 2^20
`define FP_HALF_DT_Q20      32'sd10486       // 0.01      * 2^20
`define FP_DT_OVER_6_Q20    32'sd3495        // 0.02/6    * 2^20
// Legacy Q16.16 dt constants (kept for reference / other users)
`define FP_DT               32'sd1311        // 0.02
`define FP_HALF_DT          32'sd655         // 0.01
`define FP_DT_OVER_6        32'sd218         // 0.02/6
`define FP_TWO              32'sd131072      // 2.0

// --- Step torque values (Q16.16) ---
`define FP_STEP_TORQUE      32'sd655360      // 10.0 Nm

// --- Timing constants (at 100 MHz) ---
`define CYCLES_PER_STEP     2000000          // 0.02s * 100MHz
`define STEPS_PER_PHASE     250              // 5.0s / 0.02s

//==================================================================
// PWM-input -> torque allocation pipeline constants
//==================================================================
// Allocation matrix B moment rows (Mx, My, Mz), signed Q16.16.
// tau_Nm = B_row . f_N  where f_N = u_norm * (u>0 ? MAX_FWD_N : MAX_REV_N).

// MX: -0.127553  +0.129782  -0.129807  +0.127528  +0.133844  -0.131566  +0.131591  -0.133819
`define FP_B_MX0 32'shFFFFDF59
`define FP_B_MX1 32'sh00002139
`define FP_B_MX2 32'shFFFFDEC5
`define FP_B_MX3 32'sh000020A6
`define FP_B_MX4 32'sh00002244
`define FP_B_MX5 32'shFFFFDE52
`define FP_B_MX6 32'sh000021B0
`define FP_B_MX7 32'shFFFFDDBE

// MY: +0.072740  +0.070486  -0.076788  -0.079042  -0.066448  -0.068702  +0.075004  +0.072750
`define FP_B_MY0 32'sh0000129F
`define FP_B_MY1 32'sh0000120B
`define FP_B_MY2 32'shFFFFEC58
`define FP_B_MY3 32'shFFFFEBC4
`define FP_B_MY4 32'shFFFFEEFD
`define FP_B_MY5 32'shFFFFEE6A
`define FP_B_MY6 32'sh00001333
`define FP_B_MY7 32'sh000012A0

// MZ: -0.144575  +0.144557  +0.149124  -0.149106  -0.144575  +0.144557  +0.149124  -0.149106
`define FP_B_MZ0 32'shFFFFDAFD
`define FP_B_MZ1 32'sh00002502
`define FP_B_MZ2 32'sh0000262D
`define FP_B_MZ3 32'shFFFFD9D4
`define FP_B_MZ4 32'shFFFFDAFD
`define FP_B_MZ5 32'sh00002502
`define FP_B_MZ6 32'sh0000262D
`define FP_B_MZ7 32'shFFFFD9D4

// kgf*m -> N*m conversion (Q16.16)
`define FP_G_KGFM_TO_NM 32'sh0009CE81  // 9.80665

// Normalized-u -> thruster force in NEWTONS (Q16.16).
// Python: f_N = u * max_force_kgf * 9.80665 (per sign of u), so the moments
// out of tau_alloc are in physical N*m exactly like wrench_to_net_wrench.
`define FP_MAX_FWD_N 32'sd2907410      // 4.52382421 * 9.80665 = 44.363561
`define FP_MAX_REV_N 32'sd2260239      // 3.51684997 * 9.80665 = 34.488517

// LUT range (duty 44%..76% -> PWM 1100..1900us, 4us step, 201 entries)
`define PWM_MIN_US  32'sd1100
`define PWM_STEP_US 32'sd4
`define LUT_ENTRIES 201

// --- Dual thrust-curve LUTs (from T200 14V xlsx, identical to Python LUT) ---
// Forward: FWD_PWM_US 1536:4:1900 (92 entries), force in Q16.16 kgf
`define FWD_LUT_ENTRIES 92
`define FWD_PWM_START   12'd1536
`define FP_MAX_FWD_FORCE 32'sd296473   // 4.52382421 * 65536

// Reverse: REV_PWM_US 1100:4:1464 (92 entries), force magnitudes in Q16.16
`define REV_LUT_ENTRIES 92
`define REV_PWM_START   12'd1100
`define REV_PWM_END     12'd1464
`define FP_MAX_REV_FORCE 32'sd230480   // 3.51684997 * 65536

`endif
