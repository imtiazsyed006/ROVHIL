//----------------------------------------------------------------------
// rov_params.vh -- ROV hydrodynamic & physical parameters in Q16.16
// Updated 2026-07-08 to match Python model parameters exactly.
//----------------------------------------------------------------------
`ifndef ROV_PARAMS_VH
`define ROV_PARAMS_VH

// Fixed-point format: 32-bit signed Q16.16
// Multiply:  (a * b) >>> 16
// To convert float f: $signed({f * 65536.0})

// --- Physical constants (Q16.16) ---
`define FP_MASS             32'sd1012859     // 15.455
`define FP_GRAVITY          32'sd642908      // 9.81
`define FP_WEIGHT           32'sd9929461     // 151.51155 N

// --- Inertia matrix (Q16.16) ---
`define FP_I_X              32'sd15678       // 0.239233974
`define FP_I_Y              32'sd34046       // 0.519526935
`define FP_I_Z              32'sd34003       // 0.518847330
`define FP_I_XY             32'shFFFFFFFB    // -0.000075948  ≈ -5
`define FP_I_XZ             32'sd808         // 0.012328056
`define FP_I_YZ             32'sd1           // 0.000015703  ≈ 1

// CG / CB offsets (metres, Q16.16)
`define FP_X_G              32'sd0
`define FP_Y_G              32'sd0
`define FP_Z_G              32'sd0
`define FP_X_B              32'sd8           // 0.000114505
`define FP_Y_B              32'sd2           // 0.000026475
`define FP_Z_B              32'shFFFFFF79    // -0.002057619 ≈ -135

// --- Restoring-moment products: W*offset (Q16.16) ---
`define FP_WY_B             32'sd263         // W*Y_B = 0.004011
`define FP_WZ_B             32'shFFFFB033    // W*Z_B = -0.311784  → -20433
`define FP_WX_B             32'sd1137        // W*X_B = 0.017349

// --- Added mass (Q16.16) ---
`define FP_K_dp             32'sd26343       // 0.40199742
`define FP_M_dq             32'sd72111       // 1.10076860
`define FP_N_dr             32'sd35776       // 0.54605686
`define FP_K_dq             32'sd4596        // 0.070128856
`define FP_K_dr             32'sd35251       // 0.53790016
`define FP_M_dr             32'sd8234        // 0.12565792
`define FP_M_dp             32'sd1845        // 0.02815617
`define FP_N_dp             32'sd2363        // 0.036057289
`define FP_N_dq             32'sd9914        // 0.15129341

// --- Linear damping (Q16.16) ---
`define FP_K_p              32'sd119851      // 1.8287636
`define FP_K_q              32'sd11896       // 0.18154419
`define FP_K_r              32'sd12494       // 0.19065204
`define FP_M_p              32'sd9948        // 0.15180295
`define FP_M_q              32'sd68264       // 1.0416938
`define FP_M_r              32'sd20772       // 0.31699777
`define FP_N_p              32'sd5260        // 0.080278963
`define FP_N_q              32'sd9722        // 0.14837128
`define FP_N_r              32'sd16389       // 0.25009818

// --- Quadratic damping (Q16.16) ---
`define FP_K_pp             32'sd23943       // 0.36539232
`define FP_K_qq             32'sd10241       // 0.15628606
`define FP_K_rr             32'sd20856       // 0.31827390
`define FP_M_pp             32'sd8596        // 0.13117582
`define FP_M_qq             32'sd151414      // 2.3112304
`define FP_M_rr             32'sd19345       // 0.29520896
`define FP_N_pp             32'sd4739        // 0.072323605
`define FP_N_qq             32'sd10685       // 0.16307606
`define FP_N_rr             32'sd125526      // 1.9157063

// --- Torque input gains (Q16.16) ---
`define FP_G_p              32'sd82587       // 1.2603648
`define FP_G_q              32'sd103320      // 1.57676861
`define FP_G_r              32'sd56881       // 0.86806303

// --- Limits (Q16.16) ---
`define FP_RATE_LIMIT       32'sd6553600     // 100.0 rad/s
`define FP_PITCH_LIMIT      32'sd101700      // 1.553 rad  (89 deg)
`define FP_ACCEL_CLIP       32'sd5242880     // 80.0

// --- PI (Q16.16) ---
`define FP_PI               32'sd205887      // 3.14159265
`define FP_TWO_PI           32'sd411775      // 6.2831853
`define FP_PI_OVER_180      32'sd1144        // 0.0174533
`define FP_180_OVER_PI      32'sd3754942     // 57.29578

// --- Simulation timing ---
`define FP_DT               32'sd1311        // 0.02
`define FP_HALF_DT          32'sd655         // 0.01
`define FP_DT_OVER_6        32'sd218         // 0.02/6 = 0.003333
`define FP_TWO               32'sd131072     // 2.0

// --- Step torque values (Q16.16) ---
`define FP_STEP_TORQUE      32'sd655360      // 10.0 Nm

// --- Timing constants (at 100 MHz) ---
`define CYCLES_PER_STEP     2000000          // 0.02s * 100MHz
`define STEPS_PER_PHASE     250              // 5.0s / 0.02s

//==================================================================
// PWM-input -> torque allocation pipeline constants
//==================================================================
// Allocation matrix B rows for Mx, My, Mz (signed Q16.16).
// tau = B @ u (u in kgf) -> kgf*m, then x9.80665 -> N*m.

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

// LUT range (duty 44%..76% -> PWM 1100..1900us, 4us step, 201 entries)
`define PWM_MIN_US  32'sd1100
`define PWM_STEP_US 32'sd4
`define LUT_ENTRIES 201

// --- Dual thrust-curve LUTs (MATLAB thruster_tables) ---
// Forward: FWD_PWM_US 1536:4:1900 (92 entries), force in Q16.16 kgf
`define FWD_LUT_ENTRIES 92
`define FWD_PWM_START   12'd1536
`define FP_MAX_FWD_FORCE 32'sd296473   // 4.52382421 × 65536

// Reverse: REV_PWM_US 1100:4:1464 (92 entries), force magnitudes in Q16.16
`define REV_LUT_ENTRIES 92
`define REV_PWM_START   12'd1100
`define REV_PWM_END     12'd1464
`define FP_MAX_REV_FORCE 32'sd230480   // 3.51684997 × 65536

`endif
