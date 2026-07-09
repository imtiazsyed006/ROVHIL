//----------------------------------------------------------------------
// sensor_params.vh -- Q16.16 constants for the AHRS/IMU sensor model
// Derived from rov_sensor.py SensorModel(seed=7, dt=0.02) so the FPGA
// matches the Python defaults used by the SMC feedback path.
//
// Format: signed Q16.16  (value * 65536, round-to-nearest)
//----------------------------------------------------------------------
`ifndef SENSOR_PARAMS_VH
`define SENSOR_PARAMS_VH

// ---- Rate (gyro) channel ----
// White-noise std ≈ 4.36e-5 rad/s  (density*sqrt(Nyquist), dt=0.02)
`define SP_RATE_NOISE_STD       32'sd3            // 4.58e-5 rad/s
// Turn-on bias [p,q,r]. Zeroed for a clean grounded start (rates ~0 at rest).
// Seed=7 draws were ~+0.085 / -0.023 / +0.002 deg/s.
`define SP_RATE_BIAS_P          32'sd0
`define SP_RATE_BIAS_Q          32'sd0
`define SP_RATE_BIAS_R          32'sd0
// Bias random-walk step std (instability * sqrt(dt))
`define SP_BIAS_DRIFT_STD       32'sd1            // 1.23e-5 rad/s  (~1 LSB)
// Temperature bias offset at T=ref (dT=0) -- all zero
`define SP_BIAS_TEMP_P          32'sd0
`define SP_BIAS_TEMP_Q          32'sd0
`define SP_BIAS_TEMP_R          32'sd0
// Scale factors (turn-on tol * temp)
`define SP_RATE_SF_P            32'sd65506        // 0.999537
`define SP_RATE_SF_Q            32'sd65536        // 1.000000
`define SP_RATE_SF_R            32'sd65559        // 1.000358
// Cross-axis matrix (row-major). Diagonal forced to 1.0
`define SP_CA_00                32'sd65536        //  1.0
`define SP_CA_01                32'shFFFFFEC8     // -0.004762
`define SP_CA_02                32'shFFFFFB8E     // -0.017363
`define SP_CA_10                32'shFFFFFDD5     // -0.008474
`define SP_CA_11                32'sd65536        //  1.0
`define SP_CA_12                32'shFFFFFD11     // -0.011465
`define SP_CA_20                32'shFFFFFF82     // -0.001915
`define SP_CA_21                32'sd1130         //  0.017248
`define SP_CA_22                32'sd65536        //  1.0
// Nonlinearity fraction of FS (0.2 % FS = 0.002)
`define SP_NL_FS                32'sd131          // 0.002  (0x83)
// Full-scale / saturation 2000 deg/s = 34.9066 rad/s
`define SP_RATE_FS              32'sd2287638      // 34.9066
`define SP_RATE_SAT             32'sd2287638
// Rate quantization step 0.001 deg/s
`define SP_RATE_QUANT           32'sd1            // 1.75e-5 rad/s

// ---- Attitude (EKF) channel ----
// Static noise 1-sigma (1,1,3) deg
`define SP_ATT_NOISE_ST_R       32'sd1144         // 0.01745 rad
`define SP_ATT_NOISE_ST_P       32'sd1144
`define SP_ATT_NOISE_ST_Y       32'sd3431         // 0.05236 rad
// Dynamic noise 1-sigma (3,3,5) deg
`define SP_ATT_NOISE_DY_R       32'sd3431
`define SP_ATT_NOISE_DY_P       32'sd3431
`define SP_ATT_NOISE_DY_Y       32'sd5719         // 0.08727 rad
// Dynamic-rate threshold 5 deg/s
`define SP_DYN_RATE_THR         32'sd5719         // 0.08727 rad/s
// Attitude bias (repeatability). Zeroed so a grounded ROV starts at RPY≈0
// when the plant state is 0; enable non-zero values to mimic AHRS offset.
// (Previous seed=7 draws were ~0.20 / -0.39 / 0.001 deg.)
`define SP_ATT_BIAS_R           32'sd0
`define SP_ATT_BIAS_P           32'sd0
`define SP_ATT_BIAS_Y           32'sd0
// Attitude quantization 0.01 deg
`define SP_ATT_QUANT            32'sd11           // 0.000175 rad

// ---- Common ----
`define SP_ONE                  32'sd65536        // 1.0
`define SP_ONE_QUARTER          32'sd16384        // 0.25  (MA window = 4)
`define SP_SQRT3                32'sd113512       // 1.73205  (U[-a,a] -> N(0,1) scale a=σ√3)

// LFSR seed (matches SensorParams.seed = 7; non-zero)
`define SP_LFSR_SEED            32'h00000007

`endif
