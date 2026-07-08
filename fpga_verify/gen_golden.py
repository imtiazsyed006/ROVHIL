"""
gen_golden.py -- Generate stimulus + golden reference vectors for FPGA co-sim,
verify LUT .mem files against the Python (xlsx) LUT, and compute exact Q16.16
constants for rov_params.vh.

Outputs (in fpga_verify/):
  duty_stim.hex   : 8 x 16-bit duty (pwm_ctrl counts) per sample, one word/line
  tau_ref.csv     : Python tau_fit (N*m) per sample
  y_ref.csv       : FPGA-aligned Python reference states per sample
                    (y0 = 0, step k uses tau[k-1], tau[k] with tau[-1] = 0, dt = 0.02)
  constants.txt   : exact Q16.16 constants table for rov_params.vh
  fwd_lut_ref.mem / rev_lut_ref.mem : LUTs regenerated from the xlsx
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import simulation as S
from ThrusterConfigMatrix import (
    build_thruster_config_matrix,
    load_t200_14v_lut,
    map_u_to_pwm_t200_14v,
    wrench_to_thruster_commands,
)

OUT = Path(__file__).resolve().parent


def q16(x):
    """float -> signed Q16.16 integer, round to nearest (matches golden math best)."""
    v = int(np.floor(x * 65536.0 + 0.5))
    assert -2**31 <= v < 2**31, x
    return v


def q16h(x):
    v = q16(x) & 0xFFFFFFFF
    return f"{v:08X}"


def main():
    # ------------------------------------------------------------------
    # 1. Load experiment + build tau exactly like simulation.py main()
    # ------------------------------------------------------------------
    t_arr, z_arr, u_arr = S.load_pqr_identification_data(
        ROOT / "Experiment.mat", duration_s=None, zero_reference=False)
    print(f"samples: {len(t_arr)}")
    dt = np.diff(t_arr)
    print(f"dt: min={dt.min():.6f} max={dt.max():.6f} mean={dt.mean():.6f} "
          f"median={np.median(dt):.6f}")
    print(f"initial z (rpy pqr): {z_arr[0]}")
    print(f"u_arr[:, 3:6] range: {u_arr[:, 3:6].min(axis=0)} .. {u_arr[:, 3:6].max(axis=0)}")

    lut = load_t200_14v_lut(str(ROOT / "T200-Public-Performance-Data-10-20V-September-2019.xlsx"))
    b_matrix = build_thruster_config_matrix()

    # tau exactly as simulation.py (thruster_config pipeline)
    tau_fit = S.thruster_config_tau_history(u_arr, 1.0, S.DEFAULT_INPUT_SIGNS)
    print(f"tau_fit range: {tau_fit.min(axis=0)} .. {tau_fit.max(axis=0)}")

    # ------------------------------------------------------------------
    # 2. Build the duty stimulus the FPGA would measure (STM32 replica)
    #    cmd -> signs -> wrench -> pinv -> u_sat -> per-ch negation (all but
    #    ch index 5) -> map_u_to_pwm -> integer pwm_ctrl counts (CCR)
    # ------------------------------------------------------------------
    u_signed = S.apply_input_signs(u_arr, S.DEFAULT_INPUT_SIGNS)
    moment_cmd = u_signed[:, 3:6]
    wrench_cmd = np.zeros((len(u_arr), 6))
    wrench_cmd[:, 3:6] = moment_cmd
    _, u_sat = wrench_to_thruster_commands(wrench_cmd, b_matrix=b_matrix)

    stm32_signs = -np.ones(8)
    stm32_signs[5] = 1.0                      # ch6 (index 5) not negated
    u_phys = u_sat * stm32_signs[None, :]
    pwm_ctrl = map_u_to_pwm_t200_14v(u_phys, lut)
    duty = np.rint(pwm_ctrl).astype(int)      # integer CCR counts 4400..7600
    print(f"duty range: {duty.min()} .. {duty.max()}")

    with (OUT / "duty_stim.hex").open("w") as f:
        for row in duty:
            for d in row:
                f.write(f"{d & 0xFFFF:04X}\n")

    np.savetxt(OUT / "tau_ref.csv", tau_fit, delimiter=",",
               header="tau_roll,tau_pitch,tau_yaw", comments="")

    # ------------------------------------------------------------------
    # 3. FPGA-aligned Python reference trajectory
    #    y[-1] = 0, step k uses (tau[k-1], tau[k]), tau[-1] = 0, dt = 0.02
    # ------------------------------------------------------------------
    pvec = S.normalize_pqr_param_vec(S.DEFAULT_HYDRO_PARAMS)
    n = len(t_arr)
    y_ref = np.zeros((n, 6))
    state = np.zeros(6)
    tau_prev = np.zeros(3)
    for k in range(n):
        state = S.attitude_pqr_step(state, tau_prev, tau_fit[k], 0.02, pvec)
        tau_prev = tau_fit[k]
        y_ref[k] = state
    np.savetxt(OUT / "y_ref.csv", y_ref, delimiter=",",
               header="phi,theta,psi,p,q,r", comments="")
    print(f"y_ref rate range (deg/s): {np.rad2deg(y_ref[:, 3:6]).min(axis=0)} .. "
          f"{np.rad2deg(y_ref[:, 3:6]).max(axis=0)}")

    # ------------------------------------------------------------------
    # 4. LUT verification: rebuild fwd/rev .mem from the xlsx
    #    fwd: pwm 1536:4:1900 force kgf; rev: pwm 1100:4:1464 |force| kgf
    # ------------------------------------------------------------------
    pwm_lut, force_lut = lut[:, 0], lut[:, 1]
    ux, uy = np.unique(pwm_lut, return_index=True)
    pwm_u, force_u = pwm_lut[np.sort(uy)], force_lut[np.sort(uy)]

    fwd_pwm = np.arange(1536, 1901, 4)
    rev_pwm = np.arange(1100, 1465, 4)
    fwd_force = np.interp(fwd_pwm, pwm_u, force_u)
    rev_force = np.abs(np.interp(rev_pwm, pwm_u, force_u))

    max_fwd = force_lut[np.argmax(pwm_lut)]
    max_rev = abs(force_lut[np.argmin(pwm_lut)])
    print(f"max_fwd={max_fwd!r}  max_rev={max_rev!r}")
    print(f"(simulation.py constants: fwd={S.MAX_FWD_FORCE_N / S.KGF_TO_N!r}, "
          f"rev={S.MAX_REV_FORCE_N / S.KGF_TO_N!r})")

    def write_mem(path, vals):
        with path.open("w") as f:
            for v in vals:
                f.write(f"{q16(v) & 0xFFFFFFFF:08X}\n")

    write_mem(OUT / "fwd_lut_ref.mem", fwd_force)
    write_mem(OUT / "rev_lut_ref.mem", rev_force)

    for name, ref in (("fwd_lut.mem", fwd_force), ("rev_lut.mem", rev_force)):
        cur = [int(l, 16) for l in
               (ROOT / "MeasurePWM.srcs/sources_1/new" / name).read_text().split()]
        refq = [q16(v) & 0xFFFFFFFF for v in ref]
        if len(cur) != len(refq):
            print(f"{name}: LENGTH MISMATCH {len(cur)} vs {len(refq)}")
        else:
            diffs = [(i, c, r) for i, (c, r) in enumerate(zip(cur, refq)) if c != r]
            mx = max((abs(((c ^ 0) - (1 << 32) if c >= 2**31 else c) -
                          ((r - (1 << 32)) if r >= 2**31 else r)) for _, c, r in diffs),
                     default=0)
            print(f"{name}: {len(diffs)}/{len(cur)} entries differ, max |diff| = {mx} LSB")
            for i, c, r in diffs[:5]:
                print(f"   [{i}] cur={c:08X} ref={r:08X}")

    # ------------------------------------------------------------------
    # 5. Exact Q16.16 constants for rov_params.vh
    # ------------------------------------------------------------------
    P = S.ROVParams
    names = S.PQR_PARAM_NAMES
    W = P.m * 9.81

    const = {
        "FP_MASS": P.m, "FP_GRAVITY": 9.81, "FP_WEIGHT": W,
        "FP_I_X": P.I_x, "FP_I_Y": P.I_y, "FP_I_Z": P.I_z,
        "FP_I_XY": P.I_xy, "FP_I_XZ": P.I_xz, "FP_I_YZ": P.I_yz,
        "FP_X_B": P.x_B, "FP_Y_B": P.y_B, "FP_Z_B": P.z_B,
        "FP_WY_B": W * P.y_B, "FP_WZ_B": W * P.z_B, "FP_WX_B": W * P.x_B,
    }
    for nm, v in zip(names, pvec):
        const[f"FP_{nm}"] = v
    const.update({
        "FP_RATE_LIMIT": np.deg2rad(500.0),
        "FP_PITCH_LIMIT": np.deg2rad(89.0),
        "FP_ACCEL_CLIP": 80.0,
        "FP_EULER_CLIP": 2000.0,
        "FP_PI": np.pi, "FP_TWO_PI": 2 * np.pi,
        "FP_MAX_FWD_FORCE": max_fwd, "FP_MAX_REV_FORCE": max_rev,
        "FP_MAX_FWD_N": max_fwd * S.KGF_TO_N,
        "FP_MAX_REV_N": max_rev * S.KGF_TO_N,
        "FP_G_KGFM_TO_NM": S.KGF_TO_N,
        "CORDIC_SCALE(1/K16)": 0.6072529350088813,
    })
    with (OUT / "constants.txt").open("w") as f:
        for k, v in const.items():
            q = q16(v)
            f.write(f"{k:24s} = {v!r:26} -> {q:>10d}  (0x{q & 0xFFFFFFFF:08X})\n")
    print("\nconstants.txt written")

    # Q16.20 dt constants (for the higher-precision fmul20 path)
    for nm, v in (("FP_DT_Q20", 0.02), ("FP_HALF_DT_Q20", 0.01),
                  ("FP_DT_OVER_6_Q20", 0.02 / 6.0)):
        q = int(np.floor(v * 2**20 + 0.5))
        print(f"{nm:20s} = {v:.10f} -> {q} (0x{q:08X})")


if __name__ == "__main__":
    main()
