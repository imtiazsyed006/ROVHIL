"""
Self-contained ROV rotational dynamics simulation — identical input-processing
pipeline as the GUI's "Simulate Free Run" (thruster_config tau source).

Loads command signals from Experiment.mat, runs them through the thruster
allocation + T200 lookup table, simulates the plant, and compares simulated
angular velocities (p, q, r) against the measured ones.

No imports from estimate_tf_from_telemetry — everything needed is in this file
(or the ThrusterConfigMatrix data module for the LUT / geometry).
"""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

# ── Thruster geometry + LUT (data module, not a processing script) ──
from ThrusterConfigMatrix import (
    build_thruster_config_matrix,
    load_t200_14v_lut,
    wrench_to_net_wrench_t200_14v,
)


# =============================================================================
# Constants
# =============================================================================
DEFAULT_INPUT_SIGNS = np.array([1.0, 1.0, 1.0], dtype=float)
DEFAULT_THRUSTER_CONFIG_SCALE = 1.0
RATE_LIMIT_RAD_S = np.deg2rad(500.0)
KGF_TO_N = 9.80665
MAX_REV_FORCE_N = 3.51684997 * KGF_TO_N
MAX_FWD_FORCE_N = 4.52382421 * KGF_TO_N
THRUSTER_DEADZONE = 0.03

# =============================================================================
# ROV Physical Parameters
# =============================================================================
class ROVParams:
    m = 15.185 + 0.270
    I_x = 0.23923397394621614
    I_y = 0.5195269346926577
    I_z = 0.5188473299545998
    I_xy = -0.00007594827841912
    I_xz = 0.01232805555434198
    I_yz = 0.00001570289016570
    W = m * 9.81
    B = W
    x_g, y_g, z_g = 0.0, 0.0, 0.0
    x_B = 0.11450509 / 1000.0
    y_B = 0.02647538 / 1000.0
    z_B = -2.05761889 / 1000.0


# =============================================================================
# Default hydrodynamic parameters (PQR_UKF_P0)
# =============================================================================
PQR_PARAM_NAMES = [
    "K_dp", "M_dq", "N_dr", "K_dq", "K_dr", "M_dr", "M_dp", "N_dp", "N_dq",
    "K_p", "K_q", "K_r", "M_p", "M_q", "M_r", "N_p", "N_q", "N_r",
    "K_pp", "K_qq", "K_rr", "M_pp", "M_qq", "M_rr", "N_pp", "N_qq", "N_rr",
    "G_p", "G_q", "G_r",
]

DEFAULT_HYDRO_PARAMS = np.array([
    0.40199742, 1.1007686, 0.54605686, 0.070128856, 0.53790016, 0.12565792,
    0.02815617, 0.036057289, 0.15129341, 1.8287636, 0.18154419, 0.19065204,
    0.15180295, 1.0416938, 0.31699777, 0.080278963, 0.14837128, 0.25009818,
    0.36539232, 0.15628606, 0.3182739, 0.13117582, 2.3112304, 0.29520896,
    0.072323605, 0.16307606, 1.9157063, 1.2603648, 1.57676861, 0.86806303,
], dtype=float)

PQR_PARAM_LB = np.array([0.0] * 27 + [0.25] * 3, dtype=float)
PQR_PARAM_UB = np.array([
    5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
    5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
    8.0, 8.0, 8.0,
], dtype=float)


# =============================================================================
# Thruster allocation matrices and T200 dynamics (from firmware)
# =============================================================================
PINVB = np.array([
    [0.24271643, -0.25128761, -0.18634628, -0.95648975,  1.71833348, -0.85126335],
    [0.23847872,  0.25895483, -0.18637998,  0.95649104,  1.71833138,  0.85126425],
    [-0.23844825, -0.25128793, -0.17087066, -0.95649089, -1.71833128,  0.85126227],
    [-0.24274546,  0.24362219, -0.17090438,  0.95649147, -1.71832989, -0.85126318],
    [0.25218525, -0.25130852,  0.18634601,  0.95648820, -1.71833096, -0.85126437],
    [0.25648243,  0.24364279,  0.18637971, -0.95648956, -1.71832888,  0.85126518],
    [-0.25645196, -0.23597588,  0.17087041,  0.95648970,  1.71832898,  0.85126372],
    [-0.25221424,  0.24364306,  0.17090413, -0.95649034,  1.71832760, -0.85126454],
], dtype=float)

B_ALLOC = np.array([
    [0.50512188,  0.50512137, -0.50512039, -0.50511988,  0.50512258,  0.50512207, -0.50512109, -0.50512058],
    [-0.50511988,  0.50512137, -0.50512039,  0.50512188, -0.50512058,  0.50512207, -0.50512109,  0.50512258],
    [-0.69978983, -0.69978912, -0.69979053, -0.69978983,  0.69978881,  0.69978810,  0.69978951,  0.69978881],
    [-0.12755275,  0.12978174, -0.12980658,  0.12752825,  0.13384403, -0.13156576,  0.13159052, -0.13381948],
    [0.07273965,  0.07048591, -0.07678842, -0.07904200, -0.06644809, -0.06870164,  0.07500414,  0.07275045],
    [-0.14457473,  0.14455708,  0.14912369, -0.14910605, -0.14457496,  0.14455731,  0.14912387, -0.14910623],
], dtype=float)

# T200 transfer function (DC gain = 1)
T200_NUM = np.array([-107.0, 83579.0, 39886.0], dtype=float)
T200_DEN = np.array([200.0, 30400.0, 2929600.0, 1156800.0], dtype=float)
T200_DC_GAIN = T200_NUM[-1] / T200_DEN[-1]
T200_NUM_NORM = T200_NUM / T200_DC_GAIN

_den_monic = T200_DEN / T200_DEN[0]
_a2 = _den_monic[1]; _a1 = _den_monic[2]; _a0 = _den_monic[3]
_num_monic = T200_NUM_NORM / T200_DEN[0]
_b2 = _num_monic[0]; _b1 = _num_monic[1]; _b0 = _num_monic[2]

T200_A = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-_a0, -_a1, -_a2]], dtype=float)
T200_B = np.array([0.0, 0.0, 1.0], dtype=float)
T200_C = np.array([_b0, _b1, _b2], dtype=float)


# =============================================================================
# Helpers
# =============================================================================
def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


def to_float(value):
    if value is None:
        return math.nan
    s = str(value).strip()
    if s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def normalize_pqr_param_vec(pvec):
    arr = np.asarray(pvec, dtype=float).reshape(-1)
    if arr.size == 30:
        pass
    elif arr.size == 27:
        arr = np.concatenate([arr, np.ones(3, dtype=float)])
    elif arr.size == 9:
        arr = np.array([
            arr[0], arr[1], arr[2],
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            arr[3], 0.0, 0.0, 0.0, arr[4], 0.0, 0.0, 0.0, arr[5],
            arr[6], 0.0, 0.0, 0.0, arr[7], 0.0, 0.0, 0.0, arr[8],
            1.0, 1.0, 1.0,
        ], dtype=float)
    else:
        raise ValueError(f"Unexpected parameter vector size: {arr.size}")
    return np.clip(arr, PQR_PARAM_LB, PQR_PARAM_UB)


# =============================================================================
# Telemetry data loading
# =============================================================================
def read_telemetry(path):
    rows = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({k: to_float(v) if k != "type" else (v or "") for k, v in r.items()})
    return rows


def build_mimo_data(rows):
    t_list, z_list, u_list = [], [], []
    for r in rows:
        typ = str(r.get("type", "")).strip().upper()
        t = r.get("t_s", math.nan)
        if not math.isfinite(t):
            continue
        if typ == "IMU":
            t_list.append(t)
            roll, pitch, yaw = r.get("roll", math.nan), r.get("pitch", math.nan), r.get("yaw", math.nan)
            gx, gy, gz = r.get("gx", math.nan), r.get("gy", math.nan), r.get("gz", math.nan)
            cmd_surge = r.get("cmd_surge", math.nan)
            cmd_sway = r.get("cmd_sway", math.nan)
            cmd_heave = r.get("cmd_heave", math.nan)
            cmd_roll = r.get("cmd_roll", math.nan)
            cmd_pitch = r.get("cmd_pitch", math.nan)
            cmd_yaw = r.get("cmd_yaw", math.nan)
            z_list.append([roll, pitch, yaw, gx, gy, gz])
            if all(math.isfinite(v) for v in [cmd_surge, cmd_sway, cmd_heave, cmd_roll, cmd_pitch, cmd_yaw]):
                u_list.append([cmd_surge, cmd_sway, cmd_heave, cmd_roll, cmd_pitch, cmd_yaw])
            else:
                u_list.append([0.0, 0.0, 0.0, cmd_roll, cmd_pitch, cmd_yaw])

    if len(t_list) == 0:
        return np.array([]), np.array([]), np.array([])

    z_arr = np.array(z_list, dtype=float)
    u_arr = np.array(u_list, dtype=float)
    z_arr[:, 0:3] = np.deg2rad(z_arr[:, 0:3])
    z_arr[:, 0:3] = wrap_angle(z_arr[:, 0:3])
    z0 = np.copy(z_arr[0, 0:3])
    for i in range(len(z_arr)):
        z_arr[i, 0:3] = wrap_angle(z_arr[i, 0:3] - z0)
    scale = math.pi / 180.0
    for i in range(len(t_list)):
        for j in range(3, 6):
            if math.isfinite(z_arr[i, j]):
                z_arr[i, j] *= scale
    for i in range(1, len(t_list)):
        dt = t_list[i] - t_list[i - 1]
        if dt <= 1e-4:
            continue
        for j in range(3, 6):
            if np.isnan(z_arr[i, j]):
                d_ang = wrap_angle(z_arr[i, j - 3] - z_arr[i - 1, j - 3])
                z_arr[i, j] = d_ang / dt
    for j in range(3, 6):
        if np.isnan(z_arr[0, j]):
            z_arr[0, j] = 0.0
    u_arr = np.nan_to_num(u_arr, nan=0.0)
    return np.array(t_list), z_arr, u_arr


def build_mimo_data_from_mat(path):
    mat = loadmat(path, squeeze_me=True, struct_as_record=False)
    y_struct = mat.get("y_rpy_pqr")
    u_struct = mat.get("u_rpy_cmd")

    if hasattr(y_struct, "signals") and hasattr(u_struct, "signals"):
        y_vals = np.asarray(y_struct.signals.values, dtype=float)
        y_time = np.asarray(y_struct.time, dtype=float).reshape(-1)
        u_vals = np.asarray(u_struct.signals.values, dtype=float)
        u_time = np.asarray(u_struct.time, dtype=float).reshape(-1)
        if y_vals.ndim != 2 or y_vals.shape[1] < 6:
            return np.array([]), np.array([]), np.array([])
        if u_vals.ndim != 2 or u_vals.shape[1] < 3:
            return np.array([]), np.array([]), np.array([])
        n = min(len(y_time), len(y_vals))
        if n == 0:
            return np.array([]), np.array([]), np.array([])
        t_arr = np.array(y_time[:n], dtype=float)
        z_arr = np.array(y_vals[:n, 0:6], dtype=float)
        u_roll = np.interp(t_arr, u_time, u_vals[:, 0], left=u_vals[0, 0], right=u_vals[-1, 0])
        u_pitch = np.interp(t_arr, u_time, u_vals[:, 1], left=u_vals[0, 1], right=u_vals[-1, 1])
        u_yaw = np.interp(t_arr, u_time, u_vals[:, 2], left=u_vals[0, 2], right=u_vals[-1, 2])
        u_arr = np.zeros((n, 6), dtype=float)
        u_arr[:, 3] = u_roll; u_arr[:, 4] = u_pitch; u_arr[:, 5] = u_yaw
        prs_t = np.asarray(mat.get("prs_time", []), dtype=float).reshape(-1)
        prs_cmd = np.asarray(mat.get("prs_cmd_surge_sway_heave", []), dtype=float)
        if prs_cmd.ndim == 2 and prs_cmd.shape[1] >= 3 and len(prs_t) > 1:
            for j in range(3):
                u_arr[:, j] = np.interp(t_arr, prs_t, prs_cmd[:, j], left=prs_cmd[0, j], right=prs_cmd[-1, j])
    else:
        imu_time = np.asarray(mat.get("imu_time", []), dtype=float).reshape(-1)
        imu_rpy = np.asarray(mat.get("imu_rpy", []), dtype=float)
        imu_gyro = np.asarray(mat.get("imu_gyro", []), dtype=float)
        imu_cmd_rpy = np.asarray(mat.get("imu_cmd_roll_pitch_yaw", []), dtype=float)
        imu_cmd_surge_sway = np.asarray(mat.get("imu_cmd_surge_sway", []), dtype=float)
        if imu_rpy.ndim != 2 or imu_rpy.shape[1] < 3:
            return np.array([]), np.array([]), np.array([])
        if imu_gyro.ndim != 2 or imu_gyro.shape[1] < 3:
            return np.array([]), np.array([]), np.array([])
        if imu_cmd_rpy.ndim != 2 or imu_cmd_rpy.shape[1] < 3:
            return np.array([]), np.array([]), np.array([])
        if imu_cmd_surge_sway.ndim != 2 or imu_cmd_surge_sway.shape[1] < 2:
            return np.array([]), np.array([]), np.array([])
        n = min(len(imu_time), len(imu_rpy), len(imu_gyro), len(imu_cmd_rpy), len(imu_cmd_surge_sway))
        if n == 0:
            return np.array([]), np.array([]), np.array([])
        t_arr = np.array(imu_time[:n], dtype=float)
        z_arr = np.zeros((n, 6), dtype=float)
        u_arr = np.zeros((n, 6), dtype=float)
        z_arr[:, 0:3] = np.array(imu_rpy[:n, 0:3], dtype=float)
        z_arr[:, 3:6] = np.array(imu_gyro[:n, 0:3], dtype=float)
        u_arr[:, 0:2] = np.array(imu_cmd_surge_sway[:n, 0:2], dtype=float)
        u_arr[:, 3:6] = np.array(imu_cmd_rpy[:n, 0:3], dtype=float)

    z_arr[:, 0:3] = np.deg2rad(z_arr[:, 0:3])
    z_arr[:, 0:3] = wrap_angle(z_arr[:, 0:3])
    z_arr[:, 3:6] = np.deg2rad(z_arr[:, 3:6])
    z_arr = np.nan_to_num(z_arr, nan=0.0)
    u_arr = np.nan_to_num(u_arr, nan=0.0)
    return t_arr, z_arr, u_arr


def zero_reference_mimo_data(t_arr, z_arr, u_arr):
    if len(t_arr) == 0:
        return t_arr, z_arr, u_arr
    t_arr = np.array(t_arr, dtype=float)
    z_ref = np.array(z_arr, dtype=float, copy=True)
    u_ref = np.array(u_arr, dtype=float, copy=True)
    z0 = np.array(z_ref[0], dtype=float, copy=True)
    u0 = np.array(u_ref[0], dtype=float, copy=True)
    for j in range(6):
        if j < 3:
            z_ref[:, j] = wrap_angle(z_ref[:, j] - z0[j])
        else:
            z_ref[:, j] = z_ref[:, j] - z0[j]
        u_ref[:, j] = u_ref[:, j] - u0[j]
    return t_arr, z_ref, u_ref


def load_pqr_identification_data(telemetry_path, duration_s=None, zero_reference=False,
                                  start_s=None, end_s=None):
    if telemetry_path.suffix.lower() == ".mat":
        t_arr, z_arr, u_arr = build_mimo_data_from_mat(telemetry_path)
    else:
        rows = read_telemetry(telemetry_path)
        t_arr, z_arr, u_arr = build_mimo_data(rows)

    if len(t_arr) == 0:
        return t_arr, z_arr, u_arr

    if zero_reference:
        t_arr, z_arr, u_arr = zero_reference_mimo_data(t_arr, z_arr, u_arr)
    else:
        t_arr = np.asarray(t_arr, dtype=float)
        z_arr = np.nan_to_num(np.asarray(z_arr, dtype=float), nan=0.0)
        u_arr = np.nan_to_num(np.asarray(u_arr, dtype=float), nan=0.0)

    finite_rows = np.isfinite(t_arr) & np.all(np.isfinite(z_arr[:, 3:6]), axis=1)
    t_arr = t_arr[finite_rows]
    z_arr = z_arr[finite_rows, :]
    u_arr = u_arr[finite_rows, :]

    t0 = t_arr[0] if len(t_arr) > 0 and math.isfinite(float(t_arr[0])) else 0.0
    elapsed_t = t_arr - t0
    lower_s = 0.0 if start_s is None else float(start_s)
    upper_s = None
    if end_s is not None:
        upper_s = float(end_s)
    elif duration_s is not None and math.isfinite(float(duration_s)) and float(duration_s) > 0.0:
        upper_s = lower_s + float(duration_s)
    if lower_s > 0.0 or upper_s is not None:
        keep = elapsed_t >= lower_s
        if upper_s is not None:
            keep &= elapsed_t <= upper_s
        t_arr = t_arr[keep]
        z_arr = z_arr[keep, :]
        u_arr = u_arr[keep, :]
    return t_arr, z_arr, u_arr


# =============================================================================
# Tau (input moment) construction pipelines
# =============================================================================
def apply_input_signs(u_arr, input_signs):
    u_out = np.array(u_arr, dtype=float, copy=True)
    signs = np.asarray(input_signs, dtype=float)
    if u_out.ndim == 2 and u_out.shape[1] >= 6:
        u_out[:, 3:6] *= signs[None, :]
    elif u_out.ndim == 2 and u_out.shape[1] >= 3:
        u_out[:, 0:3] *= signs[None, :]
    else:
        raise ValueError("Input command array must have at least 3 columns.")
    return u_out


def command_tau_history(u_arr, tau_scale, input_signs):
    """Direct command-to-moment scaling (no thruster physics)."""
    u_signed = apply_input_signs(u_arr, input_signs)
    if u_signed.shape[1] >= 6:
        tau = u_signed[:, 3:6]
    else:
        tau = u_signed[:, 0:3]
    return float(tau_scale) * np.asarray(tau, dtype=float)


def enforce_cmd6(u_cmd):
    u_cmd = np.asarray(u_cmd, dtype=float).reshape(-1)
    tau6 = np.zeros(6, dtype=float)
    if u_cmd.size >= 6:
        tau6[:] = u_cmd[:6]
    elif u_cmd.size >= 3:
        tau6[3:6] = u_cmd[:3]
    return tau6


def thruster_u_to_desired_force_N(u_thr):
    """Static deadzone + asymmetric thrust map."""
    u_sat = np.clip(np.asarray(u_thr, dtype=float), -1.0, 1.0)
    force = np.zeros_like(u_sat)
    for i, ui in enumerate(u_sat):
        aui = abs(ui)
        if aui <= THRUSTER_DEADZONE:
            force[i] = 0.0
            continue
        mag = (aui - THRUSTER_DEADZONE) / (1.0 - THRUSTER_DEADZONE)
        mag = np.clip(mag, 0.0, 1.0)
        if ui >= 0.0:
            force[i] = mag * MAX_FWD_FORCE_N
        else:
            force[i] = -mag * MAX_REV_FORCE_N
    return force


def thruster_bank_step(thr_state_24, force_cmd_8, dt):
    """T200 3rd-order dynamics for 8 thrusters."""
    thr_state_24 = np.asarray(thr_state_24, dtype=float)
    force_cmd_8 = np.asarray(force_cmd_8, dtype=float)
    next_state = np.zeros_like(thr_state_24)
    force_out = np.zeros(8, dtype=float)
    for i in range(8):
        s = thr_state_24[3 * i : 3 * i + 3]
        u = force_cmd_8[i]
        s_dot = T200_A @ s + T200_B * u
        s_next = s + dt * s_dot
        y = float(T200_C @ s_next)
        y = float(np.clip(y, -1.5 * MAX_REV_FORCE_N, 1.5 * MAX_FWD_FORCE_N))
        s_next = np.clip(s_next, -1e4, 1e4)
        next_state[3 * i : 3 * i + 3] = s_next
        force_out[i] = y
    return next_state, force_out


def allocator_actuator_block_with_dynamics(u_cmd, thr_state_24, dt):
    """Full actuator pipeline: pinv(B) → static map → T200 dynamics → B."""
    tau6 = enforce_cmd6(u_cmd)
    u_thr = PINVB @ tau6
    T_cmd = thruster_u_to_desired_force_N(u_thr)
    thr_state_next, T_act = thruster_bank_step(thr_state_24, T_cmd, dt)
    tau_act_6 = B_ALLOC @ T_act
    return tau_act_6, thr_state_next, T_cmd, T_act


def actuator_tau_history(t_arr, u_arr, input_signs):
    """Build rotational tau via full actuator dynamics."""
    u_signed = apply_input_signs(u_arr, input_signs)
    tau_rot = np.zeros((len(t_arr), 3), dtype=float)
    thr_state = np.zeros(24, dtype=float)
    for i in range(len(t_arr)):
        if i == 0:
            dt = 0.02
        else:
            dt = float(t_arr[i] - t_arr[i - 1])
            if not math.isfinite(dt) or dt <= 1e-5:
                dt = 0.02
            dt = max(1e-4, min(0.2, dt))
        tau_act_6, thr_state, _, _ = allocator_actuator_block_with_dynamics(u_signed[i], thr_state, dt)
        tau_rot[i, :] = tau_act_6[3:6]
    return tau_rot


def thruster_config_tau_history(u_arr, tau_scale, input_signs):
    """Build rotational tau via ThrusterConfigMatrix LUT + geometry (DEFAULT path)."""
    lut = load_t200_14v_lut()
    b_matrix = build_thruster_config_matrix()
    u_signed = apply_input_signs(u_arr, input_signs)
    if u_signed.shape[1] >= 6:
        moment_cmd = u_signed[:, 3:6]
    else:
        moment_cmd = u_signed[:, 0:3]
    wrench_cmd = np.zeros((len(u_signed), 6), dtype=float)
    wrench_cmd[:, 3:6] = float(tau_scale) * np.asarray(moment_cmd, dtype=float)
    net_wrench, _, _, _, _ = wrench_to_net_wrench_t200_14v(wrench_cmd, lut=lut, b_matrix=b_matrix)
    return net_wrench[:, 3:6]


def build_pqr_tau_history(t_arr, u_arr, source, tau_scale, input_signs):
    """Dispatcher for tau construction."""
    if source in ("thruster_config", "lut"):
        return thruster_config_tau_history(u_arr, tau_scale, input_signs)
    if source in ("command", "direct"):
        return command_tau_history(u_arr, tau_scale, input_signs)
    if source == "actuator":
        return actuator_tau_history(t_arr, u_arr, input_signs)
    raise ValueError(f"Unsupported tau source: {source}")


# =============================================================================
# Rotational dynamics
# =============================================================================
def restoring_moment_rpy(attitude):
    phi, theta, _ = np.asarray(attitude, dtype=float)
    g4 = -(ROVParams.y_g * ROVParams.W - ROVParams.y_B * ROVParams.B) * math.cos(theta) * math.cos(phi) + \
         (ROVParams.z_g * ROVParams.W - ROVParams.z_B * ROVParams.B) * math.cos(theta) * math.sin(phi)
    g5 = (ROVParams.z_g * ROVParams.W - ROVParams.z_B * ROVParams.B) * math.sin(theta) + \
         (ROVParams.x_g * ROVParams.W - ROVParams.x_B * ROVParams.B) * math.cos(theta) * math.cos(phi)
    g6 = -(ROVParams.x_g * ROVParams.W - ROVParams.x_B * ROVParams.B) * math.cos(theta) * math.sin(phi) - \
         (ROVParams.y_g * ROVParams.W - ROVParams.y_B * ROVParams.B) * math.sin(theta)
    return np.array([g4, g5, g6], dtype=float)


def pqr_rotational_accel(w, tau, pvec, attitude=None):
    pvec = normalize_pqr_param_vec(pvec)
    K_dp, M_dq, N_dr, K_dq, K_dr, M_dr, M_dp, N_dp, N_dq, \
    K_p, K_q, K_r, M_p, M_q, M_r, N_p, N_q, N_r, \
    K_pp, K_qq, K_rr, M_pp, M_qq, M_rr, N_pp, N_qq, N_rr, \
    G_p, G_q, G_r = pvec

    p_rate, q_rate, r_rate = np.asarray(w, dtype=float)
    w_vec = np.array([p_rate, q_rate, r_rate], dtype=float)

    M_val = np.array([
        [ROVParams.I_x + K_dp, -ROVParams.I_xy + K_dq, -ROVParams.I_xz + K_dr],
        [-ROVParams.I_xy + M_dp, ROVParams.I_y + M_dq, -ROVParams.I_yz + M_dr],
        [-ROVParams.I_xz + N_dp, -ROVParams.I_yz + N_dq, ROVParams.I_z + N_dr],
    ], dtype=float)

    M_added = np.array([[K_dp, K_dq, K_dr], [M_dp, M_dq, M_dr], [N_dp, N_dq, N_dr]], dtype=float)
    b1, b2, b3 = -M_added @ w_vec

    C45 = -ROVParams.I_xy * q_rate - ROVParams.I_xz * p_rate + ROVParams.I_z * r_rate - b3
    C46 = ROVParams.I_yz * r_rate + ROVParams.I_xy * p_rate - ROVParams.I_y * q_rate + b2
    C54 = ROVParams.I_xy * q_rate + ROVParams.I_xz * p_rate - ROVParams.I_z * r_rate + b3
    C56 = -ROVParams.I_xz * r_rate - ROVParams.I_xy * q_rate + ROVParams.I_x * p_rate - b1
    C64 = -ROVParams.I_yz * r_rate - ROVParams.I_xy * p_rate + ROVParams.I_y * q_rate - b2
    C65 = ROVParams.I_xz * r_rate + ROVParams.I_xy * q_rate - ROVParams.I_x * p_rate + b1
    C_val = np.array([[0.0, C45, C46], [C54, 0.0, C56], [C64, C65, 0.0]], dtype=float)

    D_linear = np.array([
        [abs(K_p), K_q, K_r], [M_p, abs(M_q), M_r], [N_p, N_q, abs(N_r)]], dtype=float)
    D_quad = np.array([
        [abs(K_pp), K_qq, K_rr], [M_pp, abs(M_qq), M_rr], [N_pp, N_qq, abs(N_rr)]], dtype=float)
    w_abs_w = np.array([abs(p_rate) * p_rate, abs(q_rate) * q_rate, abs(r_rate) * r_rate], dtype=float)

    tau_gain = np.array([G_p, G_q, G_r], dtype=float) * np.asarray(tau, dtype=float)
    restoring = np.zeros(3, dtype=float) if attitude is None else restoring_moment_rpy(attitude)
    rhs = tau_gain - C_val @ w_vec - D_linear @ w_vec - D_quad @ w_abs_w - restoring

    try:
        w_dot = np.linalg.solve(M_val + 1e-8 * np.eye(3), rhs)
    except np.linalg.LinAlgError:
        w_dot = np.zeros(3, dtype=float)
    if np.any(~np.isfinite(w_dot)):
        w_dot = np.zeros(3, dtype=float)
    return np.clip(w_dot, -80.0, 80.0)


def pqr_euler_angle_rates(attitude, rates):
    phi, theta, _ = np.asarray(attitude, dtype=float)
    p_rate, q_rate, r_rate = np.asarray(rates, dtype=float)
    theta = float(np.clip(theta, np.deg2rad(-89.0), np.deg2rad(89.0)))
    cos_theta = math.cos(theta)
    if abs(cos_theta) < 1e-6:
        cos_theta = math.copysign(1e-6, cos_theta if cos_theta != 0.0 else 1.0)
    tan_theta = math.sin(theta) / cos_theta
    phi_dot = p_rate + math.sin(phi) * tan_theta * q_rate + math.cos(phi) * tan_theta * r_rate
    theta_dot = math.cos(phi) * q_rate - math.sin(phi) * r_rate
    psi_dot = math.sin(phi) / cos_theta * q_rate + math.cos(phi) / cos_theta * r_rate
    return np.array([phi_dot, theta_dot, psi_dot], dtype=float)


def attitude_pqr_derivative(x6, tau, pvec):
    x6 = np.asarray(x6, dtype=float)
    return np.concatenate([
        pqr_euler_angle_rates(x6[0:3], x6[3:6]),
        pqr_rotational_accel(x6[3:6], tau, pvec, attitude=x6[0:3]),
    ])


def attitude_pqr_step(x6, tau0, tau1, dt, pvec):
    x0 = np.array(x6, dtype=float, copy=True)
    x0[0:3] = wrap_angle(x0[0:3])
    x0[1] = np.clip(x0[1], np.deg2rad(-89.0), np.deg2rad(89.0))
    x0[3:6] = np.clip(x0[3:6], -RATE_LIMIT_RAD_S, RATE_LIMIT_RAD_S)

    tau_mid = 0.5 * (np.asarray(tau0, dtype=float) + np.asarray(tau1, dtype=float))
    k1 = attitude_pqr_derivative(x0, tau0, pvec)
    k2 = attitude_pqr_derivative(x0 + 0.5 * dt * k1, tau_mid, pvec)
    k3 = attitude_pqr_derivative(x0 + 0.5 * dt * k2, tau_mid, pvec)
    k4 = attitude_pqr_derivative(x0 + dt * k3, tau1, pvec)
    x_next = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    if np.any(~np.isfinite(x_next)):
        x_next = x0
    x_next[0:3] = wrap_angle(x_next[0:3])
    x_next[1] = np.clip(x_next[1], np.deg2rad(-89.0), np.deg2rad(89.0))
    x_next[3:6] = np.clip(x_next[3:6], -RATE_LIMIT_RAD_S, RATE_LIMIT_RAD_S)
    return x_next


def simulate_attitude_pqr_free_run(t_arr, tau_arr, attitude0, rate0, pvec):
    pvec = normalize_pqr_param_vec(pvec)
    y = np.zeros((len(t_arr), 6), dtype=float)
    y[0, 0:3] = wrap_angle(np.asarray(attitude0, dtype=float))
    y[0, 1] = np.clip(y[0, 1], np.deg2rad(-89.0), np.deg2rad(89.0))
    y[0, 3:6] = np.clip(np.asarray(rate0, dtype=float), -RATE_LIMIT_RAD_S, RATE_LIMIT_RAD_S)
    for i in range(1, len(t_arr)):
        dt = float(t_arr[i] - t_arr[i - 1])
        if not math.isfinite(dt) or dt <= 1e-5:
            dt = 0.02
        dt = max(1e-4, min(0.2, dt))
        y[i, :] = attitude_pqr_step(y[i - 1, :], tau_arr[i - 1, :], tau_arr[i, :], dt, pvec)
    return y


# =============================================================================
# Metrics
# =============================================================================
def pqr_rmse_deg(rate_ref, rate_pred):
    return np.rad2deg(np.sqrt(np.mean((rate_pred - rate_ref) ** 2, axis=0)))


# =============================================================================
# Plotting
# =============================================================================
def set_publication_style():
    plt.rcParams.update({
        "font.family": "serif", "font.size": 10,
        "axes.titlesize": 10, "axes.labelsize": 10,
        "xtick.labelsize": 10, "ytick.labelsize": 10,
        "legend.fontsize": 8,
    })


# =============================================================================
# Main
# =============================================================================
def main():
    set_publication_style()

    # ── 1. Load telemetry ────────────────────────────────────────────────
    telemetry_path = Path(__file__).parent / "Experiment.mat"
    if not telemetry_path.exists():
        print(f"Telemetry file not found: {telemetry_path}")
        return

    print(f"Loading: {telemetry_path}")
    t_arr, z_arr, u_arr = load_pqr_identification_data(
        telemetry_path, duration_s=None, zero_reference=False)
    if len(t_arr) < 20:
        print("Insufficient valid samples.")
        return

    t_fit, z_fit, u_fit = t_arr, z_arr, u_arr
    attitude_fit = z_fit[:, 0:3]
    rate_meas = z_fit[:, 3:6]

    # ── 2. Build tau — same pipeline as GUI "Simulate Free Run" ──────────
    tau_source = "thruster_config"
    tau_scale = DEFAULT_THRUSTER_CONFIG_SCALE
    input_signs = DEFAULT_INPUT_SIGNS

    print(f"Building tau: source={tau_source}, scale={tau_scale}, signs={input_signs}")
    tau_fit = build_pqr_tau_history(t_fit, u_fit, tau_source, tau_scale, input_signs)

    # ── 3. Simulate ──────────────────────────────────────────────────────
    print(f"Simulating {len(t_fit)} samples with default hydro parameters …")
    y_sim = simulate_attitude_pqr_free_run(
        t_fit, tau_fit, attitude_fit[0], rate_meas[0], DEFAULT_HYDRO_PARAMS)

    p_sim, q_sim, r_sim = y_sim[:, 3], y_sim[:, 4], y_sim[:, 5]

    # ── 4. Diagnostics ───────────────────────────────────────────────────
    rmse = pqr_rmse_deg(rate_meas, y_sim[:, 3:6])
    print(f"Rate RMSE (deg/s): p={rmse[0]:.3f}, q={rmse[1]:.3f}, r={rmse[2]:.3f}")

    # ── 5. Plot ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), constrained_layout=True)

    rate_labels = ["p", "q", "r"]
    rate_sim = [p_sim, q_sim, r_sim]
    tau_labels = [r"$\tau_{\mathrm{roll}}$", r"$\tau_{\mathrm{pitch}}$", r"$\tau_{\mathrm{yaw}}$"]

    for row in range(3):
        ax_tau = axes[row, 0]
        ax_tau.plot(t_fit, tau_fit[:, row], color="tab:blue", linewidth=0.6)
        ax_tau.set_ylabel(f"{tau_labels[row]} (N·m)")
        ax_tau.grid(True, alpha=0.25)
        if row == 2:
            ax_tau.set_xlabel("Time (s)")

        ax_rate = axes[row, 1]
        ax_rate.plot(t_fit, np.rad2deg(rate_meas[:, row]),
                     color="k", linewidth=0.6, label="Measured")
        ax_rate.plot(t_fit, np.rad2deg(rate_sim[row]),
                     color="tab:red", linewidth=0.8, label="Simulated")
        ax_rate.set_ylabel(f"{rate_labels[row]} (deg/s)")
        ax_rate.grid(True, alpha=0.25)
        ax_rate.legend(loc="upper right", fontsize=8)
        if row == 2:
            ax_rate.set_xlabel("Time (s)")

    axes[0, 0].set_title("Input moment (thruster_config pipeline)")
    axes[0, 1].set_title("Angular velocity")

    fig.suptitle("ROV Rotational Dynamics — Experiment.mat replay\n"
                 f"tau_source={tau_source}, default hydro params | "
                 f"RMSE p={rmse[0]:.1f} q={rmse[1]:.1f} r={rmse[2]:.1f} deg/s",
                 fontweight="bold")

    out_path = Path(__file__).parent / "simulation_experiment_replay.svg"
    fig.savefig(out_path, format="svg")
    print(f"Saved plot: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
