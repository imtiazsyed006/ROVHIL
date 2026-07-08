import numpy as np
import matplotlib.pyplot as plt
import zipfile
import xml.etree.ElementTree as et
from pathlib import Path

plt.rcParams["font.family"] = "Times New Roman"

KGF_TO_N = 9.80665

THRUSTER_POSITIONS = np.array(
    [
        [0.174523918, 0.1116943, -0.0977792],   # 1 FR_Top
        [0.172946864, -0.1132360, -0.1000560],  # 2 FL_Top
        [-0.181952650, 0.1132714, -0.1000560],  # 3 RR_Top
        [-0.183529700, -0.1116590, -0.0977792], # 4 RL_Top
        [0.172946864, 0.1132714, 0.1080494],    # 5 FR_Bot
        [0.174523918, -0.1116590, 0.1057727],   # 6 FL_Bot
        [-0.183529700, 0.1116943, 0.1057727],   # 7 RR_Bot
        [-0.181952650, -0.1132360, 0.1080494],  # 8 RL_Bot
    ],
    dtype=float,
)

THRUSTER_LABELS = [
    "FR_Top",
    "FL_Top",
    "RR_Top",
    "RL_Top",
    "FR_Bot",
    "FL_Bot",
    "RR_Bot",
    "RL_Bot",
]

THRUSTER_DIRECTIONS = np.array(
    [
        [0.505122, -0.505120, -0.699790],  # 1 FR_Top
        [0.505122, 0.505122, -0.699790],   # 2 FL_Top
        [-0.505120, -0.505120, -0.699790], # 3 RR_Top
        [-0.505120, 0.505122, -0.699790],  # 4 RL_Top
        [0.505122, -0.505120, 0.699788],   # 5 FR_Bot
        [0.505122, 0.505122, 0.699788],    # 6 FL_Bot
        [-0.505120, -0.505120, 0.699788],  # 7 RR_Bot
        [-0.505120, 0.505122, 0.699788],   # 8 RL_Bot
    ],
    dtype=float,
)
THRUSTER_DIRECTIONS = THRUSTER_DIRECTIONS / np.linalg.norm(THRUSTER_DIRECTIONS, axis=1, keepdims=True)


def sind(angle_deg):
    return np.sin(np.deg2rad(angle_deg))


def cosd(angle_deg):
    return np.cos(np.deg2rad(angle_deg))


def draw_arc_arrow(ax, center, axis_name, moment, radius, sweep_deg, color, line_width):
    if abs(moment) < 1e-9 or sweep_deg < 1:
        return

    sign = np.sign(moment)
    sweep = np.deg2rad(sweep_deg)
    t = np.linspace(0.0, sign * sweep, 120)

    if axis_name.lower() == "z":
        x = center[0] + radius * np.cos(t)
        y = center[1] + radius * np.sin(t)
        z = center[2] + np.zeros_like(t)
        tangent = np.array([-radius * np.sin(t[-1]), radius * np.cos(t[-1]), 0.0])
    elif axis_name.lower() == "x":
        x = center[0] + np.zeros_like(t)
        y = center[1] + radius * np.cos(t)
        z = center[2] + radius * np.sin(t)
        tangent = np.array([0.0, -radius * np.sin(t[-1]), radius * np.cos(t[-1])])
    elif axis_name.lower() == "y":
        x = center[0] + radius * np.sin(t)
        y = center[1] + np.zeros_like(t)
        z = center[2] + radius * np.cos(t)
        tangent = np.array([radius * np.cos(t[-1]), 0.0, -radius * np.sin(t[-1])])
    else:
        raise ValueError(f"Unsupported axis name: {axis_name}")

    ax.plot(x, y, z, color=color, linewidth=line_width)

    tip = np.array([x[-1], y[-1], z[-1]])
    tangent = tangent / (np.linalg.norm(tangent) + 1e-12)
    head_len = 0.18 * radius

    ax.quiver(
        tip[0] - tangent[0] * head_len,
        tip[1] - tangent[1] * head_len,
        tip[2] - tangent[2] * head_len,
        tangent[0] * head_len,
        tangent[1] * head_len,
        tangent[2] * head_len,
        color=color,
        linewidth=line_width,
        arrow_length_ratio=0.4,
    )


def load_t200_14v_lut(excel_path="T200-Public-Performance-Data-10-20V-September-2019.xlsx"):
    excel_path = Path(excel_path)
    if not excel_path.exists() and not excel_path.is_absolute():
        excel_path = Path(__file__).resolve().parent / excel_path

    ns = {"a": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    rel_ns = {"r": "http://schemas.openxmlformats.org/package/2006/relationships"}
    doc_rel_ns = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"

    with zipfile.ZipFile(excel_path) as zf:
        shared_strings = []
        if "xl/sharedStrings.xml" in zf.namelist():
            ss_root = et.fromstring(zf.read("xl/sharedStrings.xml"))
            for si in ss_root.findall("a:si", ns):
                shared_strings.append("".join(t.text or "" for t in si.findall(".//a:t", ns)))

        wb_root = et.fromstring(zf.read("xl/workbook.xml"))
        rel_root = et.fromstring(zf.read("xl/_rels/workbook.xml.rels"))
        rid_to_target = {
            rel.attrib["Id"]: rel.attrib["Target"] for rel in rel_root.findall(".//r:Relationship", rel_ns)
        }

        sheet_target = None
        for sheet in wb_root.findall(".//a:sheets/a:sheet", ns):
            if sheet.attrib.get("name") == "14 V":
                rid = sheet.attrib.get(f"{{{doc_rel_ns}}}id")
                sheet_target = rid_to_target.get(rid)
                break

        if sheet_target is None:
            raise RuntimeError("Could not find '14 V' sheet in T200 workbook.")

        sheet_xml = "xl/" + sheet_target.replace("\\", "/")
        sh_root = et.fromstring(zf.read(sheet_xml))

        rows = []
        for row in sh_root.findall(".//a:sheetData/a:row", ns):
            row_data = {}
            for cell in row.findall("a:c", ns):
                cell_ref = cell.attrib.get("r", "")
                col = "".join(ch for ch in cell_ref if ch.isalpha())
                if col not in ("A", "F"):  # PWM (us), Force (kgf)
                    continue
                value_node = cell.find("a:v", ns)
                if value_node is None or value_node.text is None:
                    continue
                raw_value = value_node.text
                if cell.attrib.get("t") == "s":
                    raw_value = shared_strings[int(raw_value)]
                try:
                    row_data[col] = float(raw_value)
                except ValueError:
                    pass
            if "A" in row_data and "F" in row_data:
                rows.append((row_data["A"], row_data["F"]))

    if not rows:
        raise RuntimeError("No PWM/Force rows were parsed from 14 V sheet.")

    lut = np.array(rows, dtype=float)
    lut = lut[np.argsort(lut[:, 0])]  # sort by PWM (us)
    return lut


def _unique_interp_xy(x, y):
    order = np.argsort(x)
    x_sorted = x[order]
    y_sorted = y[order]
    x_unique, idx = np.unique(x_sorted, return_index=True)
    y_unique = y_sorted[idx]
    return x_unique, y_unique


def map_u_to_pwm_t200_14v(u, lut):
    u_sat = np.clip(np.asarray(u, dtype=float), -1.0, 1.0)

    pwm_us = lut[:, 0]
    force_kgf = lut[:, 1]

    rev_mask = force_kgf < 0.0
    fwd_mask = force_kgf > 0.0

    rev_force_mag = np.abs(force_kgf[rev_mask])
    rev_pwm_us = pwm_us[rev_mask]
    fwd_force_mag = force_kgf[fwd_mask]
    fwd_pwm_us = pwm_us[fwd_mask]

    rev_x, rev_y = _unique_interp_xy(rev_force_mag, rev_pwm_us)
    fwd_x, fwd_y = _unique_interp_xy(fwd_force_mag, fwd_pwm_us)

    # Use endpoint thrust values so full-scale commands always hit PWM endpoints.
    max_rev = abs(force_kgf[np.argmin(pwm_us)])   # ~force at 1100 us
    max_fwd = force_kgf[np.argmax(pwm_us)]        # ~force at 1900 us

    pwm_us_out = np.full_like(u_sat, 1500.0)

    pos = u_sat > 0.0
    neg = u_sat < 0.0

    if np.any(pos):
        desired_f = u_sat[pos] * max_fwd
        pwm_us_out[pos] = np.interp(desired_f, fwd_x, fwd_y)

    if np.any(neg):
        desired_f = np.abs(u_sat[neg]) * max_rev
        pwm_us_out[neg] = np.interp(desired_f, rev_x, rev_y)

    # Convert Blue Robotics PWM (1100..1900 us, neutral 1500) to controller scale:
    # 1100 -> 4400, 1500 -> 6000, 1900 -> 7600
    pwm_ctrl = (pwm_us_out - 1500.0) * 4.0 + 6000.0
    return np.clip(pwm_ctrl, 4400.0, 7600.0)


def map_pwm_ctrl_to_force_t200_14v_kgf(pwm_ctrl, lut):
    pwm_ctrl = np.asarray(pwm_ctrl, dtype=float)
    pwm_us = (pwm_ctrl - 6000.0) / 4.0 + 1500.0

    pwm_lut, force_lut = _unique_interp_xy(lut[:, 0], lut[:, 1])
    force_kgf = np.interp(
        np.clip(pwm_us, pwm_lut[0], pwm_lut[-1]),
        pwm_lut,
        force_lut,
    )
    return force_kgf


def map_pwm_ctrl_to_force_t200_14v_N(pwm_ctrl, lut):
    return map_pwm_ctrl_to_force_t200_14v_kgf(pwm_ctrl, lut) * KGF_TO_N


def build_thruster_config_matrix(pos=None, dirs=None):
    if pos is None:
        pos = THRUSTER_POSITIONS
    if dirs is None:
        dirs = THRUSTER_DIRECTIONS

    pos = np.asarray(pos, dtype=float)
    dirs = np.asarray(dirs, dtype=float)
    dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)

    b_matrix = np.zeros((6, 8), dtype=float)
    for i in range(8):
        force_axis = dirs[i, :]
        moment_axis = np.cross(pos[i, :], force_axis)
        b_matrix[:, i] = np.concatenate((force_axis, moment_axis))
    return b_matrix


def wrench_to_thruster_commands(wrench_6, b_matrix=None):
    if b_matrix is None:
        b_matrix = build_thruster_config_matrix()

    wrench_arr = np.asarray(wrench_6, dtype=float)
    squeeze = wrench_arr.ndim == 1
    wrench_arr = np.atleast_2d(wrench_arr)

    b_pinv = np.linalg.pinv(b_matrix)
    u_raw = wrench_arr @ b_pinv.T
    u_sat = np.clip(u_raw, -1.0, 1.0)
    return (u_raw[0], u_sat[0]) if squeeze else (u_raw, u_sat)


def wrench_to_net_wrench_t200_14v(wrench_6, lut=None, b_matrix=None):
    if lut is None:
        lut = load_t200_14v_lut()
    if b_matrix is None:
        b_matrix = build_thruster_config_matrix()

    wrench_arr = np.asarray(wrench_6, dtype=float)
    squeeze = wrench_arr.ndim == 1
    wrench_arr = np.atleast_2d(wrench_arr)

    u_raw, u_sat = wrench_to_thruster_commands(wrench_arr, b_matrix=b_matrix)
    pwm_ctrl = map_u_to_pwm_t200_14v(u_sat, lut)
    thruster_force_N = map_pwm_ctrl_to_force_t200_14v_N(pwm_ctrl, lut)
    net_wrench = thruster_force_N @ b_matrix.T

    if squeeze:
        return net_wrench[0], u_raw[0], u_sat[0], pwm_ctrl[0], thruster_force_N[0]
    return net_wrench, u_raw, u_sat, pwm_ctrl, thruster_force_N


def main():
    t200_lut_14v = load_t200_14v_lut()

    # =================== 1) Geometry & angles ===================
    pos = np.array(
        [
            [0.174523918, 0.1116943, -0.0977792],   # 1 FR_Top
            [0.172946864, -0.1132360, -0.1000560],  # 2 FL_Top
            [-0.181952650, 0.1132714, -0.1000560],  # 3 RR_Top
            [-0.183529700, -0.1116590, -0.0977792], # 4 RL_Top
            [0.172946864, 0.1132714, 0.1080494],    # 5 FR_Bot
            [0.174523918, -0.1116590, 0.1057727],   # 6 FL_Bot
            [-0.183529700, 0.1116943, 0.1057727],   # 7 RR_Bot
            [-0.181952650, -0.1132360, 0.1080494],  # 8 RL_Bot
        ],
        dtype=float,
    )

    labels = [
        "FR_Top",
        "FL_Top",
        "RR_Top",
        "RL_Top",
        "FR_Bot",
        "FL_Bot",
        "RR_Bot",
        "RL_Bot",
    ]

    # =================== Thruster direction unit vectors (PHYSICS) ===================
    dirs = np.array(
        [
            [0.505122, -0.505120, -0.699790],  # 1 FR_Top
            [0.505122, 0.505122, -0.699790],   # 2 FL_Top
            [-0.505120, -0.505120, -0.699790], # 3 RR_Top
            [-0.505120, 0.505122, -0.699790],  # 4 RL_Top
            [0.505122, -0.505120, 0.699788],   # 5 FR_Bot
            [0.505122, 0.505122, 0.699788],    # 6 FL_Bot
            [-0.505120, -0.505120, 0.699788],  # 7 RR_Bot
            [-0.505120, 0.505122, 0.699788],   # 8 RL_Bot
        ],
        dtype=float,
    )
    dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)

    lx = np.max(np.abs(pos[:, 0]))
    ly = np.max(np.abs(pos[:, 1]))
    h = np.max(np.abs(pos[:, 2]))

    # =================== Build PHYSICS-BASED B using r x d ===================
    b_matrix = np.zeros((6, 8), dtype=float)
    for i in range(8):
        r = pos[i, :]
        d = dirs[i, :]
        tau = np.cross(r, d)
        b_matrix[:, i] = np.concatenate((d, tau))

    b_pinv = np.linalg.pinv(b_matrix)

    print("B =")
    print(b_matrix)
    print("pinv(B) =")
    print(b_pinv)

    # =================== Default wrench used in test list ===================
    wrench = np.array([1, 0, 0, 0, 0, 0], dtype=float)

    def plot_case(case_name, u_case, pwm_case, f_net_case, tau_net_case):
        pwm_delta = np.abs(pwm_case - 6000.0)

        # =================== Plot thruster force vectors ===================
        len_scale = 7e-5
        vecs = dirs * (pwm_delta[:, None] * len_scale)
        tips = pos + vecs

        fig = plt.figure(facecolor="white")
        ax = fig.add_subplot(111, projection="3d")
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"{case_name}: Thruster Forces + Net Force + Curved Moments")

        ax.plot([lx, lx, -lx, -lx, lx], [ly, ly, ly, ly, ly], [h, -h, -h, h, h], "k-", linewidth=1.2)
        ax.plot([lx, lx, -lx, -lx, lx], [-ly, -ly, -ly, -ly, -ly], [h, -h, -h, h, h], "k-", linewidth=1.2)

        thruster_color = (0.0, 0.45, 0.74)
        for i in range(8):
            ax.plot(
                [pos[i, 0], tips[i, 0]],
                [pos[i, 1], tips[i, 1]],
                [pos[i, 2], tips[i, 2]],
                linewidth=3,
                color=thruster_color,
            )

        ax.quiver(
            pos[:, 0],
            pos[:, 1],
            pos[:, 2],
            vecs[:, 0],
            vecs[:, 1],
            vecs[:, 2],
            color=thruster_color,
            linewidth=2.5,
            arrow_length_ratio=0.15,
        )

        # =================== Net force + net moment from PHYSICS ===================
        force_scale = 0.10
        ax.quiver(
            0,
            0,
            0,
            f_net_case[0] * force_scale,
            f_net_case[1] * force_scale,
            f_net_case[2] * force_scale,
            color="k",
            linewidth=3,
            arrow_length_ratio=0.15,
        )
        ax.text(0, 0, 0, "  Net Force", fontweight="bold")

        # =================== Curved moment arrows (roll/pitch/yaw) ===================
        r_arc = 0.25 * max(lx, ly)
        line_width_arc = 2
        sweep_deg = 260

        ref = max(1e-9, np.max(np.abs(tau_net_case)))
        s_roll = sweep_deg * min(1, abs(tau_net_case[0]) / ref)
        s_pitch = sweep_deg * min(1, abs(tau_net_case[1]) / ref)
        s_yaw = sweep_deg * min(1, abs(tau_net_case[2]) / ref)

        draw_arc_arrow(ax, np.array([0.0, 0.0, 0.0]), "x", tau_net_case[0], r_arc, s_roll, (0.85, 0.33, 0.10), line_width_arc)
        draw_arc_arrow(ax, np.array([0.0, 0.0, 0.0]), "y", tau_net_case[1], r_arc, s_pitch, (0.10, 0.60, 0.10), line_width_arc)
        draw_arc_arrow(ax, np.array([0.0, 0.0, 0.0]), "z", tau_net_case[2], r_arc, s_yaw, (0.90, 0.10, 0.10), line_width_arc)

        label_offset = 0.03
        for i in range(8):
            ax.text(
                pos[i, 0],
                pos[i, 1],
                pos[i, 2] + label_offset,
                labels[i],
                fontweight="bold",
                fontsize=10,
                color="k",
            )

        ax.view_init(elev=25, azim=35)

    # =================== TEST HARNESS ===================
    tests = [
        ("Surge only", np.array([1, 0, 0, 0, 0, 0], dtype=float)),
        ("Sway only", np.array([0, 1, 0, 0, 0, 0], dtype=float)),
        ("Heave only", np.array([0, 0, 1, 0, 0, 0], dtype=float)),
        ("Roll only", np.array([0, 0, 0, 1, 0, 0], dtype=float)),
        ("Pitch only", np.array([0, 0, 0, 0, 1, 0], dtype=float)),
        ("Yaw only", np.array([0, 0, 0, 0, 0, 1], dtype=float)),
        ("Sway + 10*Yaw", np.array([0, 1, 0, 0, 0, 10], dtype=float)),
        ("10*Surge + 10*Yaw", np.array([10, 0, 0, 0, 0, 10], dtype=float)),
        ("Your current", wrench),
    ]

    print("\n==================== PYTHON TEST HARNESS ====================")
    print("Mapping: T200 14V LUT (Force<->PWM), then remap 1100..1900us -> 4400..7600 (neutral 6000)")
    print("Thruster order: 1 FR_T, 2 FL_T, 3 RR_T, 4 RL_T, 5 FR_B, 6 FL_B, 7 RR_B, 8 RL_B")

    for name, test_wrench in tests:
        u_test = b_pinv @ test_wrench
        u_sat_test = np.clip(u_test, -1, 1)

        pwm_test = map_u_to_pwm_t200_14v(u_sat_test, t200_lut_14v)

        f_thr = dirs * u_test[:, None]
        f_net_test = np.sum(f_thr, axis=0)
        tau_thr = np.cross(pos, f_thr)
        tau_net_test = np.sum(tau_thr, axis=0)

        print(f"\n--- {name} ---")
        print(
            "w = [Fx Fy Fz Mx My Mz] = "
            f"[{test_wrench[0]: .3f} {test_wrench[1]: .3f} {test_wrench[2]: .3f} "
            f"{test_wrench[3]: .3f} {test_wrench[4]: .3f} {test_wrench[5]: .3f}]"
        )
        print("u (raw) = [" + " ".join(f"{value: .4f}" for value in u_test) + "]")
        print("u_sat   = [" + " ".join(f"{value: .4f}" for value in u_sat_test) + "]")
        print("PWM     = [" + " ".join(f"{value: .0f}" for value in pwm_test) + "]")
        print(
            "Achieved F_net  = "
            f"[{f_net_test[0]: .4f} {f_net_test[1]: .4f} {f_net_test[2]: .4f}]"
        )
        print(
            "Achieved tau_net= "
            f"[{tau_net_test[0]: .4f} {tau_net_test[1]: .4f} {tau_net_test[2]: .4f}]"
        )
        plot_case(name, u_test, pwm_test, f_net_test, tau_net_test)

    print("\n=============================================================")
    plt.show()


if __name__ == "__main__":
    main()
