"""
compare_golden.py -- Compare xsim testbench output (tb_out.txt) against the
Python golden reference (tau_ref.csv, y_ref.csv).

Reports per-axis RMSE / max-abs error for:
  - reconstructed torques   (FPGA duty->LUT->B chain  vs Python thruster_config)
  - simulated states        (FPGA RK4 Q16.16          vs Python RK4 float64)
Also writes fpga_vs_python.svg with the overlaid trajectories.
"""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def q16_to_float(u32):
    v = u32.astype(np.int64)
    v[v >= 2**31] -= 2**32
    return v / 65536.0


def main():
    raw = np.array([
        [int(w, 16) for w in line.split()]
        for line in (HERE / "simwork" / "tb_out.txt").read_text().split("\n")
        if line.strip()
    ], dtype=np.uint64)
    n = len(raw)
    print(f"tb samples: {n}")

    fp = q16_to_float(raw.astype(np.int64))
    tau_fpga = fp[:, 0:3]
    y_fpga = fp[:, 3:9]          # phi theta psi p q r

    tau_ref = np.loadtxt(HERE / "tau_ref.csv", delimiter=",", skiprows=1)[:n]
    y_ref = np.loadtxt(HERE / "y_ref.csv", delimiter=",", skiprows=1)[:n]

    print("\n--- torque reconstruction (N*m) ---")
    for i, nm in enumerate(["roll ", "pitch", "yaw  "]):
        d = tau_fpga[:, i] - tau_ref[:, i]
        print(f"tau_{nm}: rmse={np.sqrt(np.mean(d**2)):.5f}  max|d|={np.abs(d).max():.5f}  "
              f"(ref range {tau_ref[:, i].min():+.2f}..{tau_ref[:, i].max():+.2f})")

    print("\n--- angular rates (deg/s) ---")
    for i, nm in enumerate(["p", "q", "r"]):
        d = np.rad2deg(y_fpga[:, 3 + i] - y_ref[:, 3 + i])
        ref = np.rad2deg(y_ref[:, 3 + i])
        print(f"{nm}: rmse={np.sqrt(np.mean(d**2)):.4f}  max|d|={np.abs(d).max():.4f}  "
              f"(ref range {ref.min():+.1f}..{ref.max():+.1f}, ref rms {np.sqrt(np.mean(ref**2)):.2f})")

    print("\n--- attitude (deg, psi accumulates rate noise) ---")
    for i, nm in enumerate(["phi  ", "theta", "psi  "]):
        d = np.rad2deg(np.arctan2(np.sin(y_fpga[:, i] - y_ref[:, i]),
                                  np.cos(y_fpga[:, i] - y_ref[:, i])))
        print(f"{nm}: rmse={np.sqrt(np.mean(d**2)):.4f}  max|d|={np.abs(d).max():.4f}")

    # ------------------------------------------------------------------
    t = np.arange(n) * 0.02
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), constrained_layout=True)
    tau_lbl = ["roll", "pitch", "yaw"]
    rate_lbl = ["p", "q", "r"]
    for row in range(3):
        ax = axes[row, 0]
        ax.plot(t, tau_ref[:, row], "k", lw=0.6, label="Python")
        ax.plot(t, tau_fpga[:, row], "r", lw=0.5, alpha=0.7, label="FPGA (xsim)")
        ax.set_ylabel(f"tau_{tau_lbl[row]} (N*m)")
        ax.grid(alpha=0.25)
        if row == 0:
            ax.legend(fontsize=8)
            ax.set_title("Reconstructed input moment")
        ax = axes[row, 1]
        ax.plot(t, np.rad2deg(y_ref[:, 3 + row]), "k", lw=0.6, label="Python")
        ax.plot(t, np.rad2deg(y_fpga[:, 3 + row]), "r", lw=0.5, alpha=0.7, label="FPGA (xsim)")
        ax.set_ylabel(f"{rate_lbl[row]} (deg/s)")
        ax.grid(alpha=0.25)
        if row == 0:
            ax.legend(fontsize=8)
            ax.set_title("Simulated angular velocity")
        if row == 2:
            axes[row, 0].set_xlabel("Time (s)")
            axes[row, 1].set_xlabel("Time (s)")
    fig.suptitle("FPGA (fixed RTL, xsim) vs Python simulation.py", fontweight="bold")
    fig.savefig(HERE / "fpga_vs_python.svg")
    print(f"\nplot: {HERE / 'fpga_vs_python.svg'}")


if __name__ == "__main__":
    main()
