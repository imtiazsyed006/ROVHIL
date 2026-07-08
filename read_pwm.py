#!/usr/bin/env python3
"""
read_pwm.py — Read PWM + torque debug data from ZedBoard over UART.

Supports two FPGA output formats (auto-detected by value count per line):

  Format A — 11 values: D0..D7 Mx My Mz
    First 8: duty-scaled (0..10000), duty% = value / 100.0
    Last 3:  torques (Q16.16), N·m = value / 65536.0

  Format B —  8 values: D0..D7 (duty only, legacy)

Logs all data to a MATLAB .mat file (structure format) readable by
Simulink's "From File" block.

Usage:
  python read_pwm.py [COM_PORT] [BAUD]
    COM_PORT  – default COM5
    BAUD      – default 115200
"""

import sys
import os
import csv
import serial
from datetime import datetime

Q16_SCALE = 65536.0
CSV_FILENAME = "fpga_data.csv"


def parse_line(line: str) -> list[int] | None:
    """Parse one line of space-separated hex words. Returns list of ints or None."""
    parts = line.strip().split()
    if not parts:
        return None
    try:
        return [int(p, 16) for p in parts]
    except ValueError:
        return None


def q16_to_float(v: int) -> float:
    """Convert signed Q16.16 hex to float."""
    if v >= 0x80000000:
        return (v - 0x100000000) / Q16_SCALE
    return v / Q16_SCALE


def save_csv(filename: str, data: list, signal_names: list[str]):
    """Save data to CSV file with header."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["time_s"] + signal_names)
        writer.writerows(data)
    print(f"  [Saved {len(data)} samples to {filename}]")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port} @ {baud} baud ... ", end="", flush=True)

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
        )
    except serial.SerialException as e:
        print(f"FAILED\n  {e}")
        return 1

    print("connected.")
    print(f"Logging to: {CSV_FILENAME}")

    sample_count = 0
    error_count  = 0
    detected_fmt = None  # '11val' or '8val'
    signal_names = None

    # Rolling buffer for CSV
    csv_rows = []    # each row: [time_s, val0, val1, ...]

    try:
        while True:
            line_bytes = ser.readline()
            if not line_bytes:
                continue

            try:
                line = line_bytes.decode("ascii", errors="replace").strip()
            except UnicodeDecodeError:
                error_count += 1
                continue

            if not line:
                continue

            values = parse_line(line)
            if values is None:
                error_count += 1
                if error_count <= 3:
                    print(f"  [malformed: {line[:80]}...]")
                continue

            # Auto-detect format on first valid line
            if detected_fmt is None:
                if len(values) == 11:
                    detected_fmt = "11val"
                    signal_names = [f"D{i}" for i in range(8)] + ["Mx", "My", "Mz"]
                    print(f"Detected: 11-value format "
                          f"(D0..D7 + Mx,My,Mz Q16.16)")
                    # Print header
                    hdr = f"{'Time':>12s}"
                    for i in range(8):
                        hdr += f"  {'D'+str(i):>7s}"
                    hdr += f"  {'Mx(N·m)':>9s}  {'My(N·m)':>9s}  {'Mz(N·m)':>9s}"
                    print(hdr)
                    print("-" * 12 + "  " + "-" * 7 * 8 +
                          "  " + "-" * 9 + "  " + "-" * 9 + "  " + "-" * 9)
                elif len(values) == 8:
                    detected_fmt = "8val"
                    signal_names = [f"D{i}" for i in range(8)]
                    print("Detected: 8-value duty format (0..10000)")
                    hdr = f"{'Time':>12s}"
                    for ch in range(8):
                        hdr += f"  {'CH'+str(ch):>7s}"
                    print(hdr)
                    print("-" * 12 + "  " + "-" * 7 * 8)
                elif len(values) == 3:
                    detected_fmt = "3val"
                    signal_names = ["Mx", "My", "Mz"]
                    print("Detected: 3-value torque format (Q16.16)")
                    hdr = f"{'Time':>12s}"
                    hdr += f"  {'Mx(N·m)':>9s}  {'My(N·m)':>9s}  {'Mz(N·m)':>9s}"
                    print(hdr)
                    print("-" * 12 + "  " + "-" * 9 + "  " + "-" * 9 + "  " + "-" * 9)
                else:
                    error_count += 1
                    if error_count <= 3:
                        print(f"  [unexpected: {len(values)} values]")
                    continue

            # Skip lines that don't match the detected format
            expected = {"11val": 11, "8val": 8, "3val": 3}
            if len(values) != expected.get(detected_fmt, 0):
                error_count += 1
                continue

            sample_count += 1
            now_dt = datetime.now()
            now_str = now_dt.strftime("%H:%M:%S")
            # Time in seconds since start
            if csv_rows:
                t = (now_dt - t0).total_seconds()
            else:
                t0 = now_dt
                t = 0.0

            if detected_fmt == "11val":
                duties = values[:8]
                torques = values[8:11]

                # Build display
                parts = [f"{now_str:>12s}"]
                for v in duties:
                    if v > 0:
                        parts.append(f"{v/100.0:6.2f}%")
                    else:
                        parts.append("  ---  ")
                for v in torques:
                    parts.append(f"{q16_to_float(v):9.4f}")
                print("  ".join(parts))

                # Store: duties as %, torques as N·m
                row = [t] + [v / 100.0 for v in duties] + [q16_to_float(v) for v in torques]

            elif detected_fmt == "3val":
                parts = [f"{now_str:>12s}"]
                for v in values:
                    parts.append(f"{q16_to_float(v):9.4f}")
                print("  ".join(parts))
                row = [t] + [q16_to_float(v) for v in values]

            else:  # 8val (duty only)
                parts = [f"{now_str:>12s}"]
                for v in values:
                    if v > 0:
                        parts.append(f"{v/100.0:6.2f}%")
                    else:
                        parts.append("  ---  ")
                print("  ".join(parts))
                row = [t] + [v / 100.0 for v in values]

            csv_rows.append(row)

            # Save CSV every 50 samples
            if sample_count % 50 == 0:
                save_csv(CSV_FILENAME, csv_rows, signal_names)

    except KeyboardInterrupt:
        print(f"\n\nStopped.  {sample_count} samples read, "
              f"{error_count} lines skipped.")

        # Final save
        if csv_rows:
            save_csv(CSV_FILENAME, csv_rows, signal_names)
    finally:
        ser.close()
        print("Serial port closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
