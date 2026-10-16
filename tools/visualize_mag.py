"""
Live magnetometer calibration viewer for the haptic headband.

Reads "MAG,<name>,<x>,<y>,<z>" lines over serial (raw field in mG, i.e. BEFORE
the hard-iron offset is subtracted - see the printf in main.c's imu_update)
and, for each IMU, plots the three 2D projections of every sample seen so far
(XY, XZ, YZ) with the estimated centre and fitted sphere outline.

How to read it:
  * A calibrated magnetometer traces a sphere centred on (0, 0, 0). Hard-iron
    error shifts that sphere - the shift IS the offset you need to subtract.
  * Rotate the board slowly through every orientation (spin it flat, tip it on
    each side and spin, flip it over) until each projection shows a filled-in
    ring/disc rather than a single arc.
  * The text panel shows two offset estimates per axis:
      - min/max midpoint  (simple, needs you to actually hit both extremes)
      - sphere fit        (least squares, works even with partial coverage)
    When coverage is good the two agree to within ~10-20 mG.
  * Half-range / fit radius should be roughly the local field strength
    (~550-600 mG in Melbourne). A much smaller half-range on one axis means you
    haven't pointed that axis along the field yet - keep rotating.
  * Press 'c' to print copy-paste C lines for the offsets, 'r' to reset.

Setup:
    pip install pyserial matplotlib numpy

Usage:
    1. In main.c set MAG_CAL_PRINT to 1 so the firmware prints MAG lines.
    2. Close idf.py monitor (only one program can hold the serial port).
    3. python visualize_mag.py            (or: python visualize_mag.py -p /dev/cu.xxx)
"""

import argparse
import re
import threading
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

# matplotlib binds 'r' (reset view) and 'c' (back) by default, which fights with
# this tool's reset / print keys - remove those default bindings
for _key, _binding in (("keymap.home", "r"), ("keymap.back", "c")):
    if _binding in plt.rcParams[_key]:
        plt.rcParams[_key].remove(_binding)

# --- Configuration -----------------------------------------------------

SERIAL_PORT = "/dev/cu.usbserial-0001"  # <-- change to your board's port (ls /dev/cu.*)
BAUD_RATE = 115200
IMU_NAMES = ["head", "body"]  # must match the names passed to imu_update() in main.c
MAX_POINTS = 4000             # samples kept per IMU (50 Hz -> ~80 s)
EXPECTED_FIELD_MG = 580       # rough total Earth field in Melbourne, for the coverage hint
MIN_FIT_POINTS = 50           # don't attempt a sphere fit before this many samples
MIN_FIT_SPREAD_MG = 150       # ...or before at least 2 axes have swept this much (half-range)
MAX_PLAUSIBLE_MG = 3000       # reject corrupt reads (bus glitches, empty registers = -32768)

# --- Serial reading (background thread) --------------------------------

# Matches lines like: MAG,head,-123.4,456.7,-89.0
LINE_RE = re.compile(r"MAG,(\w+),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)")

samples = {name: deque(maxlen=MAX_POINTS) for name in IMU_NAMES}
rejected = {name: 0 for name in IMU_NAMES}  # corrupt samples dropped, shown in the text panel
lock = threading.Lock()


def serial_reader(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Connected to {port} at {baud} baud. Rotate each IMU through every orientation...")
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("Serial connection lost - is the board still plugged in?")
            break
        match = LINE_RE.search(raw.decode("utf-8", errors="ignore"))
        if not match:
            continue
        name = match.group(1)
        if name not in samples:
            continue
        xyz = tuple(float(v) for v in match.groups()[1:])
        with lock:
            if max(abs(v) for v in xyz) > MAX_PLAUSIBLE_MG:
                rejected[name] += 1  # corrupt read, e.g. 0xFF bytes or empty registers
                continue
            samples[name].append(xyz)


# --- Offset estimation ---------------------------------------------------


def minmax_estimate(pts):
    """Offset = midpoint of min/max per axis, half-range = (max - min) / 2."""
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (hi + lo) / 2.0, (hi - lo) / 2.0, lo, hi


def sphere_fit(pts):
    """Least-squares sphere fit.

    |p - c|^2 = r^2 expands to  x^2+y^2+z^2 = 2cx*x + 2cy*y + 2cz*z + (r^2 - |c|^2),
    which is linear in (cx, cy, cz, k). Solve, then r = sqrt(k + |c|^2).
    Returns (centre, radius) or (None, None) if the fit is ill-conditioned.
    """
    if len(pts) < MIN_FIT_POINTS:
        return None, None
    # a board sitting still gives a tiny noisy blob - fitting a sphere to that produces
    # nonsense (huge radius), so wait until the data actually spans part of a sphere
    half = (pts.max(axis=0) - pts.min(axis=0)) / 2.0
    if np.sum(half > MIN_FIT_SPREAD_MG) < 2:
        return None, None
    A = np.column_stack([2.0 * pts, np.ones(len(pts))])
    b = (pts ** 2).sum(axis=1)
    sol, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 4:
        return None, None
    centre = sol[:3]
    r2 = sol[3] + centre @ centre
    if r2 <= 0:
        return None, None
    radius = float(np.sqrt(r2))
    if radius > MAX_PLAUSIBLE_MG:
        return None, None  # still ill-conditioned (e.g. data is nearly flat) - keep rotating
    return centre, radius


def current_estimates():
    """Snapshot every IMU's samples and compute both estimates."""
    out = {}
    with lock:
        snap = {name: np.array(s) for name, s in samples.items()}
        rej = dict(rejected)
    for name, pts in snap.items():
        if len(pts) < 2:
            out[name] = None if rej[name] == 0 else dict(pts=None, rejected=rej[name])
            continue
        mm_off, half, lo, hi = minmax_estimate(pts)
        fit_c, fit_r = sphere_fit(pts)
        out[name] = dict(pts=pts, mm_off=mm_off, half=half, lo=lo, hi=hi,
                         fit_c=fit_c, fit_r=fit_r, rejected=rej[name])
    return out


def c_code(estimates):
    """Copy-paste lines for main.c (prefers the sphere fit when available)."""
    lines = []
    for name, est in estimates.items():
        if est is None or est["pts"] is None:
            continue
        off = est["fit_c"] if est["fit_c"] is not None else est["mm_off"]
        for i, axis in enumerate("xyz"):
            lines.append(f"imu_{name}.mag_offset[{i}] = {off[i]:.1f}f; // {axis}")
    return "\n".join(lines)


# --- Plotting --------------------------------------------------------------

PROJECTIONS = [(0, 1, "X", "Y"), (0, 2, "X", "Z"), (1, 2, "Y", "Z")]

fig, axes = plt.subplots(
    len(IMU_NAMES), 4, figsize=(15, 4.2 * len(IMU_NAMES)),
    gridspec_kw={"width_ratios": [1, 1, 1, 1.15]}, squeeze=False,
)
fig.canvas.manager.set_window_title("Magnetometer calibration")


def coverage_hint(half):
    weak = [axis for axis, h in zip("XYZ", half) if h < 0.7 * EXPECTED_FIELD_MG]
    if not weak:
        return "coverage: good"
    return f"coverage: keep rotating ({', '.join(weak)} not swept fully)"


def update(_frame):
    estimates = current_estimates()
    for row, name in enumerate(IMU_NAMES):
        est = estimates[name]
        text_ax = axes[row][3]
        text_ax.cla()
        text_ax.axis("off")

        for col, (i, j, li, lj) in enumerate(PROJECTIONS):
            ax = axes[row][col]
            ax.cla()
            ax.set_title(f"{name}: {li}-{lj}")
            ax.set_xlabel(f"{li} (mG)")
            ax.set_ylabel(f"{lj} (mG)")
            ax.axhline(0, color="0.8", lw=0.8)
            ax.axvline(0, color="0.8", lw=0.8)
            if est is None or est["pts"] is None:
                continue
            pts = est["pts"]
            ax.scatter(pts[:, i], pts[:, j], s=3, alpha=0.35, color="tab:blue")
            ax.scatter(pts[-1, i], pts[-1, j], s=30, color="tab:red", zorder=3)  # latest sample
            # min/max centre
            ax.plot(est["mm_off"][i], est["mm_off"][j], "k+", ms=14, mew=2, label="min/max centre")
            # sphere fit centre + outline (a sphere's 2D projection is a circle of the same radius)
            if est["fit_c"] is not None:
                c = est["fit_c"]
                ax.plot(c[i], c[j], "x", color="tab:green", ms=12, mew=2, label="sphere-fit centre")
                ax.add_patch(Circle((c[i], c[j]), est["fit_r"], fill=False, color="tab:green", lw=1.2))
            # explicit, equal-scale limits around the points (and the fit circle if valid),
            # so a stray patch can never zoom the view out until the data vanishes
            xs, ys = [pts[:, i].min(), pts[:, i].max()], [pts[:, j].min(), pts[:, j].max()]
            if est["fit_c"] is not None:
                c, r = est["fit_c"], est["fit_r"]
                xs += [c[i] - r, c[i] + r]
                ys += [c[j] - r, c[j] + r]
            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            half_span = max(max(xs) - min(xs), max(ys) - min(ys), 100.0) / 2 * 1.15
            ax.set_xlim(cx - half_span, cx + half_span)
            ax.set_ylim(cy - half_span, cy + half_span)
            ax.set_aspect("equal", adjustable="box")
            if col == 0:
                ax.legend(loc="upper right", fontsize=7)

        if est is None:
            text_ax.text(0, 0.5, f"{name}: waiting for MAG,{name},... lines", fontsize=10, family="monospace")
            continue
        if est["pts"] is None:
            text_ax.text(0, 0.5, f"{name}: {est['rejected']} samples received, all rejected as\n"
                         f"corrupt (|value| > {MAX_PLAUSIBLE_MG} mG). Did {name} IMU setup fail?",
                         fontsize=10, family="monospace")
            continue

        lines = [f"{name.upper()}   n = {len(est['pts'])}   (rejected {est['rejected']})", ""]
        lines.append("axis    min     max   offset  half-rng")
        for k, axis in enumerate("XYZ"):
            lines.append(
                f"  {axis}  {est['lo'][k]:7.1f} {est['hi'][k]:7.1f} {est['mm_off'][k]:7.1f}  {est['half'][k]:7.1f}"
            )
        lines.append("")
        if est["fit_c"] is not None:
            c = est["fit_c"]
            lines.append("sphere fit:")
            lines.append(f"  offset  {c[0]:7.1f} {c[1]:7.1f} {c[2]:7.1f}")
            lines.append(f"  radius  {est['fit_r']:7.1f} mG")
        else:
            lines.append("sphere fit: keep rotating (data too flat/small to fit)")
        lines.append("")
        lines.append(coverage_hint(est["half"]))
        lines.append(f"(expect half-range/radius ~{EXPECTED_FIELD_MG} mG)")
        text_ax.text(0, 1, "\n".join(lines), va="top", fontsize=9.5, family="monospace")

    fig.tight_layout()


def on_key(event):
    if event.key == "r":
        with lock:
            for s in samples.values():
                s.clear()
            for name in rejected:
                rejected[name] = 0
        print("Reset - start rotating again.")
    elif event.key == "c":
        code = c_code(current_estimates())
        print("\n// paste into app_main after imu_driver_setup():\n" + (code or "// no data yet") + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live magnetometer calibration viewer")
    parser.add_argument("-p", "--port", default=SERIAL_PORT)
    parser.add_argument("-b", "--baud", type=int, default=BAUD_RATE)
    args = parser.parse_args()

    threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True).start()
    fig.canvas.mpl_connect("key_press_event", on_key)
    ani = FuncAnimation(fig, update, interval=250, cache_frame_data=False)
    plt.show()

    # on window close, print the final offsets too
    final = c_code(current_estimates())
    if final:
        print("\n// final offsets:\n" + final)
