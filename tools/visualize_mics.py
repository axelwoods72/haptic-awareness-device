"""
Live polar visualiser for the mic array direction-of-arrival angle.

Reads the ESP32 serial log, picks out the "Theta: <deg>" lines printed by
app_main(), and draws a line from the origin at that angle on a circle.

Angle convention matches tau_to_angle() in mic_driver.c:
    theta = atan2(x, y) in degrees, range [-180, 180]
    0 deg  = mic0 (+y, drawn at the top)
    +deg   = clockwise (towards +x)

Usage:
    pip install pyserial matplotlib
    python tools/visualize_mics.py COM5
    python tools/visualize_mics.py COM5 --baud 115200 --trail 10

Close `idf.py monitor` first - only one program can hold the serial port.
"""

import argparse
import math
import re
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

THETA_RE = re.compile(r"Theta:\s*(-?\d+(?:\.\d+)?)")

# mic bearings from mic_driver.c (degrees, clockwise from +y)
MIC_ANGLES = {"mic0": 0.0, "mic1": -120.0, "mic2": 120.0}


def serial_reader(port, baud, readings, stop):
    with serial.Serial(port, baud, timeout=0.1) as ser:
        while not stop.is_set():
            line = ser.readline().decode("utf-8", errors="ignore")
            if not line:
                continue
            m = THETA_RE.search(line)
            if m:
                readings.append((time.time(), float(m.group(1))))


def main():
    parser = argparse.ArgumentParser(description="Mic array angle visualiser")
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--trail", type=int, default=8,
                        help="number of previous angles to show faded")
    args = parser.parse_args()

    readings = deque(maxlen=args.trail + 1)
    stop = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(args.port, args.baud, readings, stop),
                              daemon=True)
    reader.start()

    fig = plt.figure(figsize=(6, 6.5))
    ax = fig.add_subplot(projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)  # clockwise, so +theta goes towards +x
    ax.set_ylim(0, 1)
    ax.set_yticklabels([])
    ax.set_xticks([math.radians(a) for a in range(0, 360, 30)])
    ax.set_xticklabels([f"{a if a <= 180 else a - 360}°" for a in range(0, 360, 30)])

    for name, ang in MIC_ANGLES.items():
        r = math.radians(ang)
        ax.plot(r, 0.15, "s", color="grey", markersize=8)
        ax.annotate(name, (r, 0.15), xytext=(r, 0.27), ha="center",
                    va="center", color="grey", fontsize=9)

    trail_lines = [ax.plot([], [], color="tab:blue", lw=2)[0]
                   for _ in range(args.trail)]
    (main_line,) = ax.plot([], [], color="tab:red", lw=3)
    (tip,) = ax.plot([], [], "o", color="tab:red", markersize=10)
    title = ax.set_title("waiting for data...", pad=25, fontsize=14)
    age_text = fig.text(0.5, 0.03, "", ha="center", color="grey")

    def update(_):
        if not reader.is_alive() and not readings:
            title.set_text(f"serial port {args.port} closed / unavailable")
            return

        snapshot = list(readings)
        if not snapshot:
            return

        ts, theta = snapshot[-1]
        r = math.radians(theta)
        main_line.set_data([r, r], [0, 1])
        tip.set_data([r], [1])
        title.set_text(f"θ = {theta:+.1f}°")
        age_text.set_text(f"last update {time.time() - ts:.1f} s ago")

        older = snapshot[:-1][::-1]
        for i, line in enumerate(trail_lines):
            if i < len(older):
                ro = math.radians(older[i][1])
                line.set_data([ro, ro], [0, 0.9])
                line.set_alpha(0.5 * (1 - i / len(trail_lines)))
            else:
                line.set_data([], [])

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        stop.set()


if __name__ == "__main__":
    sys.exit(main())
