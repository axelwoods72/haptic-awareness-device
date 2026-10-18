"""
Live polar visualiser for the mic array direction-of-arrival angle.

Reads the ESP32 serial log and draws a line from the origin at the angle
estimated by the mic array alone (no IMU / belt correction). The line is only
drawn while a sound is active; it disappears once no reading has arrived for
--hold seconds.

Sources (pick with --source):
    smoothed  "Theta: <deg>"      app_main(), the averaged angle sent to the
                                  haptics. Printed at 100 Hz while sound is
                                  active, but only once the IMUs are zeroed.
    raw       "srp theta: <deg>"  mic_task, the per-frame SRP-PHAT angle.
                                  Printed for every frame above threshold,
                                  independent of the IMUs.

Angle convention matches srp_phat() / tau_to_angle() in mic_driver.c:
    range [-180, 180]
    0 deg  = mic0 (front, +y, drawn at the top)
    +deg   = clockwise (towards mic2, +x)

Usage:
    pip install pyserial matplotlib
    python tools/visualize_mics.py COM5
    python tools/visualize_mics.py COM5 --source raw --hold 0.2 --trail 10

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

NUM = r"(-?\d+(?:\.\d+)?)"
SOURCE_RE = {
    # case-sensitive "Theta:" so it doesn't match "srp theta:" or "Belt theta:"
    "smoothed": re.compile(r"Theta:\s*" + NUM),
    "raw": re.compile(r"srp theta:\s*" + NUM),
}

# mic bearings from get_mic_positions() in mic_driver.c (degrees, clockwise from +y)
MIC_ANGLES = {"mic0": 0.0, "mic1": -90.0, "mic2": 90.0}


def serial_reader(port, baud, pattern, readings, stop):
    with serial.Serial(port, baud, timeout=0.1) as ser:
        while not stop.is_set():
            line = ser.readline().decode("utf-8", errors="ignore")
            if not line:
                continue
            m = pattern.search(line)
            if not m:
                continue
            theta = float(m.group(1))
            # app_main repeats the same value every loop, only keep changes for the trail
            if readings and readings[-1][1] == theta:
                readings[-1] = (time.time(), theta)
            else:
                readings.append((time.time(), theta))


def main():
    parser = argparse.ArgumentParser(description="Mic array angle visualiser")
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source", choices=SOURCE_RE, default="smoothed",
                        help="which log line to plot (default: smoothed)")
    parser.add_argument("--hold", type=float, default=0.3,
                        help="seconds without a reading before the sound counts "
                             "as stopped (firmware hold is 0.25 s)")
    parser.add_argument("--trail", type=int, default=8,
                        help="number of previous angles to show faded")
    args = parser.parse_args()

    readings = deque(maxlen=args.trail + 1)
    stop = threading.Event()
    reader = threading.Thread(target=serial_reader,
                              args=(args.port, args.baud, SOURCE_RE[args.source],
                                    readings, stop),
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

    def clear_lines():
        main_line.set_data([], [])
        tip.set_data([], [])
        for line in trail_lines:
            line.set_data([], [])

    def update(_):
        if not reader.is_alive() and not readings:
            title.set_text(f"serial port {args.port} closed / unavailable")
            return

        now = time.time()
        # only readings from the current sound event
        snapshot = [(t, th) for t, th in readings if now - t < args.hold]
        if not snapshot:
            clear_lines()
            if readings:
                title.set_text("no sound")
                age_text.set_text(f"last sound {now - readings[-1][0]:.1f} s ago "
                                  f"at {readings[-1][1]:+.1f}°")
            return

        ts, theta = snapshot[-1]
        r = math.radians(theta)
        main_line.set_data([r, r], [0, 1])
        tip.set_data([r], [1])
        title.set_text(f"θ = {theta:+.1f}°")
        age_text.set_text(f"sound active ({args.source})")

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
