"""
Live 3D orientation visualizer for the haptic headband IMU.

Reads "ORIENT,<roll>,<pitch>,<yaw>" lines over serial (printed by main.c)
and draws a rotating box in real time so you can sanity-check that the
complementary filter's roll/pitch/yaw actually track how you're moving
the board.

Also plots a rolling time-series of roll/pitch/yaw alongside the box.
The 3D box alone only shows you the latest sample - it's fine for a quick
"does this look roughly right" check, but you can't judge noise floor or
settling time from a single rotating cube. The time-series panel is what
you actually want when tuning the complementary filter's ALPHA: watch the
line's wobble when the board is held still (noise floor - lower ALPHA
means more wobble here, since accel/mag are weighted more) and how long
it takes to settle after a known rotation (step response - higher ALPHA
means slower settling here).

Note: roll/pitch/yaw are unwrapped for display, so a real physical
rotation past +-180 shows as a continuous ramp in the plot rather than a
vertical jump - don't mistake a genuine wrap-around in the raw serial
data for filter noise.

Setup:
    pip install pyserial matplotlib numpy

Usage:
    1. Set SERIAL_PORT below to match your board (check with `ls /dev/cu.*`).
    2. Flash + run the firmware so it's printing ORIENT lines.
    3. python visualize_imu.py
"""

import re
import threading
import time
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Configuration -----------------------------------------------------

SERIAL_PORT = "/dev/cu.usbserial-0001"  # <-- change this to your board's port
BAUD_RATE = 115200
HISTORY_SECONDS = 10  # how much time-series history to keep and show

# --- Serial reading (runs in a background thread) -----------------------

# Matches lines like: ORIENT,-3.21,12.50,178.04
LINE_RE = re.compile(r"ORIENT,(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)")

latest = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
lock = threading.Lock()

start_time = time.monotonic()
# Each entry: (elapsed_seconds, roll, pitch, yaw). Bounded to the last
# HISTORY_SECONDS so the plot is a rolling window, not the whole run.
history = deque()


def serial_reader():
    """Continuously reads serial lines and updates `latest` and `history`.

    Runs on its own thread so the matplotlib animation loop never blocks
    waiting on serial data - it just reads whatever `latest`/`history`
    holds each time it redraws.
    """
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud. Reading...")
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("Serial connection lost - is the board still plugged in?")
            break

        line = raw.decode("utf-8", errors="ignore").strip()
        match = LINE_RE.search(line)
        if match:
            roll, pitch, yaw = map(float, match.groups())
            elapsed = time.monotonic() - start_time
            with lock:
                latest["roll"] = roll
                latest["pitch"] = pitch
                latest["yaw"] = yaw
                history.append((elapsed, roll, pitch, yaw))
                while history and elapsed - history[0][0] > HISTORY_SECONDS:
                    history.popleft()


# --- Board geometry ------------------------------------------------------

# A flat rectangular box, roughly board-shaped, centered at the origin.
# x = length, y = width, z = height (thickness).
L, W, H = 1.2, 2.0, 0.3
vertices = np.array(
    [
        [-L / 2, -W / 2, -H / 2],
        [L / 2, -W / 2, -H / 2],
        [L / 2, W / 2, -H / 2],
        [-L / 2, W / 2, -H / 2],
        [-L / 2, -W / 2, H / 2],
        [L / 2, -W / 2, H / 2],
        [L / 2, W / 2, H / 2],
        [-L / 2, W / 2, H / 2],
    ]
)

faces_idx = [
    [0, 1, 2, 3],  # bottom
    [4, 5, 6, 7],  # top
    [0, 1, 5, 4],  # front (+x end)
    [2, 3, 7, 6],  # back  (-x end)
    [1, 2, 6, 5],  # right
    [0, 3, 7, 4],  # left
]

# Top face is red so you can always tell which side is facing up.
# Front face is blue so you can tell which end is "forward".
face_colors = ["gray", "red", "blue", "lightgray", "lightgray", "lightgray"]


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Builds a rotation matrix from roll/pitch/yaw in degrees.

    Applies yaw (about z), then pitch (about y), then roll (about x) -
    the standard aerospace Z-Y-X Euler convention. This only needs to be
    internally consistent for the visual to look right; it doesn't need
    to exactly match your filter's internal math.
    """
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])

    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


# --- Plotting --------------------------------------------------------------

fig = plt.figure(figsize=(11, 5))
ax_box = fig.add_subplot(121, projection="3d")
ax_hist = fig.add_subplot(122)


def update(frame):
    with lock:
        roll, pitch, yaw = latest["roll"], latest["pitch"], latest["yaw"]
        hist_snapshot = list(history)

    # --- 3D box: latest orientation only ---
    ax_box.cla()
    ax_box.set_xlim(-2, 2)
    ax_box.set_ylim(-2, 2)
    ax_box.set_zlim(-2, 2)
    ax_box.set_box_aspect([1, 1, 1])
    ax_box.set_xlabel("x")
    ax_box.set_ylabel("y")
    ax_box.set_zlabel("z")

    R = rotation_matrix(roll, pitch, yaw)
    rotated = vertices @ R.T
    faces = [[rotated[i] for i in face] for face in faces_idx]
    box = Poly3DCollection(faces, facecolors=face_colors, edgecolors="black", alpha=0.9)
    ax_box.add_collection3d(box)
    ax_box.set_title(f"roll: {roll:6.1f}   pitch: {pitch:6.1f}   yaw: {yaw:6.1f}")

    # --- Rolling time-series: noise floor + settling time ---
    ax_hist.cla()
    if len(hist_snapshot) >= 2:
        t = np.array([h[0] for h in hist_snapshot])
        # period=360 unwraps the +-180 deg wrap into a continuous ramp,
        # so a real spin doesn't look like a noise spike in the plot.
        roll_hist = np.unwrap([h[1] for h in hist_snapshot], period=360)
        pitch_hist = np.unwrap([h[2] for h in hist_snapshot], period=360)
        yaw_hist = np.unwrap([h[3] for h in hist_snapshot], period=360)
        t_now = t[-1]

        ax_hist.plot(t, roll_hist, label="roll", color="tab:blue")
        ax_hist.plot(t, pitch_hist, label="pitch", color="tab:orange")
        ax_hist.plot(t, yaw_hist, label="yaw", color="tab:green")
        ax_hist.set_xlim(max(0.0, t_now - HISTORY_SECONDS), t_now)

        all_vals = np.concatenate([roll_hist, pitch_hist, yaw_hist])
        margin = 10.0
        span_half = max((all_vals.max() - all_vals.min()) / 2 + margin, 20.0)
        mid = (all_vals.max() + all_vals.min()) / 2
        ax_hist.set_ylim(mid - span_half, mid + span_half)

    ax_hist.set_xlabel("time (s)")
    ax_hist.set_ylabel("degrees (unwrapped)")
    ax_hist.set_title(f"last {HISTORY_SECONDS}s - watch noise (still) / settling (after a move)")
    ax_hist.legend(loc="upper right")
    ax_hist.grid(True, alpha=0.3)

    fig.tight_layout()


if __name__ == "__main__":
    threading.Thread(target=serial_reader, daemon=True).start()
    ani = FuncAnimation(fig, update, interval=50)
    plt.show()
