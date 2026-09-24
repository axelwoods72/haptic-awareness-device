"""
Live 3D orientation visualizer for the haptic headband IMU.

Reads "ORIENT,<roll>,<pitch>,<yaw>" lines over serial (printed by main.c)
and draws a rotating box in real time so you can sanity-check that the
complementary filter's roll/pitch/yaw actually track how you're moving
the board.

Setup:
    pip install pyserial matplotlib numpy

Usage:
    1. Set SERIAL_PORT below to match your board (check with `ls /dev/cu.*`).
    2. Flash + run the firmware so it's printing ORIENT lines.
    3. python visualize_imu.py
"""

import re
import threading

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Configuration -----------------------------------------------------

SERIAL_PORT = "/dev/cu.usbmodemXXXX"  # <-- change this to your board's port
BAUD_RATE = 115200

# --- Serial reading (runs in a background thread) -----------------------

# Matches lines like: ORIENT,-3.21,12.50,178.04
LINE_RE = re.compile(r"ORIENT,(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)")

latest = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
lock = threading.Lock()


def serial_reader():
    """Continuously reads serial lines and updates `latest`.

    Runs on its own thread so the matplotlib animation loop never blocks
    waiting on serial data - it just reads whatever `latest` holds each
    time it redraws.
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
            with lock:
                latest["roll"] = roll
                latest["pitch"] = pitch
                latest["yaw"] = yaw


# --- Board geometry ------------------------------------------------------

# A flat rectangular box, roughly board-shaped, centered at the origin.
# x = length, y = width, z = height (thickness).
L, W, H = 2.0, 1.2, 0.3
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

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


def update(frame):
    ax.cla()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    with lock:
        roll, pitch, yaw = latest["roll"], latest["pitch"], latest["yaw"]

    R = rotation_matrix(roll, pitch, yaw)
    rotated = vertices @ R.T

    faces = [[rotated[i] for i in face] for face in faces_idx]
    box = Poly3DCollection(faces, facecolors=face_colors, edgecolors="black", alpha=0.9)
    ax.add_collection3d(box)

    ax.set_title(f"roll: {roll:6.1f}   pitch: {pitch:6.1f}   yaw: {yaw:6.1f}")


if __name__ == "__main__":
    threading.Thread(target=serial_reader, daemon=True).start()
    ani = FuncAnimation(fig, update, interval=50)
    plt.show()
