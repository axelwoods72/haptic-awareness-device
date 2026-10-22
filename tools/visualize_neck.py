"""
Live head + body IMU visualizer for the haptic headband.

Reads three line types over serial (printed by main.c every IMU loop):
    HEAD,<roll>,<pitch>,<yaw>     head IMU  (yaw already zeroed at startup;
                                  the older ORIENT,... name is accepted too)
    BODY,<roll>,<pitch>,<yaw>     body IMU  (yaw already zeroed at startup)
    NECK,<angle>                  head yaw relative to body yaw

and shows:
  * Left:  a top-down dial. Blue arrow = head, orange arrow = body, the shaded
           wedge between them is the neck angle. 0 deg is straight up (the
           pose the IMUs were zeroed in); positive yaw turns the arrow
           clockwise, matching how the yaw increased when you turned clockwise.
  * Top right:    head and body yaw over time (unwrapped, so a full spin is a
                  ramp rather than a jump at +-180).
  * Bottom right: neck angle over time. When only your head turns this should
                  move; when you turn your whole body (head + body together)
                  it should stay near 0 while both yaw lines move together.

Setup:
    pip install pyserial matplotlib numpy

Usage:
    1. Close idf.py monitor (only one program can hold the serial port).
    2. python visualize_neck.py            (or: python visualize_neck.py -p /dev/cu.xxx)
"""

import argparse
import re
import threading
import time
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Wedge

# --- Configuration -----------------------------------------------------

SERIAL_PORT = "/dev/cu.usbmodem101"  # <-- change to your board's port (ls /dev/cu.*)
BAUD_RATE = 115200
HISTORY_SECONDS = 15  # time-series window

HEAD_COLOR = "tab:blue"
BODY_COLOR = "tab:orange"
NECK_COLOR = "tab:green"

# --- Serial reading (background thread) --------------------------------

NUM = r"(-?\d+\.?\d*|nan|-?inf)"
ORIENT_RE = re.compile(rf"^(HEAD|ORIENT|BODY),{NUM},{NUM},{NUM}")
NECK_RE = re.compile(rf"^NECK,{NUM}")

latest = {
    "ORIENT": (0.0, 0.0, 0.0),  # head roll, pitch, yaw
    "BODY": (0.0, 0.0, 0.0),
    "NECK": 0.0,
}
seen_body = False
# Each entry: (t, head_yaw, body_yaw, neck). Appended on every NECK line, which
# main.c prints once per loop after ORIENT and BODY, so the three are in step.
history = deque()
lock = threading.Lock()
start_time = time.monotonic()


def serial_reader(port, baud):
    global seen_body
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Connected to {port} at {baud} baud. Reading...")
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("Serial connection lost - is the board still plugged in?")
            break
        line = raw.decode("utf-8", errors="ignore").strip()

        m = ORIENT_RE.match(line)
        if m:
            vals = tuple(map(float, m.groups()[1:]))
            key = "BODY" if m.group(1) == "BODY" else "ORIENT"  # HEAD and ORIENT both = head
            with lock:
                latest[key] = vals
                if key == "BODY":
                    seen_body = True
            continue

        m = NECK_RE.match(line)
        if m:
            neck = float(m.group(1))
            t = time.monotonic() - start_time
            with lock:
                latest["NECK"] = neck
                history.append((t, latest["ORIENT"][2], latest["BODY"][2], neck))
                while history and t - history[0][0] > HISTORY_SECONDS:
                    history.popleft()


# --- Plotting ----------------------------------------------------------

fig = plt.figure(figsize=(12, 6))
grid = fig.add_gridspec(2, 2, width_ratios=[1, 1.4])
ax_dial = fig.add_subplot(grid[:, 0])
ax_yaw = fig.add_subplot(grid[0, 1])
ax_neck = fig.add_subplot(grid[1, 1], sharex=ax_yaw)


def dial_xy(yaw_deg, r=1.0):
    """Point on the dial for a yaw: 0 deg = up, positive = clockwise."""
    a = np.radians(yaw_deg)
    return r * np.sin(a), r * np.cos(a)


def draw_dial(head, body, neck, have_body):
    ax_dial.cla()
    ax_dial.set_xlim(-1.3, 1.3)
    ax_dial.set_ylim(-1.3, 1.3)
    ax_dial.set_aspect("equal")
    ax_dial.axis("off")

    # outer ring + ticks every 30 deg, labels every 90
    ring = np.linspace(0, 2 * np.pi, 200)
    ax_dial.plot(np.cos(ring), np.sin(ring), color="0.7", lw=1)
    for d in range(0, 360, 30):
        x0, y0 = dial_xy(d, 0.93)
        x1, y1 = dial_xy(d, 1.0)
        ax_dial.plot([x0, x1], [y0, y1], color="0.6", lw=1)
    for d, label in ((0, "0"), (90, "+90"), (180, "180"), (-90, "-90")):
        x, y = dial_xy(d, 1.15)
        ax_dial.text(x, y, label, ha="center", va="center", color="0.4", fontsize=9)

    head_yaw = head[2]
    body_yaw = body[2]

    # neck wedge: from body arrow to head arrow, going the short way round
    if have_body and np.isfinite(neck):
        # Wedge angles are counter-clockwise from +x; convert from dial angles
        a_body = 90.0 - body_yaw
        a_head = a_body - neck  # positive neck = head clockwise of body
        lo, hi = sorted((a_body, a_head))
        ax_dial.add_patch(Wedge((0, 0), 0.55, lo, hi, color=NECK_COLOR, alpha=0.25))

    def arrow(yaw, color, length):
        if not np.isfinite(yaw):
            return
        x, y = dial_xy(yaw, length)
        ax_dial.annotate("", xy=(x, y), xytext=(0, 0),
                         arrowprops=dict(arrowstyle="-|>", color=color, lw=3,
                                         mutation_scale=20))

    if have_body:
        arrow(body_yaw, BODY_COLOR, 0.75)
    arrow(head_yaw, HEAD_COLOR, 0.9)

    title = f"head {head_yaw:7.1f}°"
    if have_body:
        title += f"   body {body_yaw:7.1f}°   neck {neck:7.1f}°"
    else:
        title += "   (no BODY lines yet - see main.c)"
    ax_dial.set_title(title, fontsize=10)
    ax_dial.text(0, -1.28,
                 f"head roll {head[0]:6.1f}  pitch {head[1]:6.1f}"
                 + (f"   |   body roll {body[0]:6.1f}  pitch {body[1]:6.1f}" if have_body else ""),
                 ha="center", va="center", fontsize=8, color="0.35")
    ax_dial.plot([], [], color=HEAD_COLOR, lw=3, label="head")
    ax_dial.plot([], [], color=BODY_COLOR, lw=3, label="body")
    ax_dial.legend(loc="upper left", frameon=False, fontsize=9)


def draw_history(hist, have_body):
    ax_yaw.cla()
    ax_neck.cla()
    if len(hist) >= 2:
        h = np.array(hist, dtype=float)
        h = h[np.all(np.isfinite(h), axis=1)]
    else:
        h = np.empty((0, 4))

    if len(h) >= 2:
        t = h[:, 0]
        head_yaw = np.unwrap(h[:, 1], period=360)
        body_yaw = np.unwrap(h[:, 2], period=360)
        neck = h[:, 3]

        ax_yaw.plot(t, head_yaw, color=HEAD_COLOR, label="head")
        if have_body:
            ax_yaw.plot(t, body_yaw, color=BODY_COLOR, label="body")
        ax_neck.plot(t, neck, color=NECK_COLOR)
        ax_neck.set_xlim(max(0.0, t[-1] - HISTORY_SECONDS), t[-1])

        vals = np.concatenate([head_yaw, body_yaw]) if have_body else head_yaw
        mid = (vals.max() + vals.min()) / 2
        half = max((vals.max() - vals.min()) / 2 + 10.0, 30.0)
        ax_yaw.set_ylim(mid - half, mid + half)

    ax_yaw.set_ylabel("yaw (deg, unwrapped)")
    ax_yaw.set_title("yaw over time - turn your whole body: lines move together")
    ax_yaw.grid(True, alpha=0.3)
    if ax_yaw.lines:
        ax_yaw.legend(loc="upper left", fontsize=9)
    plt.setp(ax_yaw.get_xticklabels(), visible=False)

    ax_neck.axhline(0, color="0.6", lw=1)
    ax_neck.set_ylim(-180, 180)
    ax_neck.set_yticks([-180, -90, 0, 90, 180])
    ax_neck.set_ylabel("neck (deg)")
    ax_neck.set_xlabel("time (s)")
    ax_neck.set_title("neck angle (head - body) - turn only your head: this moves")
    ax_neck.grid(True, alpha=0.3)


def update(_frame):
    with lock:
        head = latest["ORIENT"]
        body = latest["BODY"]
        neck = latest["NECK"]
        have_body = seen_body
        hist = list(history)
    draw_dial(head, body, neck, have_body)
    draw_history(hist, have_body)
    fig.tight_layout()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Head/body IMU + neck angle viewer")
    parser.add_argument("-p", "--port", default=SERIAL_PORT)
    parser.add_argument("-b", "--baud", type=int, default=BAUD_RATE)
    args = parser.parse_args()

    threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True).start()
    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
