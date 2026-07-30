from __future__ import annotations

"""
spm_angle_comm.py — Live SPM simulation + periodic joint-angle streaming
=========================================================================

This script is the angle-command version of command_comm.py.
It reads the SpaceMouse, runs the same inverse kinematics used by
spm_live_sim.py, displays the live SPM, and periodically streams the current
simulated joint angles to spm_full.ino at a user-specified rate.

Protocol sent to Arduino:
    $S,<theta1_deg>,<theta2_deg>,<theta3_deg>,<enable>*<xor>\n
The theta values are the IK joint angles from the simulation in degrees.
The updated spm_full.ino converts those simulated joint angles into
absolute encoder targets using ENCODER_HOME_DEG[] and SIM_TO_ENCODER_SIGN[].

Run:
    python spm_angle_comm.py --port COM8

Controls:
    SpaceMouse motion       move the simulated SPM
    e                       enable/resume periodic angle streaming
    h                       hold current measured encoder position
    s                       stop/disable motors
    q or Esc                quit safely

Useful options:
    --send-hz 50            stream current angles to firmware 50 times/second
    --move-speed 1.0        scale SpaceMouse movement speed
    --start-disabled        start with streaming disabled until you press e

Dependencies:
    pip install numpy matplotlib pywinusb pyserial
"""

import argparse
import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import serial

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

try:
    import pywinusb.hid as hid
except ImportError:
    sys.exit("ERROR: pywinusb not installed. Run: pip install pywinusb")

from spm_kinematics import Geometry, rpy_to_R, inverse_kinematics, w_i, IKError


# ----------------------------------------------------------------------
# SpaceMouse HID reader, same basic approach as spm_live_sim.py
# ----------------------------------------------------------------------
VID_3DCONNEXION = 0x256F
VID_LOGITECH = 0x046D
SKIP_NAME_KEYWORDS = ("download assistant", "receiver", "g hub", "unifying")

_state = {"rx": 0, "ry": 0, "rz": 0, "buttons": 0}
_lock = threading.Lock()


def _i16(lo: int, hi: int) -> int:
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v


def _on_hid_data(data):
    if not data:
        return
    rid = data[0]
    with _lock:
        if rid == 1 and len(data) >= 13:
            _state["rx"] = _i16(data[7], data[8])
            _state["ry"] = _i16(data[9], data[10])
            _state["rz"] = _i16(data[11], data[12])
        elif rid == 2 and len(data) >= 7:
            _state["rx"] = _i16(data[1], data[2])
            _state["ry"] = _i16(data[3], data[4])
            _state["rz"] = _i16(data[5], data[6])
        elif rid == 3 and len(data) >= 2:
            b = data[1]
            if len(data) >= 3:
                b |= data[2] << 8
            _state["buttons"] = b


def _looks_like_spacemouse(d) -> bool:
    try:
        name = (d.product_name or "").lower()
    except Exception:
        name = ""
    return not any(kw in name for kw in SKIP_NAME_KEYWORDS)


def open_spacemouse():
    primary = hid.HidDeviceFilter(vendor_id=VID_3DCONNEXION).get_devices()
    fallback = [
        d for d in hid.HidDeviceFilter(vendor_id=VID_LOGITECH).get_devices()
        if _looks_like_spacemouse(d)
    ]
    dev = primary[0] if primary else (fallback[0] if fallback else None)
    if dev is None:
        sys.exit("ERROR: No SpaceMouse found.")
    print(f"Using {dev.product_name}")
    dev.open()
    dev.set_raw_data_handler(_on_hid_data)
    return dev


# ----------------------------------------------------------------------
# Keyboard helper copied from command_comm.py style
# ----------------------------------------------------------------------
class Keyboard:
    def __init__(self) -> None:
        self.is_windows = os.name == "nt"
        if self.is_windows:
            import msvcrt
            self.msvcrt = msvcrt
        else:
            import select
            import termios
            import tty
            self.select = select
            self.termios = termios
            self.tty = tty
            self.fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def close(self) -> None:
        if not self.is_windows:
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.old_settings)

    def get_key(self) -> Optional[str]:
        if self.is_windows:
            if self.msvcrt.kbhit():
                ch = self.msvcrt.getwch()
                if ch in ("\x00", "\xe0"):
                    if self.msvcrt.kbhit():
                        self.msvcrt.getwch()
                    return None
                return ch
            return None

        dr, _, _ = self.select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None


# ----------------------------------------------------------------------
# Serial protocol helpers
# ----------------------------------------------------------------------
def checksum_body(body: str) -> int:
    cs = 0
    for b in body.encode("ascii"):
        cs ^= b
    return cs


def make_packet(theta_deg: np.ndarray, enable: bool) -> str:
    body = f"S,{theta_deg[0]:.2f},{theta_deg[1]:.2f},{theta_deg[2]:.2f},{1 if enable else 0}"
    return f"${body}*{checksum_body(body):02X}"


def send_packet(ser: serial.Serial, theta_deg: np.ndarray, enable: bool) -> None:
    ser.write((make_packet(theta_deg, enable) + "\n").encode("ascii"))


def send_ascii(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd.strip() + "\n").encode("ascii"))


# ----------------------------------------------------------------------
# Simulation constants and drawing helpers from spm_live_sim.py
# ----------------------------------------------------------------------
FULL_SCALE = 350.0
DEADZONE = 0.02
RATE_GAIN = 0.04
DECAY = 0.0
RPY_LIMIT_RP = np.deg2rad((60.0, 60.0))
JOINT_CLEARANCE = np.deg2rad(60.0)
FRAME_HZ = 30


def joint_clash(theta: np.ndarray, geom: Geometry, clearance: float) -> bool:
    az = [0.5 * np.pi - geom.eta[i] + theta[i] for i in range(3)]
    for i in range(3):
        for j in range(i + 1, 3):
            d = az[i] - az[j]
            d = ((d + np.pi) % (2.0 * np.pi)) - np.pi
            if abs(d) < clearance:
                return True
    return False


def arc(p1: np.ndarray, p2: np.ndarray, n: int = 30) -> np.ndarray:
    p1 = np.asarray(p1)
    p2 = np.asarray(p2)
    omega = np.arccos(np.clip(np.dot(p1, p2), -1.0, 1.0))
    if omega < 1e-9:
        return np.tile(p1, (n, 1))
    t = np.linspace(0, 1, n)
    s = np.sin(omega)
    return (
        np.outer(np.sin((1 - t) * omega) / s, p1)
        + np.outer(np.sin(t * omega) / s, p2)
    )


def build_scene(ax, geom: Geometry):
    sphi, sth = np.mgrid[0:np.pi:25j, 0:2*np.pi:50j]
    xs = np.sin(sphi) * np.cos(sth)
    ys = np.sin(sphi) * np.sin(sth)
    zs = np.cos(sphi)
    ax.plot_wireframe(xs, ys, zs, color="lightgray", linewidth=0.3, alpha=0.3)
    ax.scatter([0], [0], [-1], color="k", s=80, label="base axis")
    prox = [ax.plot([], [], [], color="C0", linewidth=2.5)[0] for _ in range(3)]
    dist = [ax.plot([], [], [], color="C3", linewidth=2.5)[0] for _ in range(3)]
    plat, = ax.plot([], [], [], color="C2", linewidth=1.8)
    plat_n, = ax.plot([], [], [], color="C2", linewidth=2.5)
    w_dots = ax.scatter([], [], [], color="C0", s=35)
    v_dots = ax.scatter([], [], [], color="C3", s=35)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    return prox, dist, plat, plat_n, w_dots, v_dots


def update_scene(geom, R, theta, prox, dist, plat, plat_n, w_dots, v_dots):
    u = np.array([0.0, 0.0, -1.0])
    ws = np.array([w_i(geom.eta[i], theta[i], geom.alpha1) for i in range(3)])
    v_b = (R @ geom.v_platform().T).T
    n = R @ np.array([0.0, 0.0, 1.0])

    for i in range(3):
        a1 = arc(u, ws[i])
        prox[i].set_data(a1[:, 0], a1[:, 1])
        prox[i].set_3d_properties(a1[:, 2])
        a2 = arc(ws[i], v_b[i])
        dist[i].set_data(a2[:, 0], a2[:, 1])
        dist[i].set_3d_properties(a2[:, 2])

    tri = np.vstack([v_b, v_b[0]])
    plat.set_data(tri[:, 0], tri[:, 1])
    plat.set_3d_properties(tri[:, 2])
    plat_n.set_data([0, n[0]], [0, n[1]])
    plat_n.set_3d_properties([0, n[2]])
    w_dots._offsets3d = (ws[:, 0], ws[:, 1], ws[:, 2])
    v_dots._offsets3d = (v_b[:, 0], v_b[:, 1], v_b[:, 2])


@dataclass
class CommandState:
    enabled: bool = True
    target_theta_deg: Optional[np.ndarray] = None


def main() -> int:
    ap = argparse.ArgumentParser(description="Live SPM sim with periodic angle streaming to spm_full.ino")
    ap.add_argument("--port", required=True, help="Arduino serial port, e.g. COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--send-hz", type=float, default=25.0,
                    help="Rate at which current displayed joint angles are streamed to firmware")
    ap.add_argument("--move-speed", type=float, default=3.0,
                    help="Scale SpaceMouse movement speed. 1.0 keeps the original speed; 0.5 is half; 2.0 is double.")
    ap.add_argument("--rate-gain", type=float, default=RATE_GAIN,
                    help="Base roll/pitch/yaw increment per frame before --move-speed scaling")
    ap.add_argument("--start-disabled", action="store_true",
                    help="Start with periodic streaming disabled. Press e to enable/resume.")
    ap.add_argument("--link-clearance-deg", type=float, default=60.0,
                    help="Minimum allowed azimuth gap between proximal-link planes in the simulation")
    args = ap.parse_args()

    geom = Geometry()
    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    kb = Keyboard()
    dev = open_spacemouse()
    if args.send_hz <= 0:
        ap.error("--send-hz must be greater than 0")
    if args.move_speed < 0:
        ap.error("--move-speed must be non-negative")
    if args.rate_gain < 0:
        ap.error("--rate-gain must be non-negative")

    cmd = CommandState(enabled=not args.start_disabled)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    artists = build_scene(ax, geom)
    title = ax.set_title("")

    rpy = np.zeros(3)
    R = rpy_to_R(*rpy)
    theta = inverse_kinematics(R, geom)
    neutral_theta = theta.copy()
    neutral_theta_deg = np.rad2deg(neutral_theta)
    joint_clearance = np.deg2rad(args.link_clearance_deg)
    update_scene(geom, R, theta, *artists)

    # No click handler: commands are streamed periodically from the live pose.
    plt.ion()
    plt.show()

    print(f"Opened serial {args.port} @ {args.baud}")
    print("Move the SpaceMouse to pose the simulated SPM.")
    print(f"Streaming displayed theta angles at {args.send_hz:.2f} Hz" + (" after you press e." if args.start_disabled else "."))
    print(f"Movement speed scale: {args.move_speed:.3f}  base rate gain: {args.rate_gain:.5f}")
    print("Keys: e=enable/resume streaming, g=go home + reset sim neutral, h=hold measured position, s=stop/disable, q/Esc=quit")
    print(
        "Neutral IK theta_deg="
        f"[{neutral_theta_deg[0]:+.2f}, {neutral_theta_deg[1]:+.2f}, {neutral_theta_deg[2]:+.2f}] "
        "will be subtracted from every sent command. Sim-neutral sends [0, 0, 0]."
    )
    print()

    period = 1.0 / FRAME_HZ
    send_period = 1.0 / args.send_hz
    last_send = 0.0
    last_print = 0.0
    last_stream_print = 0.0

    try:
        time.sleep(1.5)  # Arduino/VESC boot window
        while plt.fignum_exists(fig.number) and dev.is_plugged():
            t0 = time.perf_counter()

            key = kb.get_key()
            if key:
                k = key.lower()
                if k in ("q", "\x1b"):
                    break
                if k == "s":
                    cmd.enabled = False
                    cmd.target_theta_deg = None
                    send_ascii(ser, "STOP")
                    print("Sent STOP; periodic streaming disabled. Press e to resume.")
                if k == "e":
                    cmd.enabled = True
                    print("Periodic angle streaming enabled")
                if k == "g":
                    # Command firmware to fixed encoder home and reset the simulation frame.
                    # After this, the neutral pose streams [0, 0, 0] instead of the
                    # absolute IK theta values, avoiding an unexpected spin on first input.
                    rpy[:] = 0.0
                    R = rpy_to_R(*rpy)
                    theta = inverse_kinematics(R, geom, prev_theta=neutral_theta)
                    neutral_theta = theta.copy()
                    neutral_theta_deg = np.rad2deg(neutral_theta)
                    cmd.enabled = False
                    cmd.target_theta_deg = None
                    send_ascii(ser, "GOHOME")
                    print("Sent GOHOME and reset sim neutral frame; streaming disabled. Press e to resume.")
                if k == "h":
                    cmd.enabled = False
                    cmd.target_theta_deg = None
                    send_ascii(ser, "HOLD")
                    print("Sent HOLD; periodic streaming disabled. Press e to resume.")

            with _lock:
                rx = _state["rx"] / FULL_SCALE
                ry = _state["ry"] / FULL_SCALE
                rz = _state["rz"] / FULL_SCALE
                btn = _state["buttons"]

            def dz(v):
                return 0.0 if abs(v) < DEADZONE else v

            d_roll = dz(ry)
            d_pitch = dz(rx)
            d_yaw = dz(rz)

            rpy_try = rpy.copy()
            move_gain = args.rate_gain * args.move_speed
            rpy_try[0] += d_roll * move_gain
            rpy_try[1] += d_pitch * move_gain
            rpy_try[2] += d_yaw * move_gain
            rpy_try *= (1.0 - DECAY)

            rpy_try[0] = np.clip(rpy_try[0], -RPY_LIMIT_RP[0], RPY_LIMIT_RP[0])
            rpy_try[1] = np.clip(rpy_try[1], -RPY_LIMIT_RP[1], RPY_LIMIT_RP[1])
            rpy_try[2] = ((rpy_try[2] + np.pi) % (2.0 * np.pi)) - np.pi

            R_try = rpy_to_R(*rpy_try)
            limit_hit = False
            try:
                theta_try = inverse_kinematics(R_try, geom, prev_theta=theta)
                if joint_clash(theta_try, geom, joint_clearance):
                    limit_hit = True
                else:
                    rpy = rpy_try
                    theta = theta_try
                    R = R_try
            except IKError:
                limit_hit = True

            theta_deg = np.rad2deg(theta)
            theta_cmd_deg = theta_deg - neutral_theta_deg

            if cmd.enabled:
                now_send = time.monotonic()
                if now_send - last_send >= send_period:
                    cmd.target_theta_deg = theta_cmd_deg.copy()
                    send_packet(ser, cmd.target_theta_deg, True)
                    last_send = now_send

                    # Print a low-rate heartbeat so the terminal confirms streaming
                    # without flooding stdout at high --send-hz values.
                    if now_send - last_stream_print > 1.0:
                        print(
                            "Streaming neutral-relative theta_deg="
                            f"[{cmd.target_theta_deg[0]:+.2f}, {cmd.target_theta_deg[1]:+.2f}, {cmd.target_theta_deg[2]:+.2f}]"
                        )
                        last_stream_print = now_send

            try:
                waiting = ser.in_waiting
                if waiting:
                    txt = ser.read(waiting).decode(errors="ignore")
                    lines = [ln for ln in txt.splitlines() if ln.strip()]
                    now = time.monotonic()
                    if lines and now - last_print > 0.10:
                        print(f"FW: {lines[-1]}")
                        last_print = now
            except Exception:
                pass

            update_scene(geom, R, theta, *artists)
            active_tag = "  [STREAMING]" if cmd.enabled else "  [DISABLED]"
            limit_tag = "  [LIMIT]" if limit_hit else ""
            title.set_text(
                f"rpy=({np.rad2deg(rpy[0]):+5.1f}, {np.rad2deg(rpy[1]):+5.1f}, {np.rad2deg(rpy[2]):+6.1f}) deg    "
                f"theta=({theta_deg[0]:+6.1f}, {theta_deg[1]:+6.1f}, {theta_deg[2]:+6.1f}) deg    "
                f"cmd=({theta_cmd_deg[0]:+6.1f}, {theta_cmd_deg[1]:+6.1f}, {theta_cmd_deg[2]:+6.1f}) deg"
                f"{active_tag}{limit_tag}\n"
                f"Streaming at {args.send_hz:.1f} Hz; speed={args.move_speed:.2f}. Press s/HOLD to disable, e to resume."
            )
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            dt = period - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            send_ascii(ser, "STOP")
        except Exception:
            pass
        try:
            dev.close()
        except Exception:
            pass
        kb.close()
        ser.close()
        plt.ioff()
        print("Stopped and closed serial.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())