#!/usr/bin/env python3
"""
EV ADAS Dashboard
=================
Receives the $EV / $AD / $ST / $ACK telemetry frames emitted by the STM32
firmware (see uart_shell.c) over a serial port, verifies each frame's
CRC-16/CCITT, and renders a live Tesla-style instrument cluster:
  - Speedometer arc (0-200 km/h)
  - Battery SOC bar + range + drive mode + torque/power readout
  - ADAS bird's-eye view: front/left/right proximity, TTC, collision +
    blind-spot indicators, alarm banner
  - Rolling 30 s speed history
  - Status bar: vehicle state, fault byte, link health, packet counters

Usage
-----
    python dashboard.py --port COM5             (Windows, real hardware)
    python dashboard.py --port /dev/ttyUSB0      (Linux/macOS, real hardware)
    python dashboard.py --demo                    (no hardware required)

Wire protocol
-------------
Every frame is:  $TAG,field:value,field:value,...*CCCC\r\n
CCCC is a 4-hex-digit CRC-16/CCITT (poly 0x1021, init 0xFFFF) computed over
the bytes between '$' and '*'. This module's crc16_ccitt() must match the
firmware's CRC16_CCITT() in uart_shell.c exactly -- they were written and
tested side by side.
"""
import argparse
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Wedge, Circle
from matplotlib.gridspec import GridSpec

try:
    import serial
except ImportError:
    serial = None  # only required for --port mode; --demo works without it


# ---------------------------------------------------------------------------
# CRC-16/CCITT -- must mirror uart_shell.c's CRC16_CCITT() bit-for-bit.
# ---------------------------------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


FRAME_RE = re.compile(r"^\$([A-Z]+),(.*)\*([0-9A-Fa-f]{4})$")


def parse_frame(line: str):
    """Parses one '$TAG,k:v,...*CRC' line. Returns (tag, {field: str}) on a
    verified frame, or None if the line is malformed or fails CRC."""
    line = line.strip()
    m = FRAME_RE.match(line)
    if not m:
        return None

    tag, payload, crc_hex = m.group(1), m.group(2), m.group(3)
    expected = crc16_ccitt(payload.encode("ascii", errors="replace"))
    if int(crc_hex, 16) != expected:
        return None

    fields = {}
    for token in payload.split(","):
        if ":" not in token:
            continue
        k, v = token.split(":", 1)
        fields[k] = v
    return tag, fields


def _f(fields, key, default=0.0):
    try:
        return float(fields.get(key, default))
    except ValueError:
        return default


def _i(fields, key, default=0):
    try:
        return int(fields.get(key, default))
    except ValueError:
        return default


# ---------------------------------------------------------------------------
# Shared telemetry state (written by the reader thread, read by the GUI
# thread's animation callback -- all access goes through `lock`).
# ---------------------------------------------------------------------------
@dataclass
class TelemetryState:
    lock: threading.Lock = field(default_factory=threading.Lock)

    speed_kmh: float = 0.0
    soc_pct: float = 100.0
    torque_nm: float = 0.0
    power_kw: float = 0.0
    range_km: float = 0.0
    motor_temp_c: float = 25.0
    mode: int = 1
    accel_pct: int = 0
    brake_pct: int = 0

    front_cm: float = 400.0
    left_cm: float = 400.0
    right_cm: float = 400.0
    ttc_sec: float = -1.0
    collision_lvl: int = 0
    blindspot_l: int = 0
    blindspot_r: int = 0
    alarm_lvl: int = 0
    fault_flags: int = 0

    vehicle_state: int = 0

    last_ev_time: float = 0.0
    last_ad_time: float = 0.0
    ok_count: int = 0
    bad_count: int = 0

    speed_history: deque = field(default_factory=lambda: deque(maxlen=300))
    time_history: deque = field(default_factory=lambda: deque(maxlen=300))
    t0: float = field(default_factory=time.time)

    def apply(self, tag, fields):
        now = time.time()
        with self.lock:
            if tag == "EV":
                self.speed_kmh = _f(fields, "SPD")
                self.soc_pct = _f(fields, "SOC")
                self.torque_nm = _f(fields, "TRQ")
                self.power_kw = _f(fields, "PWR")
                self.range_km = _f(fields, "RNG")
                self.motor_temp_c = _f(fields, "TMP")
                self.mode = _i(fields, "MODE", 1)
                self.accel_pct = _i(fields, "ACC")
                self.brake_pct = _i(fields, "BRK")
                self.last_ev_time = now
                self.speed_history.append(self.speed_kmh)
                self.time_history.append(now - self.t0)
            elif tag == "AD":
                self.front_cm = _f(fields, "F", 400)
                self.left_cm = _f(fields, "L", 400)
                self.right_cm = _f(fields, "R", 400)
                ttc_raw = fields.get("TTC", "--")
                self.ttc_sec = float(ttc_raw) if ttc_raw not in ("--", "") else -1.0
                self.collision_lvl = _i(fields, "COL")
                bsd = fields.get("BSD", "00")
                self.blindspot_l = 1 if len(bsd) > 0 and bsd[0] == "1" else 0
                self.blindspot_r = 1 if len(bsd) > 1 and bsd[1] == "1" else 0
                self.alarm_lvl = _i(fields, "ALM")
                self.fault_flags = int(fields.get("FLT", "0"), 16)
                self.last_ad_time = now
            elif tag == "ST":
                self.vehicle_state = _i(fields, "STATE")
                self.mode = _i(fields, "MODE", self.mode)
                self.fault_flags = int(fields.get("FLT", "0"), 16)
            self.ok_count += 1

    def mark_bad(self):
        with self.lock:
            self.bad_count += 1

    def snapshot(self):
        with self.lock:
            return dict(
                speed_kmh=self.speed_kmh, soc_pct=self.soc_pct, torque_nm=self.torque_nm,
                power_kw=self.power_kw, range_km=self.range_km, motor_temp_c=self.motor_temp_c,
                mode=self.mode, accel_pct=self.accel_pct, brake_pct=self.brake_pct,
                front_cm=self.front_cm, left_cm=self.left_cm, right_cm=self.right_cm,
                ttc_sec=self.ttc_sec, collision_lvl=self.collision_lvl,
                blindspot_l=self.blindspot_l, blindspot_r=self.blindspot_r,
                alarm_lvl=self.alarm_lvl, fault_flags=self.fault_flags,
                vehicle_state=self.vehicle_state,
                last_ev_time=self.last_ev_time, last_ad_time=self.last_ad_time,
                ok_count=self.ok_count, bad_count=self.bad_count,
                speed_history=list(self.speed_history), time_history=list(self.time_history),
            )


# ---------------------------------------------------------------------------
# Reader threads: real serial port, or a synthetic --demo generator. Both
# funnel lines through parse_frame()/state.apply() so the render code is
# exercised identically either way.
# ---------------------------------------------------------------------------
def serial_reader_thread(state: TelemetryState, port: str, baud: int, stop_evt: threading.Event):
    if serial is None:
        print("pyserial is not installed. Run: pip install pyserial", file=sys.stderr)
        stop_evt.set()
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"Could not open {port}: {exc}", file=sys.stderr)
        stop_evt.set()
        return

    with ser:
        while not stop_evt.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="replace")
            except UnicodeDecodeError:
                state.mark_bad()
                continue
            result = parse_frame(line)
            if result is None:
                if line.strip():
                    state.mark_bad()
                continue
            state.apply(*result)


def _demo_frame(tag: str, payload: str) -> str:
    crc = crc16_ccitt(payload.encode("ascii"))
    return f"${tag},{payload}*{crc:04X}"


def demo_generator_thread(state: TelemetryState, stop_evt: threading.Event):
    """Synthesizes a plausible drive cycle: accelerate, cruise, a front
    obstacle triggering a collision warning, then regen braking. Builds
    real CRC-checked frames and pushes them through the same parser as
    live serial data would use."""
    t = 0.0
    rng = np.random.default_rng(7)
    while not stop_evt.is_set():
        t += 0.1
        cycle = t % 40.0

        if cycle < 12:
            speed = min(120.0, cycle * 10.0)
            accel, brake, front = 60, 0, 400
        elif cycle < 20:
            speed = 120.0 - (cycle - 12) * 2
            front = max(15.0, 400 - (cycle - 12) * 55)
            accel, brake = 20, 0
        elif cycle < 26:
            speed = max(0.0, 120.0 - (cycle - 12) * 2 - (cycle - 20) * 18)
            front = max(8.0, 60 - (cycle - 20) * 9)
            accel, brake = 0, 40
        else:
            speed = 0.0
            accel, brake, front = 0, 0, 400

        speed = max(0.0, speed + rng.normal(0, 0.4))
        soc = max(0.0, 80.0 - t * 0.05)
        torque = (accel / 100.0) * 250.0 - (brake / 100.0) * 80.0
        power = torque * (speed / 3.6 / 0.30) * 8.0 / 1000.0
        mode = 1

        ev_payload = (f"SPD:{speed:.1f},SOC:{soc:.1f},TRQ:{torque:.1f},PWR:{power:.2f},"
                      f"RNG:{(soc/100*20000/30):.0f},TMP:{27.0 + rng.normal(0,0.3):.1f},"
                      f"MODE:{mode},ACC:{accel},BRK:{brake},UP:{int(t*1000)}")
        state.apply(*parse_frame(_demo_frame("EV", ev_payload)))

        left = 400 if cycle < 30 else 25
        col = 2 if front < 20 else (1 if front < 50 else 0)
        alm = 3 if col == 2 else (2 if col == 1 else (1 if left < 30 else 0))
        ttc_val = (front / max(speed / 3.6 * 100, 1)) if speed > 1 else -1
        ttc_field = f"{ttc_val:.1f}" if ttc_val > 0 else "--"
        ad_payload = (f"F:{front:.0f},L:{left:.0f},R:400,TTC:{ttc_field},COL:{col},"
                      f"BSD:{1 if left < 30 else 0}0,ALM:{alm},FLT:00")
        state.apply(*parse_frame(_demo_frame("AD", ad_payload)))

        time.sleep(0.1)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
BG = "#0b0f14"
PANEL = "#121821"
GRID = "#232c38"
TEXT = "#e7edf3"
MUTED = "#7c8a9a"
ACCENT = "#3ddc97"
WARN = "#f5a524"
CRIT = "#ef4444"
BLUE = "#4ea8ff"

MODE_NAMES = {0: "ECO", 1: "NORMAL", 2: "SPORT"}
STATE_NAMES = {0: "PARKED", 1: "READY", 2: "DRIVING", 3: "REGEN", 4: "FAULT"}
ALARM_COLORS = {0: ACCENT, 1: BLUE, 2: WARN, 3: CRIT}
ALARM_NAMES = {0: "CLEAR", 1: "ADVISORY", 2: "WARNING", 3: "CRITICAL"}


class Dashboard:
    def __init__(self, state: TelemetryState):
        self.state = state
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(13, 8), facecolor=BG)
        self.fig.canvas.manager.set_window_title("EV ADAS Dashboard")
        gs = GridSpec(2, 3, figure=self.fig, height_ratios=[1.15, 1],
                      wspace=0.28, hspace=0.32, left=0.04, right=0.97, top=0.90, bottom=0.07)

        self.ax_speed = self.fig.add_subplot(gs[0, 0])
        self.ax_batt = self.fig.add_subplot(gs[0, 1])
        self.ax_adas = self.fig.add_subplot(gs[0, 2])
        self.ax_hist = self.fig.add_subplot(gs[1, 0:2])
        self.ax_status = self.fig.add_subplot(gs[1, 2])

        for ax in (self.ax_speed, self.ax_batt, self.ax_adas, self.ax_status):
            ax.set_facecolor(PANEL)
            ax.set_xticks([]); ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
        self.ax_hist.set_facecolor(PANEL)
        for s in self.ax_hist.spines.values():
            s.set_color(GRID)

        self.fig.suptitle("EV ADAS  \u2014  Live Telemetry", color=TEXT,
                           fontsize=16, fontweight="bold", x=0.04, ha="left")
        self.banner = self.fig.text(0.5, 0.955, "", color=TEXT, fontsize=12,
                                     fontweight="bold", ha="center", va="center")

    # ---- individual panels -------------------------------------------------
    def draw_speed(self, snap):
        ax = self.ax_speed
        ax.clear(); ax.set_facecolor(PANEL); ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values(): s.set_visible(False)
        ax.set_xlim(-1.3, 1.3); ax.set_ylim(-1.1, 1.3); ax.set_aspect("equal")

        speed = max(0.0, min(200.0, snap["speed_kmh"]))
        frac = speed / 200.0
        start_deg, end_deg = 210, -30  # sweep clockwise, 240 degrees total

        ax.add_patch(Wedge((0, 0), 1.0, end_deg, start_deg, width=0.14, facecolor=GRID))
        needle_color = CRIT if speed > 160 else (WARN if speed > 120 else ACCENT)
        sweep_deg = start_deg - frac * 240.0
        ax.add_patch(Wedge((0, 0), 1.0, sweep_deg, start_deg, width=0.14, facecolor=needle_color))

        for v in range(0, 201, 40):
            a = np.radians(start_deg - (v / 200.0) * 240.0)
            x1, y1 = 0.86 * np.cos(a), 0.86 * np.sin(a)
            x2, y2 = 0.99 * np.cos(a), 0.99 * np.sin(a)
            ax.plot([x1, x2], [y1, y2], color=MUTED, linewidth=1.2)
            ax.text(0.72 * np.cos(a), 0.72 * np.sin(a), str(v), color=MUTED,
                    fontsize=8, ha="center", va="center")

        ax.text(0, 0.05, f"{speed:.0f}", color=TEXT, fontsize=34, fontweight="bold",
                ha="center", va="center")
        ax.text(0, -0.22, "km/h", color=MUTED, fontsize=11, ha="center", va="center")

        mode_txt = MODE_NAMES.get(snap["mode"], "?")
        ax.text(0, -0.55, mode_txt, color=BLUE, fontsize=12, fontweight="bold",
                ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.35", facecolor=BG, edgecolor=BLUE, linewidth=1))

        ax.text(-1.15, -1.0, f"ACCEL {snap['accel_pct']:>3d}%", color=ACCENT, fontsize=9, ha="left")
        ax.text(1.15, -1.0, f"BRAKE {snap['brake_pct']:>3d}%", color=CRIT, fontsize=9, ha="right")

    def draw_battery(self, snap):
        ax = self.ax_batt
        ax.clear(); ax.set_facecolor(PANEL); ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values(): s.set_visible(False)
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)

        ax.text(0.06, 0.93, "BATTERY", color=MUTED, fontsize=10, fontweight="bold", va="top")
        soc = max(0.0, min(100.0, snap["soc_pct"]))
        color = CRIT if soc < 15 else (WARN if soc < 30 else ACCENT)

        ax.add_patch(FancyBboxPatch((0.06, 0.66), 0.82, 0.14, boxstyle="round,pad=0.006",
                                     facecolor=GRID, edgecolor="none"))
        ax.add_patch(FancyBboxPatch((0.06, 0.66), 0.82 * (soc / 100.0), 0.14,
                                     boxstyle="round,pad=0.006", facecolor=color, edgecolor="none"))
        ax.add_patch(FancyBboxPatch((0.89, 0.695), 0.03, 0.07, boxstyle="round,pad=0.003",
                                     facecolor=GRID, edgecolor="none"))
        ax.text(0.06, 0.85, f"{soc:.1f}%", color=TEXT, fontsize=20, fontweight="bold", va="bottom")

        rows = [
            ("Range", f"{snap['range_km']:.0f} km"),
            ("Torque", f"{snap['torque_nm']:+.0f} Nm"),
            ("Power", f"{snap['power_kw']:+.2f} kW"),
            ("Motor temp", f"{snap['motor_temp_c']:.1f} \u00b0C"),
        ]
        y = 0.52
        for label, val in rows:
            ax.text(0.06, y, label, color=MUTED, fontsize=10, va="center")
            ax.text(0.94, y, val, color=TEXT, fontsize=11, fontweight="bold", va="center", ha="right")
            y -= 0.13

    def draw_adas(self, snap):
        ax = self.ax_adas
        ax.clear(); ax.set_facecolor(PANEL); ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values(): s.set_visible(False)
        ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.25, 1.25); ax.set_aspect("equal")

        ax.text(-1.12, 1.14, "ADAS", color=MUTED, fontsize=10, fontweight="bold", va="top")

        col = snap["collision_lvl"]
        car_color = CRIT if col == 2 else (WARN if col == 1 else GRID)
        ax.add_patch(FancyBboxPatch((-0.22, -0.32), 0.44, 0.64, boxstyle="round,pad=0.05",
                                     facecolor=car_color, edgecolor=TEXT, linewidth=1))

        def prox_color(cm):
            if cm < 20: return CRIT
            if cm < 50: return WARN
            return ACCENT

        front_norm = 1.0 - min(snap["front_cm"], 120.0) / 120.0   # saturates beyond 120 cm
        front_h = 0.05 + front_norm * 0.80                          # stays within the panel's ylim
        ax.add_patch(FancyBboxPatch((-0.12, 0.36), 0.24, front_h, boxstyle="round,pad=0.02",
                                     facecolor=prox_color(snap["front_cm"]), edgecolor="none"))
        ax.text(0, 1.08, f"F {snap['front_cm']:.0f}cm", color=TEXT, fontsize=9, ha="center")

        bs_l = "\u25CF" if snap["blindspot_l"] else "\u25CB"
        bs_r = "\u25CF" if snap["blindspot_r"] else "\u25CB"
        ax.text(-0.85, 0, bs_l, color=(CRIT if snap["blindspot_l"] else MUTED), fontsize=22, ha="center", va="center")
        ax.text(0.85, 0, bs_r, color=(CRIT if snap["blindspot_r"] else MUTED), fontsize=22, ha="center", va="center")
        ax.text(-0.85, -0.35, f"L {snap['left_cm']:.0f}cm", color=MUTED, fontsize=8, ha="center")
        ax.text(0.85, -0.35, f"R {snap['right_cm']:.0f}cm", color=MUTED, fontsize=8, ha="center")

        ttc_txt = f"TTC {snap['ttc_sec']:.1f}s" if snap["ttc_sec"] > 0 else "TTC \u2014"
        ax.text(0, -0.85, ttc_txt, color=TEXT, fontsize=10, ha="center")

        alarm = snap["alarm_lvl"]
        ax.add_patch(FancyBboxPatch((-1.1, -1.22), 2.2, 0.22, boxstyle="round,pad=0.02",
                                     facecolor=ALARM_COLORS.get(alarm, MUTED), edgecolor="none"))
        ax.text(0, -1.11, ALARM_NAMES.get(alarm, "?"), color=BG, fontsize=10,
                fontweight="bold", ha="center", va="center")

    def draw_history(self, snap):
        ax = self.ax_hist
        ax.clear()
        ax.set_facecolor(PANEL)
        for s in ax.spines.values(): s.set_color(GRID)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(True, color=GRID, linewidth=0.6)

        t_hist, s_hist = snap["time_history"], snap["speed_history"]
        ax.set_title("Speed history", color=MUTED, fontsize=10, loc="left")
        if len(t_hist) >= 2:
            t0 = t_hist[-1]
            xs = [t - t0 for t in t_hist]
            ax.plot(xs, s_hist, color=ACCENT, linewidth=1.8)
            ax.fill_between(xs, s_hist, 0, color=ACCENT, alpha=0.12)
            ax.set_xlim(min(min(xs), -1.0), 0)
        else:
            ax.set_xlim(-30, 0)
        ax.set_ylim(0, 210)
        ax.set_xlabel("seconds ago", color=MUTED, fontsize=8)
        ax.set_ylabel("km/h", color=MUTED, fontsize=8)

    def draw_status(self, snap):
        ax = self.ax_status
        ax.clear(); ax.set_facecolor(PANEL); ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values(): s.set_visible(False)
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.text(0.06, 0.93, "STATUS", color=MUTED, fontsize=10, fontweight="bold", va="top")

        now = time.time()
        link_age = now - max(snap["last_ev_time"], snap["last_ad_time"])
        link_ok = snap["last_ev_time"] > 0 and link_age < 5.0
        link_txt = "LINK OK" if link_ok else ("NO SIGNAL" if snap["last_ev_time"] == 0 else "SIGNAL LOST")
        link_color = ACCENT if link_ok else CRIT

        ax.add_patch(Circle((0.09, 0.83), 0.025, facecolor=link_color))
        ax.text(0.15, 0.83, link_txt, color=link_color, fontsize=10, fontweight="bold", va="center")

        vstate = STATE_NAMES.get(snap["vehicle_state"], "?")
        state_color = CRIT if snap["vehicle_state"] == 4 else TEXT
        rows = [
            ("Vehicle state", vstate, state_color),
            ("Fault byte", f"0x{snap['fault_flags']:02X}", (CRIT if snap["fault_flags"] else TEXT)),
            ("Packets OK", str(snap["ok_count"]), TEXT),
            ("Packets bad", str(snap["bad_count"]), (WARN if snap["bad_count"] else MUTED)),
        ]
        y = 0.68
        for label, val, color in rows:
            ax.text(0.06, y, label, color=MUTED, fontsize=9.5, va="center")
            ax.text(0.94, y, val, color=color, fontsize=10.5, fontweight="bold", va="center", ha="right")
            y -= 0.14

        if not link_ok and snap["last_ev_time"] > 0:
            ax.text(0.06, 0.06, "Showing last known values", color=MUTED, fontsize=8.5,
                     style="italic", va="bottom")

    def update(self, _frame):
        snap = self.state.snapshot()
        self.draw_speed(snap)
        self.draw_battery(snap)
        self.draw_adas(snap)
        self.draw_history(snap)
        self.draw_status(snap)

        alarm = snap["alarm_lvl"]
        if alarm >= 2:
            self.banner.set_text("\u26A0  " + ("COLLISION CRITICAL" if snap["collision_lvl"] == 2 else "COLLISION WARNING"))
            self.banner.set_color(CRIT if alarm == 3 else WARN)
        elif snap["fault_flags"]:
            self.banner.set_text("\u26A0  FAULT ACTIVE \u2014 send 'fault clear' to recover")
            self.banner.set_color(CRIT)
        else:
            self.banner.set_text("")
        return []


def main():
    ap = argparse.ArgumentParser(description="EV ADAS live telemetry dashboard")
    ap.add_argument("--port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="Run with synthetic data, no hardware required")
    args = ap.parse_args()

    if not args.demo and not args.port:
        ap.error("pass --port <name> for real hardware, or --demo to run without it")

    state = TelemetryState()
    stop_evt = threading.Event()

    if args.demo:
        reader = threading.Thread(target=demo_generator_thread, args=(state, stop_evt), daemon=True)
    else:
        reader = threading.Thread(target=serial_reader_thread,
                                   args=(state, args.port, args.baud, stop_evt), daemon=True)
    reader.start()

    dash = Dashboard(state)
    # NOTE: `anim` must stay referenced for the life of the program -- if this
    # binding is dropped, Python garbage-collects the FuncAnimation and the
    # dashboard silently freezes on its first frame. This is one of
    # matplotlib's best-known gotchas.
    anim = matplotlib.animation.FuncAnimation(dash.fig, dash.update, interval=150, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()
