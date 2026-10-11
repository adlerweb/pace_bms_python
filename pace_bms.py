#!/usr/bin/env python3
"""Poll a PACE-based BMS (PowMr POW-LIO48100-16S) over its RS232 port.

Implements the read-only parts of "PACE RS232 Communication Protocol
(PACE-RS232-TY16S) 2018-07-05": CID2 0x42 (analog data) and 0x44 (warnings).
The protection limits (CID2 0xA0..0xED "read configuration") are not in that
PDF; their layouts are taken from github.com/nkinnan/esphome-pace-bms, which
documents them from PBmsTools captures. Nothing here writes to the BMS.

Wiring (RJ12/6P6C, pin 1 on the left, tab away from you):
  pin 3 = BMS TX -> adapter RxD, pin 4 = BMS RX <- adapter TxD, pin 5 = GND.
"""
import argparse
import json
import sys
import time
from collections import deque

import serial
from rich import box
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

CID1_LIFEPO4 = 0x46
CID2_ANALOG = 0x42
CID2_WARN = 0x44

RTN = {0x00: "ok", 0x01: "version error", 0x02: "checksum error",
       0x03: "length checksum error", 0x04: "CID2 undefined",
       0x05: "command format error", 0x06: "invalid data"}

PROTECT1 = ["above cell V", "below cell V", "above total V", "below total V",
            "charge current", "discharge current", "short circuit"]
PROTECT2 = ["above charge temp", "above discharge temp", "below charge temp",
            "below discharge temp", "above MOS temp", "above env temp",
            "below env temp", "fully charged"]
INSTRUCT = ["current limit", "charge FET on", "discharge FET on", "pack indicate",
            "reverse", "AC in", None, "heartbeat"]
FAULT = ["charge MOS fault", "discharge MOS fault", "NTC fault", None,
         "cell fault", "sample fault"]
WARN1 = ["above cell V", "below cell V", "above total V", "below total V",
         "charge current", "discharge current"]
WARN2 = ["above charge temp", "above discharge temp", "below charge temp",
         "below discharge temp", "high env temp", "low env temp",
         "high MOS temp", "low power"]
LEVEL = {0x00: "ok", 0x01: "below lower limit", 0x02: "above upper limit",
         0xF0: "other fault"}


def checksum(body: bytes) -> int:
    return (-sum(body)) & 0xFFFF


def lenid(n: int) -> int:
    """LENGTH field for an INFO of n ASCII bytes: 12-bit length + 4-bit check nibble."""
    s = ((n >> 8) & 0xF) + ((n >> 4) & 0xF) + (n & 0xF)
    return ((((~s) + 1) & 0xF) << 12) | n


def build(adr: int, cid2: int, info: bytes) -> bytes:
    body = b"25%02X%02X%02X%04X" % (adr, CID1_LIFEPO4, cid2, lenid(len(info))) + info
    return b"~" + body + b"%04X\r" % checksum(body)


def parse_frame(raw: bytes):
    """Validate a response frame; return (adr, rtn, info_ascii)."""
    if not (raw.startswith(b"~") and raw.endswith(b"\r")):
        raise ValueError("no SOI/EOI framing: %r" % raw[:40])
    body, chk = raw[1:-5], raw[-5:-1]
    if int(chk, 16) != checksum(body):
        raise ValueError("checksum mismatch")
    adr, rtn, length = int(body[2:4], 16), int(body[6:8], 16), int(body[8:12], 16)
    info = body[12:]
    if (length & 0xFFF) != len(info):
        raise ValueError("length mismatch")
    return adr, rtn, info.decode("ascii")


class Reader:
    """Sequential hex-ASCII reader over an INFO string."""

    def __init__(self, s):
        self.s, self.p = s, 0

    def u(self, nbytes=1):
        v = int(self.s[self.p:self.p + 2 * nbytes], 16)
        self.p += 2 * nbytes
        return v

    def i16(self):
        v = self.u(2)
        return v - 0x10000 if v & 0x8000 else v

    def rest(self):
        return self.s[self.p:]


def decode_analog(info: str) -> dict:
    r = Reader(info)
    r.u()  # INFOFLAG
    r.u()  # command value / pack number
    cells = [r.u(2) for _ in range(r.u())]
    temps = [(r.u(2) - 2731) / 10 for _ in range(r.u())]
    d = {
        "cells_mV": cells,
        "temps_C": temps,
        "current_A": r.i16() / 100,
        "voltage_V": r.u(2) / 1000,
        "remain_Ah": r.u(2) / 100,
    }
    r.u()  # "define number" (= 3)
    d["full_Ah"] = r.u(2) / 100
    d["cycles"] = r.u(2)
    d["design_Ah"] = r.u(2) / 100
    d["soc_calc_pct"] = round(100 * d["remain_Ah"] / d["full_Ah"], 1) if d["full_Ah"] else None
    d["cell_min_mV"], d["cell_max_mV"] = min(cells), max(cells)
    d["cell_delta_mV"] = d["cell_max_mV"] - d["cell_min_mV"]
    d["power_W"] = round(d["current_A"] * d["voltage_V"], 1)
    # Bytes beyond the spec. First byte matches SOC on the LIO48100; rest unknown.
    d["extra_hex"] = r.rest()
    if d["extra_hex"]:
        d["soc_ext_pct"] = int(d["extra_hex"][:2], 16)
    return d


def bits(value, names):
    return [n for i, n in enumerate(names) if n and value & (1 << i)]


def decode_warn(info: str) -> dict:
    r = Reader(info)
    r.u()
    r.u()
    cell_w = [r.u() for _ in range(r.u())]
    temp_w = [r.u() for _ in range(r.u())]
    chg_w, volt_w, dis_w = r.u(), r.u(), r.u()
    p1, p2, ins, ctl, flt = r.u(), r.u(), r.u(), r.u(), r.u()
    bal = r.u() | (r.u() << 8)
    w1, w2 = r.u(), r.u()
    lvl = lambda v: LEVEL.get(v, "user 0x%02X" % v)
    return {
        "cell_warn": {i + 1: lvl(v) for i, v in enumerate(cell_w) if v},
        "temp_warn": {i + 1: lvl(v) for i, v in enumerate(temp_w) if v},
        "pack_warn": {k: lvl(v) for k, v in (("charge current", chg_w),
                                              ("total voltage", volt_w),
                                              ("discharge current", dis_w)) if v},
        "protect": bits(p1, PROTECT1) + bits(p2 & 0x7F, PROTECT2),
        "fully_charged": bool(p2 & 0x80),
        "state": bits(ins, INSTRUCT),
        "faults": bits(flt, FAULT),
        "balancing": [i + 1 for i in range(16) if bal & (1 << i)],
        "warnings": bits(w1, WARN1) + bits(w2, WARN2),
        "control_raw": ctl,
    }


# --- protection limits / settings (read-only) --------------------------------
# Each entry: (CID2, name, [(key, nbytes, transform)]). key None = skipped byte(s).
# Trailing bytes beyond the listed fields are ignored.
def _v(x): return x / 1000
def _t(x): return (x - 2730) / 10
def _i16(x): return x - 0x10000 if x & 0x8000 else x
def _abs16(x): return abs(_i16(x))
def _ms100(x): return x * 100
def _ms25(x): return x * 25
def _same(x): return x

def _voltage_set():
    return [(None, 1, None), ("alarm_V", 2, _v), ("protect_V", 2, _v),
            ("release_V", 2, _v), ("delay_ms", 1, _ms100)]

def _temp_set(*names):
    return [(None, 1, None)] + [(n, 2, _t) for n in names]

SETTINGS = [
    (0xD1, "cell_ov", _voltage_set()),
    (0xD3, "cell_uv", _voltage_set()),
    (0xD5, "pack_ov", _voltage_set()),
    (0xD7, "pack_uv", _voltage_set()),
    (0xD9, "charge_oc", [(None, 1, None), ("alarm_A", 2, _same), ("protect_A", 2, _same),
                         ("delay_ms", 1, _ms100)]),
    (0xDB, "discharge_oc1", [(None, 1, None), ("alarm_A", 2, _abs16), ("protect_A", 2, _abs16),
                             ("delay_ms", 1, _ms100)]),
    (0xE3, "discharge_oc2", [(None, 1, None), ("protect_A", 1, _same), ("delay_ms", 1, _ms25)]),
    (0xE5, "short_circuit", [("delay_us", 1, _ms25)]),
    (0xB6, "balance", [("start_V", 2, _v), ("delta_mV", 2, _same)]),
    (0xA0, "sleep", [("cell_V", 2, _v), (None, 1, None), ("delay_min", 1, _same)]),
    (0xAF, "full_charge", [("voltage_V", 2, _v), ("current_mA", 2, _same),
                           ("low_soc_alarm_pct", 1, _same)]),
    (0xDD, "temp_over", _temp_set("chg_alarm", "chg_protect", "chg_release",
                                  "dis_alarm", "dis_protect", "dis_release")),
    (0xDF, "temp_under", _temp_set("chg_alarm", "chg_protect", "chg_release",
                                   "dis_alarm", "dis_protect", "dis_release")),
    (0xE1, "mos_temp", _temp_set("alarm", "protect", "release")),
    (0xE7, "env_temp", _temp_set("under_alarm", "under_protect", "under_release",
                                 "over_alarm", "over_protect", "over_release")),
    (0xED, "charge_limiter_start_A", [("start_A", 2, _same)]),
]


def decode_setting(fields, info: str) -> dict:
    r = Reader(info)
    out = {}
    for key, n, fn in fields:
        v = r.u(n)
        if key:
            out[key] = fn(v)
    return out


class Bms:
    def __init__(self, port, baud=9600, adr=1, pack=1, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.adr, self.pack = adr, pack

    def query(self, cid2, info=None):
        if info is None:
            info = b"%02X" % self.pack
        self.ser.reset_input_buffer()
        self.ser.write(build(self.adr, cid2, info))
        raw = self.ser.read_until(b"\r")
        if not raw:
            raise TimeoutError("no answer (check wiring/TX-RX swap/address)")
        _, rtn, info = parse_frame(raw)
        if rtn:
            raise RuntimeError("BMS error: %s" % RTN.get(rtn, hex(rtn)))
        return info

    def read_settings(self):
        """Read every limit; a command the firmware rejects becomes {"error": ...}."""
        out = {}
        for cid2, name, fields in SETTINGS:
            try:
                out[name] = decode_setting(fields, self.query(cid2, b""))
            except (TimeoutError, ValueError, RuntimeError) as e:
                out[name] = {"error": str(e)}
        return out

    def poll(self):
        d = decode_analog(self.query(CID2_ANALOG))
        d["status"] = decode_warn(self.query(CID2_WARN))
        return d


# --- display -----------------------------------------------------------------
def num(v, unit, nd=0, sign=False, style=None):
    """One right-aligned number with its unit, or a dim dash when absent."""
    if v is None:
        return Text("–", style="dim", justify="right")
    return Text("%s %s" % (format(v, "%s.%df" % ("+" if sign else "", nd)), unit),
                style=style or "", justify="right")


def flags(items, ok_text="none", bad_style="bold red"):
    return Text(", ".join(items), style=bad_style) if items else Text(ok_text, style="green")


def grid(*cols):
    t = Table.grid(padding=(0, 2))
    for c in cols:
        t.add_column(**c)
    return t


BAR, SPREAD = 24, 16  # gauge widths in characters
PARTIAL = " ▏▎▍▌▋▊▉█"


def zone_style(x, uv_p, uv_a, ov_a, ov_p):
    """Green inside the alarm band, yellow between alarm and protect, red beyond protect."""
    return "red" if x <= uv_p or x >= ov_p else "yellow" if x <= uv_a or x >= ov_a else "green"


def gauge(value, lo, hi, width, style_at):
    """Horizontal bar from lo to hi. The fill takes the colour of the value's zone,
    the empty track shows the zones ahead (dimmed) so the danger areas stay visible."""
    fill = style_at(value)
    t = Text()
    for i in range(width):
        x0, x1 = lo + (hi - lo) * i / width, lo + (hi - lo) * (i + 1) / width
        f = min(1.0, max(0.0, (value - x0) / (x1 - x0)))
        if f >= 1:
            t.append("█", style=fill)
        elif f > 0 or (i == 0 and value <= lo):  # below the scale: keep a sliver so it isn't blank
            f = f or 1 / 8
            t.append(PARTIAL[round(f * 8)] or "▏", style=fill)
        else:
            t.append("·", style="dim " + style_at((x0 + x1) / 2))
    return t


def centered_gauge(value, neg_max, pos_max, width, style_at):
    """Bar growing left (negative) or right (positive) from a centre tick; style_at gets |value|."""
    half = width // 2
    fill = style_at(abs(value))
    t = Text()

    def seg(mag, on):
        t.append("█" if on else "·", style=fill if on else "dim " + style_at(mag))

    for j in reversed(range(half)):  # left half, outermost segment first
        mag = neg_max * (j + 0.5) / half
        seg(mag, value < 0 and -value >= mag)
    t.append("│", style="bold")
    for j in range(half):
        mag = pos_max * (j + 0.5) / half
        seg(mag, value > 0 and value >= mag)
    return t


def axis(left, right, width):
    return "%s%s%s" % (left, " " * max(1, width - len(left) - len(right)), right)


def limit_set(cfg, key, a, p):
    c = (cfg or {}).get(key, {})
    return (c[a], c[p]) if a in c and p in c else None


def volt_scale(cfg, uv_key, ov_key, n, default):
    """(lo, hi, style_at, known) for a voltage gauge; falls back to a plain scale without limits."""
    uv, ov = limit_set(cfg, uv_key, "alarm_V", "protect_V"), limit_set(cfg, ov_key, "alarm_V", "protect_V")
    if not (uv and ov):
        lo, hi = default
        return lo, hi, (lambda x: "green"), False
    pad = 0.04 * (ov[1] - uv[1])
    return uv[1] - pad, ov[1] + pad, (lambda x: zone_style(x, uv[1], uv[0], ov[0], ov[1])), True


def pack_panel(d, cfg):
    soc = d["soc_calc_pct"] or 0
    cur = d["current_A"]
    cur_style = "green" if cur > 0.05 else "yellow" if cur < -0.05 else "dim"
    n = len(d["cells_mV"])
    lo, hi, style_at, known = volt_scale(cfg, "pack_uv", "pack_ov", n, (2.5 * n, 3.7 * n))
    chg, dis = limit_set(cfg, "charge_oc", "alarm_A", "protect_A"), limit_set(cfg, "discharge_oc1", "alarm_A", "protect_A")
    low_soc = ((cfg or {}).get("full_charge") or {}).get("low_soc_alarm_pct", 0)

    t = grid({"style": "dim"}, {"justify": "right"}, {})
    t.add_row("Voltage", num(d["voltage_V"], "V", 3, style="bold"), gauge(d["voltage_V"], lo, hi, BAR, style_at))
    if chg and dis:
        amp = lambda x: "green" if x < min(chg[0], dis[0]) else "yellow" if x < min(chg[1], dis[1]) else "red"
        t.add_row("Current", num(cur, "A", 2, True, cur_style),
                  centered_gauge(cur, dis[1], chg[1], BAR, amp))
    else:
        t.add_row("Current", num(cur, "A", 2, True, cur_style))
    t.add_row("Power", num(d["power_W"], "W", 1, True, cur_style))
    t.add_row("Charge", num(soc, "%", 1, style="bold"),
              gauge(soc, 0, 100, BAR, lambda x: "red" if x < low_soc else "green"))
    t.add_row("Remaining", num(d["remain_Ah"], "Ah", 2))
    t.add_row("Full capacity", num(d["full_Ah"], "Ah", 2))
    t.add_row("Design capacity", num(d["design_Ah"], "Ah", 2))
    t.add_row("Cycles", Text(str(d["cycles"]), justify="right"))
    sub = "yellow = alarm level, red = protect level" if known and chg and dis else None
    return Panel(t, title="Pack", title_align="left", border_style="blue", subtitle=sub, subtitle_align="right")


HISTORY_S = 3600
CHART_ROWS = 6
GUTTER = 8
BRAILLE = ((0x01, 0x02, 0x04, 0x40), (0x08, 0x10, 0x20, 0x80))  # [dot column][dot row]


def power_panel(history, now, width):
    """Line chart of pack power over the last HISTORY_S seconds, drawn with braille dots (2x4 per cell)."""
    w = max(10, width - GUTTER)
    nx, ny = 2 * w, 4 * CHART_ROWS
    acc = [[] for _ in range(nx)]
    for ts, p in history:
        age = now - ts
        if 0 <= age < HISTORY_S:
            acc[min(nx - 1, int((1 - age / HISTORY_S) * nx))].append(p)
    vals = [sum(a) / len(a) if a else None for a in acc]
    known = [v for v in vals if v is not None]
    if not known:
        body = Text("collecting data…", style="dim")
        return Panel(body, title="Power, last %d min" % (HISTORY_S // 60), title_align="left", border_style="blue")
    lo, hi = min(0, min(known)), max(0, max(known))
    if hi - lo < 50:
        hi, lo = hi + (25 if hi else 0), lo - (25 if lo else 0)
        if hi == lo:
            hi, lo = 25, -25
    row_of = lambda v: round((hi - v) / (hi - lo) * (ny - 1))
    bits = [[0] * w for _ in range(CHART_ROWS)]
    sign = [[0] * w for _ in range(CHART_ROWS)]

    def dot(x, y, v):
        bits[y // 4][x // 2] |= BRAILLE[x % 2][y % 4]
        sign[y // 4][x // 2] = 1 if v > 0.5 else -1 if v < -0.5 else sign[y // 4][x // 2]

    prev = None
    for x, v in enumerate(vals):
        if v is None:
            prev = None
            continue
        y = row_of(v)
        for yy in range(min(y, prev), max(y, prev) + 1) if prev is not None else (y,):
            dot(x, yy, v)
        prev = y
    zero = row_of(0)
    zero_bits = [[0] * w for _ in range(CHART_ROWS)]
    for x in range(0, nx, 3):
        zero_bits[zero // 4][x // 2] |= BRAILLE[x % 2][zero % 4]

    fmt = lambda v: "0 W" if abs(v) < 0.5 else "%+.0f W" % v
    labels = {zero // 4: "0 W", 0: fmt(hi), CHART_ROWS - 1: fmt(lo)}
    t = Text()
    for r in range(CHART_ROWS):
        t.append(labels.get(r, "").rjust(GUTTER - 1) + " ", style="dim")
        for c in range(w):
            b = bits[r][c]
            if b:
                t.append(chr(0x2800 + (b | zero_bits[r][c])), style="green" if sign[r][c] > 0 else "yellow" if sign[r][c] < 0 else "")
            else:
                t.append(chr(0x2800 + zero_bits[r][c]) if zero_bits[r][c] else " ", style="dim")
        t.append("\n" if r < CHART_ROWS - 1 else "")
    t.append("\n" + " " * GUTTER + axis("-%d min" % (HISTORY_S // 60), "now", w), style="dim")
    sub = "green = charging, yellow = discharging"
    return Panel(t, title="Power, last %d min" % (HISTORY_S // 60), title_align="left", border_style="blue",
                 subtitle=sub, subtitle_align="right")


def cells_panel(d, cfg):
    cells, lo_c, hi_c = d["cells_mV"], d["cell_min_mV"], d["cell_max_mV"]
    mean = sum(cells) / len(cells)
    bal = set(d["status"]["balancing"])
    lo, hi, style_at, known = volt_scale(cfg, "cell_uv", "cell_ov", 1, (2.5, 3.7))
    t = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False, collapse_padding=True)
    t.add_column("Cell", justify="right", style="dim")
    t.add_column("Voltage", justify="right")
    t.add_column(axis("%.2f V" % lo, "%.2f V" % hi, BAR), no_wrap=True)
    span = max(10.0, 1.25 * max(abs(c - mean) for c in cells))  # mV; zoom on the imbalance
    t.add_column("vs. mean", justify="right", no_wrap=True)
    t.add_column(axis("−%d mV" % span, "+%d mV" % span, SPREAD), no_wrap=True)
    t.add_column("", width=1, style="yellow")
    for k, c in enumerate(cells):
        style = "cyan" if c == lo_c != hi_c else "magenta" if c == hi_c != lo_c else ""
        t.add_row(str(k + 1), num(c / 1000, "V", 3, style=style), gauge(c / 1000, lo, hi, BAR, style_at),
                  num(c - mean, "mV", 1, True, "dim"),
                  centered_gauge(c - mean, span, span, SPREAD, lambda x: "cyan"), "●" if k + 1 in bal else "")
    title = "Cells   min %.3f V · max %.3f V · Δ %d mV · Σ %.3f V" % (
        lo_c / 1000, hi_c / 1000, d["cell_delta_mV"], sum(cells) / 1000)
    sub = ("bars: %.2f–%.2f V, yellow = alarm, red = protect" % (lo, hi) if known else "limits unknown") + (
        "   ● = balancing" if bal else "")
    return Panel(t, title=title, title_align="left", border_style="blue", subtitle=sub, subtitle_align="right")


def temps_panel(d):
    t = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False)
    for i in range(len(d["temps_C"])):
        t.add_column("T%d" % (i + 1), justify="right")
    t.add_row(*[num(v, "°C", 1) for v in d["temps_C"]])
    return Panel(t, title="Temperatures", title_align="left", border_style="blue")


def status_panel(d):
    s = d["status"]
    fet = lambda name: Text("ON", style="green") if name in s["state"] else Text("OFF", style="bold red")
    t = grid({"style": "dim"}, {})
    t.add_row("Charge FET", fet("charge FET on"))
    t.add_row("Discharge FET", fet("discharge FET on"))
    t.add_row("Other", Text(", ".join(n for n in s["state"] if "FET" not in n) or "-", style="dim"))
    t.add_row("Balancing", Text(", ".join("cell %d" % c for c in s["balancing"]) or "none",
                                style="yellow" if s["balancing"] else "dim"))
    t.add_row("Protection", flags(s["protect"]))
    t.add_row("Warnings", flags(s["warnings"], bad_style="yellow"))
    t.add_row("Faults", flags(s["faults"]))
    return Panel(t, title="Status", title_align="left", border_style="blue")


def limits_panel(cfg):
    t = Table(box=box.SIMPLE_HEAD, show_edge=False, pad_edge=False)
    t.add_column("")
    for h in ("Alarm", "Protect", "Release", "Delay"):
        t.add_column(h, justify="right")

    def add(name, key, keys, unit, nd, delay=None, dnd=0):
        c = cfg.get(key, {})
        if "error" in c:
            t.add_row(name, *[Text("n/a", style="dim red", justify="right")] * 4)
            return
        vals = [num(c.get(k), unit, nd) for k in keys]
        dv = None if delay is None else num(c[delay[0]], delay[1], dnd)
        t.add_row(name, *vals, dv if dv is not None else num(None, ""))

    volt = ("alarm_V", "protect_V", "release_V")
    add("Cell over voltage", "cell_ov", volt, "V", 2, ("delay_ms", "ms"))
    add("Cell under voltage", "cell_uv", volt, "V", 2, ("delay_ms", "ms"))
    add("Pack over voltage", "pack_ov", volt, "V", 2, ("delay_ms", "ms"))
    add("Pack under voltage", "pack_uv", volt, "V", 2, ("delay_ms", "ms"))
    t.add_section()
    add("Charge overcurrent", "charge_oc", ("alarm_A", "protect_A", "-"), "A", 0, ("delay_ms", "ms"))
    add("Discharge overcurrent 1", "discharge_oc1", ("alarm_A", "protect_A", "-"), "A", 0, ("delay_ms", "ms"))
    add("Discharge overcurrent 2", "discharge_oc2", ("-", "protect_A", "-"), "A", 0, ("delay_ms", "ms"))
    add("Short circuit", "short_circuit", ("-", "-", "-"), "", 0, ("delay_us", "µs"))
    t.add_section()
    add("Charge over temp", "temp_over", ("chg_alarm", "chg_protect", "chg_release"), "°C", 0)
    add("Discharge over temp", "temp_over", ("dis_alarm", "dis_protect", "dis_release"), "°C", 0)
    add("Charge under temp", "temp_under", ("chg_alarm", "chg_protect", "chg_release"), "°C", 0)
    add("Discharge under temp", "temp_under", ("dis_alarm", "dis_protect", "dis_release"), "°C", 0)
    add("MOSFET over temp", "mos_temp", ("alarm", "protect", "release"), "°C", 0)
    add("Ambient under temp", "env_temp", ("under_alarm", "under_protect", "under_release"), "°C", 0)
    add("Ambient over temp", "env_temp", ("over_alarm", "over_protect", "over_release"), "°C", 0)
    failed = any("error" in c for c in cfg.values())
    return Panel(t, title="Protection limits", title_align="left", border_style="magenta",
                 subtitle="n/a = not answered by this BMS (see --json)" if failed else None,
                 subtitle_align="right")


def other_panel(cfg):
    t = grid({"style": "dim"}, {"justify": "right"})

    def add(label, key, field, unit, nd=0):
        c = cfg.get(key, {})
        t.add_row(label, Text("n/a", style="dim red") if "error" in c else num(c[field], unit, nd))

    add("Balancing starts above", "balance", "start_V", "V", 2)
    add("Balancing cell difference", "balance", "delta_mV", "mV")
    add("Full-charge voltage", "full_charge", "voltage_V", "V", 2)
    add("Full-charge tail current", "full_charge", "current_mA", "mA")
    add("Low-charge alarm at", "full_charge", "low_soc_alarm_pct", "%")
    add("Sleep below cell voltage", "sleep", "cell_V", "V", 2)
    add("Sleep delay", "sleep", "delay_min", "min")
    add("Charge limiter starts at", "charge_limiter_start_A", "start_A", "A")
    return Panel(t, title="Other settings", title_align="left", border_style="magenta")


def view(d, cfg, port, baud, err, width, history=()):
    head = Text.assemble(("pace bms status ", "bold"), "%s @ %d baud   %s   " % (port, baud, time.strftime("%H:%M:%S")),
                         ("● ok", "green") if not err else
                         ("● %s (retrying, data may be stale)" % err, "bold red"))
    if d is None:
        return Group(head)
    left_w = width * 5 // 9 if width >= 150 else width
    left = Group(pack_panel(d, cfg), power_panel(history, time.time(), left_w - 8), cells_panel(d, cfg), temps_panel(d))
    right = Group(status_panel(d), limits_panel(cfg), other_panel(cfg)) if cfg else status_panel(d)
    if width >= 150:
        body = Table.grid(padding=(0, 1))
        body.add_column(ratio=5)
        body.add_column(ratio=4)
        body.add_row(left, right)
        return Group(head, body)
    return Group(head, left, right)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=9600)
    ap.add_argument("-a", "--adr", type=int, default=1, help="BMS address in the frame (ADS switch)")
    ap.add_argument("-n", "--pack", type=int, default=1, help="pack number for the command byte")
    ap.add_argument("-i", "--interval", type=float, default=2.0, help="seconds between polls")
    ap.add_argument("--once", action="store_true", help="poll once and exit")
    ap.add_argument("--no-settings", action="store_true", help="don't read the protection limits")
    ap.add_argument("--json", action="store_true", help="print one JSON object per poll")
    a = ap.parse_args()

    bms = Bms(a.port, a.baud, a.adr, a.pack)
    cfg = None if a.no_settings else bms.read_settings()
    console = Console()
    last = None
    history = deque()  # (unix time, power W) of successful polls

    def poll():
        nonlocal last
        try:
            last, err = bms.poll(), None
        except (TimeoutError, ValueError, RuntimeError) as e:
            err = str(e)
        if err is None:
            now = time.time()
            history.append((now, last["power_W"]))
            while history[0][0] < now - HISTORY_S:
                history.popleft()
        return err

    if a.json or a.once:
        err = poll()
        while True:
            if a.json:
                out = {"error": err} if err or last is None else dict(last, **({"settings": cfg} if cfg else {}))
                print(json.dumps(out), flush=True)
            else:
                console.print(view(last, cfg, a.port, a.baud, err, console.width))
            if a.once:
                sys.exit(1 if err else 0)
            time.sleep(a.interval)
            err = poll()

    try:
        with Live(console=console, screen=True, auto_refresh=False) as live:
            while True:
                err = poll()
                live.update(view(last, cfg, a.port, a.baud, err, console.width, history), refresh=True)
                time.sleep(a.interval)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
