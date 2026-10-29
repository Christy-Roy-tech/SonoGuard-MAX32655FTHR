#!/usr/bin/env python3
"""
SonoGuard dashboard - PySide6 + pyqtgraph, dark "Audacity 4" style.

Install:   pip install PySide6 pyqtgraph numpy pyserial
Run:       python sonoguard_gui.py
Pick the COM port and press Connect (close other serial programs first).
Pick "DEMO (simulated data)" to look at the interface without the board.

Reads the text protocol printed by main.c:
  I: info   S: spectrum   W: waveform envelope   T: status   A: alert
"""
import csv
import math
import sys
import time
from datetime import datetime

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QRectF, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFrame, QHBoxLayout, QHeaderView, QLabel,
    QMainWindow, QPlainTextEdit, QPushButton, QSplitter, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # demo mode still works without pyserial
    serial = None

# ---------------------------------------------------------------- constants
DEMO_PORT = "DEMO (simulated data)"
LOG_FILE = "sonoguard_review_log.csv"

SPEC_POINTS = 48          # must match main.c
HZ_PER_POINT = 93.75
WAVE_BLOCKS = 32          # must match main.c
BLOCK_SECONDS = 160 / 48000.0
WAVE_DIV = 16             # overwritten by the I: line
DB_FS = 140.0             # overwritten by the I: line
BAND_HZ = [515.0, 3984.0]  # overwritten by the I: line
WAVE_HISTORY_BLOCKS = 960  # about 3.2 s
SPECTROGRAM_COLUMNS = 140  # about 15 s

STATE_NAMES = {0: "LEARNING NOISE FLOOR", 1: "LISTENING", 2: "TONE PRESENT", 3: "PULSE CHAIN"}

# palette taken from Audacity 4 "modern" dark
BG = "#1f2329"
PANEL = "#2a2f36"
PANEL2 = "#343a43"
BORDER = "#3f4652"
TEXT = "#e8ebf0"
MUTED = "#9aa4b2"
BLUE = "#84b4ff"
RED = "#f0436b"
GREEN = "#35c28a"
AMBER = "#f5b84a"

QSS = f"""
QMainWindow, QWidget#root {{ background: {BG}; }}
QWidget {{ color: {TEXT}; font-family: 'Segoe UI', 'Inter', Arial; font-size: 13px; }}
QFrame#bar {{ background: {PANEL}; border-bottom: 1px solid {BORDER}; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 8px; }}
QLabel#cardTitle {{ color: {MUTED}; font-size: 11px; letter-spacing: 1px; }}
QLabel#cardValue {{ font-size: 24px; font-weight: 600; }}
QLabel#muted {{ color: {MUTED}; }}
QLabel#plotTitle {{ color: {MUTED}; font-size: 11px; letter-spacing: 1px; padding-left: 4px; }}
QPushButton {{ background: {PANEL2}; border: 1px solid {BORDER}; border-radius: 6px; padding: 6px 14px; }}
QPushButton:hover {{ background: #3d444f; }}
QPushButton#connect {{ background: {GREEN}; color: #0d1f17; font-weight: 600; border: none; }}
QPushButton#connect[on="true"] {{ background: {RED}; color: white; }}
QPushButton#real {{ padding: 2px 10px; }}
QPushButton#real:checked {{ background: {GREEN}; color: #0d1f17; border: none; }}
QPushButton#false {{ padding: 2px 10px; }}
QPushButton#false:checked {{ background: {RED}; color: white; border: none; }}
QComboBox {{ background: {PANEL2}; border: 1px solid {BORDER}; border-radius: 6px; padding: 5px 10px; min-width: 190px; }}
QComboBox QAbstractItemView {{ background: {PANEL2}; selection-background-color: {BLUE}; selection-color: #0b1220; }}
QTableWidget {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px; gridline-color: {BORDER}; }}
QHeaderView::section {{ background: {PANEL2}; color: {MUTED}; border: none; border-bottom: 1px solid {BORDER}; padding: 5px; }}
QPlainTextEdit {{ background: #171a1f; border: 1px solid {BORDER}; border-radius: 6px; color: #8fd9b6;
                  font-family: Consolas, 'Cascadia Mono', monospace; font-size: 11px; }}
QSplitter::handle {{ background: {BG}; }}
"""


# ---------------------------------------------------------------- parsing
def parse_line(line):
    """Return (kind, payload) or None. Never raises on bad lines."""
    tag, sep, body = line.partition(":")
    if not sep or len(tag) != 1:
        return None
    body = body.strip()
    try:
        if tag == "S":
            raw = bytes.fromhex(body)
            if len(raw) != SPEC_POINTS:
                return None
            return "spec", np.frombuffer(raw, dtype=np.uint8).astype(float) * (DB_FS / 255.0)
        if tag == "W":
            raw = bytes.fromhex(body)
            if len(raw) != 2 * WAVE_BLOCKS:
                return None
            v = (np.frombuffer(raw, dtype=np.uint8).astype(int) - 128) * WAVE_DIV
            return "wave", (v[0::2], v[1::2])
        if tag == "T":
            f = [int(x) for x in body.split(",")]
            if len(f) != 9:
                return None
            keys = ("rms", "peak_hz", "tone", "noise_db", "band_db", "state", "pulses", "overruns", "ms")
            return "status", dict(zip(keys, f))
        if tag == "A":
            f = body.split(",")
            if len(f) != 5:
                return None
            return "alert", {"type": f[0], "freq": int(f[1]), "tone": int(f[2]),
                             "detail": int(f[3]), "ms": int(f[4])}
        if tag == "I":
            f = body.split(",")
            if f[0] != "SG1" or len(f) != 9:
                return None
            return "info", {"bin_min": int(f[5]), "bin_max": int(f[6]),
                            "db_fs": float(f[7]), "wave_div": int(f[8])}
    except ValueError:
        return None
    return None


# ---------------------------------------------------------------- data sources
class _Source(QThread):
    spectrum = Signal(object)
    wave = Signal(object, object)
    status = Signal(dict)
    alert = Signal(dict)
    info = Signal(dict)
    console = Signal(str)
    link = Signal(bool, str)

    def __init__(self):
        super().__init__()
        self.running = True

    def stop(self):
        self.running = False
        self.wait(2000)


class SerialSource(_Source):
    def __init__(self, port, baud=115200):
        super().__init__()
        self.port, self.baud = port, baud

    def run(self):
        if serial is None:
            self.link.emit(False, "pyserial is not installed")
            return
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:  # noqa: BLE001
            self.link.emit(False, str(exc))
            return
        self.link.emit(True, f"Connected to {self.port}")
        try:
            while self.running:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                parsed = parse_line(line)
                if parsed is None:
                    self.console.emit(line)  # boot text and anything unknown
                    continue
                kind, data = parsed
                if kind == "spec":
                    self.spectrum.emit(data)
                elif kind == "wave":
                    self.wave.emit(*data)
                elif kind == "status":
                    self.status.emit(data)
                elif kind == "alert":
                    self.console.emit(line)
                    self.alert.emit(data)
                elif kind == "info":
                    self.info.emit(data)
        except Exception as exc:  # noqa: BLE001
            self.link.emit(False, f"Serial error: {exc}")
        finally:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass


class DemoSource(_Source):
    """Synthetic data in the same format. Clearly labelled SIMULATED in the window."""

    def run(self):
        self.link.emit(True, "DEMO: simulated data, not from the board")
        rng = np.random.default_rng(1)
        t0 = time.time()
        pulses_done = 0
        last_on = False
        overruns = 0
        while self.running:
            t = time.time() - t0
            cycle = t % 9.0
            # 0-3 s quiet, 3-8 s beeper 1000 Hz (0.4 s on / 0.4 s off), then quiet
            beeping = 3.0 <= cycle < 8.0 and ((cycle - 3.0) % 0.8) < 0.4
            if beeping and not last_on:
                pulses_done += 1
            if not beeping and cycle < 3.0:
                pulses_done = 0
            last_on = beeping

            spec = 38 + rng.normal(0, 2.5, SPEC_POINTS) + np.linspace(14, 0, SPEC_POINTS)
            if beeping:
                k = int(1000 / HZ_PER_POINT)
                spec[k] += 48
                spec[k - 1] += 26
                spec[k + 1] += 26
            self.spectrum.emit(np.clip(spec, 0, DB_FS))

            n = WAVE_BLOCKS
            tt = t + np.arange(n) * BLOCK_SECONDS
            amp = (700 if beeping else 40) + rng.normal(0, 6, n)
            sig = np.sin(2 * math.pi * 1000 * tt)
            self.wave.emit((-np.abs(amp * (0.6 + 0.4 * sig))).astype(int),
                           (np.abs(amp * (0.6 + 0.4 * sig))).astype(int))

            peak = 1000 if beeping else 0
            state = 3 if (not beeping and 0 < pulses_done < 3 and cycle < 8.5) else (2 if beeping else 1)
            self.status.emit({"rms": 600 if beeping else 38, "peak_hz": peak, "tone": 91 if beeping else 14,
                              "noise_db": 52, "band_db": 96 if beeping else 53, "state": state,
                              "pulses": min(pulses_done, 3), "overruns": overruns, "ms": int(t * 1000)})
            if beeping and pulses_done == 3 and ((cycle - 3.0) % 0.8) < 0.12 and not getattr(self, "_fired", False):
                self._fired = True
                self.alert.emit({"type": "PULSED", "freq": 1000, "tone": 91, "detail": 3, "ms": int(t * 1000)})
            if cycle < 2.0:
                self._fired = False
            time.sleep(0.107)


# ---------------------------------------------------------------- widgets
def make_card(title, value="--", color=TEXT):
    card = QFrame()
    card.setObjectName("card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(14, 10, 14, 10)
    lay.setSpacing(2)
    t = QLabel(title)
    t.setObjectName("cardTitle")
    v = QLabel(value)
    v.setObjectName("cardValue")
    v.setStyleSheet(f"color: {color};")
    lay.addWidget(t)
    lay.addWidget(v)
    return card, v


class ReviewCell(QWidget):
    changed = Signal(str)

    def __init__(self):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(6)
        self.b_real = QPushButton("Real")
        self.b_real.setObjectName("real")
        self.b_real.setCheckable(True)
        self.b_false = QPushButton("False alarm")
        self.b_false.setObjectName("false")
        self.b_false.setCheckable(True)
        lay.addWidget(self.b_real)
        lay.addWidget(self.b_false)
        self.b_real.clicked.connect(lambda: self._pick("real"))
        self.b_false.clicked.connect(lambda: self._pick("false"))

    def _pick(self, verdict):
        self.b_real.setChecked(verdict == "real")
        self.b_false.setChecked(verdict == "false")
        self.changed.emit(verdict)


class SonoGuardWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SonoGuard  -  Acoustic Hazard Monitor")
        self.resize(1280, 860)
        self.setStyleSheet(QSS)

        self.source = None
        self.demo = False
        self.last_data = 0.0
        self.alert_until = 0.0
        self.alert_text = ""
        self.records = []  # one dict per history row

        pg.setConfigOptions(antialias=True, background=BG, foreground=MUTED)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._build_bar())

        body = QWidget()
        body_l = QVBoxLayout(body)
        body_l.setContentsMargins(14, 12, 14, 12)
        body_l.setSpacing(10)
        self.banner = QLabel("SIMULATED DATA - this is not coming from the board")
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setStyleSheet(f"background:{RED}; color:white; font-weight:600; border-radius:6px; padding:4px;")
        self.banner.hide()
        body_l.addWidget(self.banner)
        body_l.addLayout(self._build_cards())

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self._build_plots())
        split.addWidget(self._build_history())
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        body_l.addWidget(split, 1)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(200)
        self.console.setFixedHeight(70)
        body_l.addWidget(self.console)
        outer.addWidget(body, 1)

        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._tick)
        self.watchdog.start(400)

        self.wmin = np.zeros(WAVE_HISTORY_BLOCKS)
        self.wmax = np.zeros(WAVE_HISTORY_BLOCKS)
        self.sgram = np.full((SPECTROGRAM_COLUMNS, SPEC_POINTS), 20.0)
        self._update_counters()

    # ---------------------------------------------------------- layout
    def _build_bar(self):
        bar = QFrame()
        bar.setObjectName("bar")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 10, 16, 10)
        lay.setSpacing(10)
        title = QLabel("SonoGuard")
        title.setFont(QFont("Segoe UI", 15, QFont.Bold))
        sub = QLabel("MAX32655FTHR edge acoustic hazard monitor")
        sub.setObjectName("muted")
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addStretch(1)

        self.port_box = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("connect")
        self.btn_connect.setProperty("on", False)
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.lbl_link = QLabel("Disconnected")
        self.lbl_link.setObjectName("muted")
        for w in (QLabel("Port"), self.port_box, self.btn_refresh, self.btn_connect, self.lbl_link):
            lay.addWidget(w)
        self.refresh_ports()
        return bar

    def _build_cards(self):
        row = QHBoxLayout()
        row.setSpacing(10)
        c1, self.v_freq = make_card("DOMINANT FREQUENCY", "--", BLUE)
        c2, self.v_tone = make_card("TONE PURITY", "--", AMBER)
        c3, self.v_level = make_card("BAND LEVEL / NOISE FLOOR", "--", TEXT)
        c4, self.v_state = make_card("DETECTOR STATE", "--", GREEN)
        c5, self.v_power = make_card("POWER", "USB", TEXT)
        self.v_power.setText("USB, no battery")
        self.v_power.setStyleSheet(f"color:{MUTED}; font-size:15px; font-weight:600;")
        self.v_state.setStyleSheet(f"color:{GREEN}; font-size:17px; font-weight:600;")
        for c in (c1, c2, c3, c4, c5):
            row.addWidget(c, 1)
        return row

    def _plot_block(self, title, plot):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        t = QLabel(title.upper())
        t.setObjectName("plotTitle")
        lay.addWidget(t)
        lay.addWidget(plot, 1)
        return w

    def _style_plot(self, p):
        p.setMenuEnabled(False)
        p.setMouseEnabled(False, False)
        p.hideButtons()
        p.getAxis("left").setPen(pg.mkPen(BORDER))
        p.getAxis("bottom").setPen(pg.mkPen(BORDER))
        p.getAxis("left").setTextPen(pg.mkPen(MUTED))
        p.getAxis("bottom").setTextPen(pg.mkPen(MUTED))

    def _build_plots(self):
        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        # waveform
        self.wave_plot = pg.PlotWidget()
        self._style_plot(self.wave_plot)
        self.wave_range = 2000
        self.wave_plot.setYRange(-self.wave_range, self.wave_range, padding=0)
        self.wave_x = (np.arange(WAVE_HISTORY_BLOCKS) - WAVE_HISTORY_BLOCKS) * BLOCK_SECONDS
        self.wave_plot.setXRange(self.wave_x[0], 0, padding=0)
        self.wave_plot.showGrid(x=False, y=True, alpha=0.15)
        self.cmin = pg.PlotCurveItem(self.wave_x, self.wmin_init(), pen=pg.mkPen(BLUE, width=1))
        self.cmax = pg.PlotCurveItem(self.wave_x, self.wmin_init(), pen=pg.mkPen(BLUE, width=1))
        self.wave_plot.addItem(self.cmin)
        self.wave_plot.addItem(self.cmax)
        self.wave_plot.addItem(pg.FillBetweenItem(self.cmin, self.cmax, brush=pg.mkBrush(132, 180, 255, 170)))
        top = QWidget()
        top_l = QHBoxLayout(top)
        top_l.setContentsMargins(0, 0, 0, 0)
        self.range_box = QComboBox()
        self.range_box.setMinimumWidth(110)
        for label, val in (("+/-  256", 256), ("+/-  1000", 1000), ("+/-  2000", 2000),
                           ("+/-  8000", 8000), ("+/-  32768", 32768)):
            self.range_box.addItem(label, val)
        self.range_box.setCurrentIndex(2)
        self.range_box.currentIndexChanged.connect(self._range_changed)
        top_l.addWidget(self._plot_block("Waveform (min / max envelope, last 3 s)", self.wave_plot), 1)
        lay.addWidget(top, 2)
        wr = QHBoxLayout()
        wr.addStretch(1)
        wr.addWidget(QLabel("Waveform range"))
        wr.addWidget(self.range_box)
        lay.addLayout(wr)

        # spectrum
        self.spec_plot = pg.PlotWidget()
        self._style_plot(self.spec_plot)
        self.spec_plot.setYRange(0, 120, padding=0)
        self.spec_plot.setXRange(0, SPEC_POINTS * HZ_PER_POINT, padding=0)
        self.spec_plot.showGrid(x=True, y=True, alpha=0.15)
        self.spec_plot.setLabel("bottom", "Hz")
        self.spec_plot.setLabel("left", "dB")
        edges = np.arange(SPEC_POINTS + 1) * HZ_PER_POINT
        self.spec_curve = pg.PlotCurveItem(edges, np.zeros(SPEC_POINTS), stepMode="center",
                                           pen=pg.mkPen(BLUE, width=1.5), fillLevel=0,
                                           brush=pg.mkBrush(132, 180, 255, 110))
        self.spec_plot.addItem(self.spec_curve)
        self.band = pg.LinearRegionItem(BAND_HZ, movable=False, brush=pg.mkBrush(53, 194, 138, 28),
                                        pen=pg.mkPen(GREEN, width=1, style=Qt.DashLine))
        self.spec_plot.addItem(self.band)
        self.peak_line = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(AMBER, width=1.5))
        self.spec_plot.addItem(self.peak_line)
        lay.addWidget(self._plot_block("Spectrum (green = band searched)", self.spec_plot), 2)

        # spectrogram
        self.sg_plot = pg.PlotWidget()
        self._style_plot(self.sg_plot)
        self.sg_img = pg.ImageItem()
        try:
            self.sg_img.setLookupTable(pg.colormap.get("inferno").getLookupTable(0.0, 1.0, 256))
        except Exception:  # noqa: BLE001
            pass
        span = SPECTROGRAM_COLUMNS * 0.107
        self.sg_img.setRect(QRectF(-span, 0, span, SPEC_POINTS * HZ_PER_POINT))
        self.sg_plot.addItem(self.sg_img)
        self.sg_plot.setXRange(-span, 0, padding=0)
        self.sg_plot.setYRange(0, SPEC_POINTS * HZ_PER_POINT, padding=0)
        self.sg_plot.setLabel("left", "Hz")
        self.sg_plot.setLabel("bottom", "seconds")
        lay.addWidget(self._plot_block("Spectrogram", self.sg_plot), 2)
        return holder

    def wmin_init(self):
        return np.zeros(WAVE_HISTORY_BLOCKS)

    def _build_history(self):
        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        t = QLabel("ALERT HISTORY AND REVIEW")
        t.setObjectName("plotTitle")
        lay.addWidget(t)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["Time", "Type", "Freq", "Tone", "Detail", "Review"])
        hh = self.table.horizontalHeader()
        for i in range(5):
            hh.setSectionResizeMode(i, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(5, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        lay.addWidget(self.table, 1)

        self.lbl_counts = QLabel("")
        self.lbl_counts.setObjectName("muted")
        self.lbl_counts.setWordWrap(True)
        lay.addWidget(self.lbl_counts)

        row = QHBoxLayout()
        self.btn_missed = QPushButton("I heard an alarm it missed")
        self.btn_missed.clicked.connect(self.mark_missed)
        self.btn_save = QPushButton("Save review CSV")
        self.btn_save.clicked.connect(self.save_csv)
        self.btn_clear = QPushButton("Clear display")
        self.btn_clear.clicked.connect(self.clear_display)
        row.addWidget(self.btn_missed)
        row.addWidget(self.btn_save)
        row.addWidget(self.btn_clear)
        lay.addLayout(row)
        note = QLabel("Mark every alert Real or False alarm. Counts and the CSV give your measured accuracy.")
        note.setObjectName("muted")
        note.setWordWrap(True)
        lay.addWidget(note)
        return holder

    # ---------------------------------------------------------- connection
    def refresh_ports(self):
        self.port_box.clear()
        if serial is not None:
            for p in serial.tools.list_ports.comports():
                self.port_box.addItem(f"{p.device}  -  {p.description}", p.device)
        self.port_box.addItem(DEMO_PORT, DEMO_PORT)
        # prefer a real port when there is one
        if self.port_box.count() > 1:
            self.port_box.setCurrentIndex(0)

    def toggle_connection(self):
        if self.source is not None:
            self.disconnect_source()
            return
        port = self.port_box.currentData()
        self.demo = (port == DEMO_PORT)
        self.source = DemoSource() if self.demo else SerialSource(port)
        s = self.source
        s.spectrum.connect(self.on_spectrum)
        s.wave.connect(self.on_wave)
        s.status.connect(self.on_status)
        s.alert.connect(self.on_alert)
        s.info.connect(self.on_info)
        s.console.connect(self.console.appendPlainText)
        s.link.connect(self.on_link)
        self.banner.setVisible(self.demo)
        self.last_data = time.time()
        s.start()
        self._set_connect_button(True)

    def disconnect_source(self):
        if self.source is not None:
            self.source.stop()
            self.source = None
        self.banner.hide()
        self._set_connect_button(False)
        self.lbl_link.setText("Disconnected")

    def _set_connect_button(self, on):
        self.btn_connect.setText("Disconnect" if on else "Connect")
        self.btn_connect.setProperty("on", on)
        self.btn_connect.style().unpolish(self.btn_connect)
        self.btn_connect.style().polish(self.btn_connect)

    def on_link(self, ok, msg):
        self.lbl_link.setText(msg)
        self.lbl_link.setStyleSheet(f"color:{GREEN if ok else RED};")
        if not ok:
            self.source = None
            self._set_connect_button(False)

    # ---------------------------------------------------------- data
    def on_info(self, d):
        global DB_FS, WAVE_DIV, BAND_HZ
        DB_FS = d["db_fs"]
        WAVE_DIV = d["wave_div"]
        BAND_HZ = [d["bin_min"] * 46.875, d["bin_max"] * 46.875]
        self.band.setRegion(BAND_HZ)

    def on_spectrum(self, db):
        self.last_data = time.time()
        self.spec_curve.setData(np.arange(SPEC_POINTS + 1) * HZ_PER_POINT, db)
        self.sgram = np.roll(self.sgram, -1, axis=0)
        self.sgram[-1, :] = db
        self.sg_img.setImage(self.sgram, levels=(20, 110), autoLevels=False)

    def on_wave(self, mins, maxs):
        self.last_data = time.time()
        self.wmin = np.concatenate((self.wmin[WAVE_BLOCKS:], mins.astype(float)))
        self.wmax = np.concatenate((self.wmax[WAVE_BLOCKS:], maxs.astype(float)))
        self.cmin.setData(self.wave_x, self.wmin)
        self.cmax.setData(self.wave_x, self.wmax)

    def on_status(self, s):
        self.last_data = time.time()
        self.v_freq.setText(f"{s['peak_hz']} Hz" if s["tone"] >= 30 else "--")
        self.v_tone.setText(f"{s['tone']} %")
        self.v_level.setText(f"{s['band_db']} / {s['noise_db']} dB")
        self.peak_line.setPos(s["peak_hz"])
        if time.time() < self.alert_until:
            return
        name = STATE_NAMES.get(s["state"], "?")
        if s["state"] == 3:
            name = f"PULSE {s['pulses']} of 3"
        colour = {0: AMBER, 1: GREEN, 2: BLUE, 3: AMBER}.get(s["state"], MUTED)
        if s["overruns"]:
            name += f"   (overruns {s['overruns']})"
        self.v_state.setText(name)
        self.v_state.setStyleSheet(f"color:{colour}; font-size:17px; font-weight:600;")

    def on_alert(self, a):
        self.alert_until = time.time() + 2.5
        self.v_state.setText(f"ALERT: {a['type']}  {a['freq']} Hz")
        self.v_state.setStyleSheet(f"color:{RED}; font-size:17px; font-weight:700;")
        self._add_row(a)

    # ---------------------------------------------------------- history
    def _add_row(self, a, missed=False):
        rec = {"pc_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"), "board_ms": a.get("ms", ""),
               "type": a["type"], "freq": a.get("freq", ""), "tone": a.get("tone", ""),
               "detail": a.get("detail", ""), "verdict": "missed" if missed else "unreviewed",
               "simulated": self.demo}
        self.records.append(rec)
        row = self.table.rowCount()
        self.table.insertRow(row)
        detail = ""
        if not missed:
            detail = f"{a['detail']} pulses" if a["type"] == "PULSED" else f"sweep {a['detail']} Hz"
        for col, text in enumerate((rec["pc_time"][11:], a["type"] + (" (sim)" if self.demo else ""),
                                    f"{a['freq']} Hz" if not missed else "", f"{a['tone']} %" if not missed else "",
                                    detail)):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, col, item)
        if missed:
            lbl = QLabel("missed by detector")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f"color:{AMBER};")
            self.table.setCellWidget(row, 5, lbl)
        else:
            cell = ReviewCell()
            idx = len(self.records) - 1
            cell.changed.connect(lambda verdict, i=idx: self._set_verdict(i, verdict))
            self.table.setCellWidget(row, 5, cell)
        self.table.scrollToBottom()
        self._update_counters()
        self.save_csv()

    def _set_verdict(self, idx, verdict):
        self.records[idx]["verdict"] = verdict
        self._update_counters()
        self.save_csv()

    def mark_missed(self):
        self._add_row({"type": "MISSED", "ms": self.last_status_ms()}, missed=True)

    def last_status_ms(self):
        return ""

    def _update_counters(self):
        real = sum(1 for r in self.records if r["verdict"] == "real")
        false = sum(1 for r in self.records if r["verdict"] == "false")
        unrev = sum(1 for r in self.records if r["verdict"] == "unreviewed")
        missed = sum(1 for r in self.records if r["verdict"] == "missed")
        text = f"Alerts {real + false + unrev}   Real {real}   False {false}   Unreviewed {unrev}   Missed {missed}"
        if real + false:
            text += f"\nPrecision of reviewed alerts: {100.0 * real / (real + false):.0f} %"
        if real + missed:
            text += f"   Recall: {100.0 * real / (real + missed):.0f} %"
        self.lbl_counts.setText(text)

    def save_csv(self):
        try:
            with open(LOG_FILE, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=["pc_time", "board_ms", "type", "freq", "tone",
                                                  "detail", "verdict", "simulated"])
                w.writeheader()
                w.writerows(self.records)
        except OSError as exc:
            self.console.appendPlainText(f"Could not write {LOG_FILE}: {exc}")

    def clear_display(self):
        self.table.setRowCount(0)
        self.records.clear()
        self._update_counters()

    # ---------------------------------------------------------- misc
    def _range_changed(self):
        self.wave_range = self.range_box.currentData()
        self.wave_plot.setYRange(-self.wave_range, self.wave_range, padding=0)

    def _tick(self):
        if self.source is not None and not self.demo and time.time() - self.last_data > 2.0:
            self.lbl_link.setText("Connected, but no data. Press SW2 on the board.")
            self.lbl_link.setStyleSheet(f"color:{AMBER};")

    def closeEvent(self, event):  # noqa: N802
        self.disconnect_source()
        self.save_csv()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    win = SonoGuardWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()