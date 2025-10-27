from __future__ import annotations

import os, json, sys, time, math, asyncio, threading
from collections import deque
from typing import Dict, Any, Optional, Set

from PySide6.QtCore import Qt, QTimer, QRectF, QSize, QPointF, QSettings, QStandardPaths, QUrl
from PySide6.QtGui import (
    QColor, QPainter, QFont, QPen, QFontDatabase, QAction, QIcon, QPixmap, QPainterPath, QDesktopServices
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QFrame, QGridLayout, QSizePolicy, QSlider, QDoubleSpinBox, QSpinBox,
    QTabWidget, QCheckBox, QLineEdit, QMessageBox, QComboBox, QStatusBar,
    QFormLayout, QToolButton, QLayout, QStyle, QInputDialog, QScrollArea
)

# ---------- Optional imports (mocked later if missing) ----------
try:
    from vjoy_ffb_monitor import VJoyFfbMonitor  # type: ignore
    from ffb_odrive_controller import ODriveController, _get_attr_path, _safe_write  # type: ignore
except Exception:
    VJoyFfbMonitor = None  # type: ignore
    ODriveController = None  # type: ignore
    _get_attr_path = None  # type: ignore
    _safe_write = None  # type: ignore

# ---------- ODrive axis state constants ----------
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3

# ---------- ODrive helpers ----------
AXIS_STATE_NAMES = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
    9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
}

def _fmt_hex(v: Optional[int]) -> str:
    try:
        return f"0x{int(v) & 0xFFFFFFFF:08X}"
    except Exception:
        return "—"


# --- ICON HELPERS -------------------------------------------------------------
def _maybe_logo_icon() -> Optional[QIcon]:
    """Use logo.png as the window icon if present in the same folder."""
    try:
        here = os.path.dirname(__file__)
    except NameError:
        here = os.getcwd()
    path = os.path.join(here, "logo.png")
    return QIcon(path) if os.path.exists(path) else None

def _std_icon(style: QStyle, sp: QStyle.StandardPixmap) -> QIcon:
    return style.standardIcon(sp)

def _icon_wheel(size: int = 16) -> QIcon:
    """Crisp 3-spoke steering wheel with rim + hub + small blue TDC tick."""
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)

    c = size / 2.0
    r_outer = int(size * 0.47)
    r_inner = int(size * 0.28)

    fg   = QColor("#F2F2F7")  # light UI foreground
    tick = QColor("#0A84FF")  # accent blue

    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)

    # Rim: donut (outer - inner) using OddEven fill
    rim = QPainterPath()
    rim.setFillRule(Qt.OddEvenFill)
    rim.addEllipse(QPointF(c, c), r_outer, r_outer)
    rim.addEllipse(QPointF(c, c), r_inner, r_inner)
    p.setPen(Qt.NoPen)
    p.setBrush(fg)
    p.drawPath(rim)

    # Spokes
    p.setPen(QPen(fg, max(2, size // 6), Qt.SolidLine, Qt.RoundCap))
    def spoke(angle_deg: float):
        rad = math.radians(angle_deg)
        x0, y0 = c, c
        x1 = c + math.cos(rad) * (r_outer - 2)
        y1 = c + math.sin(rad) * (r_outer - 2)
        p.drawLine(int(x0), int(y0), int(x1), int(y1))

    # Classic 3-spoke layout: top, bottom-left, bottom-right
    spoke(-90)   # up
    spoke(210)   # bottom-left
    spoke(330)   # bottom-right

    # Hub
    p.setPen(Qt.NoPen)
    p.setBrush(fg)
    p.drawEllipse(QPointF(c, c), max(2, size // 7), max(2, size // 7))

    # Tiny top-dead-center tick on the rim for readability
    p.setBrush(tick)
    tick_w = max(2, size // 6)
    tick_h = max(2, size // 14)
    p.drawRoundedRect(
        QRectF(c - tick_w / 2, c - r_outer + 1, tick_w, tick_h),
        1.2, 1.2
    )

    p.end()
    return QIcon(pm)

def _icon_effects(size: int = 16) -> QIcon:
    """Slider bar + knob."""
    pm = QPixmap(size, size); pm.fill(Qt.transparent)
    p = QPainter(pm); p.setRenderHint(QPainter.Antialiasing)
    bar = QColor("#9A9A9E"); knob = QColor("#F2F2F7")
    y = int(size*0.6); h = max(2, size//10); x0 = int(size*0.15); x1 = int(size*0.85)
    p.setPen(QPen(bar, h)); p.drawLine(x0, y, x1, y)
    p.setBrush(knob); p.setPen(Qt.NoPen)
    p.drawEllipse(QPointF(int(size*0.55), y), max(2, size//7), max(2, size//7))
    p.end()
    return QIcon(pm)

def _icon_chip(size: int = 16) -> QIcon:
    """Tiny IC chip with pins (for ODrive)."""
    pm = QPixmap(size, size); pm.fill(Qt.transparent)
    p = QPainter(pm); p.setRenderHint(QPainter.Antialiasing)
    body = QColor("#F2F2F7"); pins = QColor("#9A9A9E")
    rect = QRectF(size*0.25, size*0.25, size*0.5, size*0.5)
    p.setPen(QPen(body, 1)); p.setBrush(Qt.NoBrush); p.drawRoundedRect(rect, 2, 2)
    p.setPen(QPen(pins, 1))
    for i in range(3):
        y = rect.top() + (i+1)*rect.height()/4
        p.drawLine(rect.left()-3, y, rect.left(), y)
        p.drawLine(rect.right(), y, rect.right()+3, y)
    for i in range(3):
        x = rect.left() + (i+1)*rect.width()/4
        p.drawLine(x, rect.top()-3, x, rect.top())
        p.drawLine(x, rect.bottom(), x, rect.bottom()+3)
    p.end()
    return QIcon(pm)
# -----------------------------------------------------------------------------


# ---------- Typography helper ----------
def _font(size: float, weight: int = QFont.Normal):
    """Prefer SF/Segoe/Inter if installed, otherwise fall back gracefully."""
    f = QFont()
    f.setPointSizeF(size)
    f.setWeight(weight)
    for fam in ("SF Pro Text", "SF Pro Display", "Segoe UI Variable", "Inter", "Helvetica Neue", "Helvetica", "Arial"):
        try:
            if QFontDatabase.hasFamily(fam):
                f.setFamily(fam)
                break
        except Exception:
            try:
                if fam in QFontDatabase.families():
                    f.setFamily(fam)
                    break
            except Exception:
                pass
    return f


# ---------- Dark palette + robust QSS ----------
def _apply_qss(app: QApplication):
    bg     = "#1C1C1E" # Window background
    card   = "#2C2C2E" # Card background
    sub    = "#3A3A3D" # Input/Segment bar background
    border = "#4A4A4C" # Subtle borders
    text   = "#F2F2F7" # Primary text
    mute   = "#9A9A9E" # Muted text
    blue   = "#0A84FF" # Accent
    red    = "#FF453A" # Destructive

    app.setStyleSheet(f"""
        QMainWindow {{ background: {bg}; }}
        QWidget {{
            background: {bg};
            color: {text};
            font-family: 'SF Pro Text', 'Segoe UI Variable', 'Inter', 'Helvetica Neue', 'Helvetica', 'Arial';
            font-size: 13px;
        }}

        QFrame[class="card"] {{
            background: {card};
            border: 1px solid {border};
            border-radius: 14px;
        }}

        QFrame[class="segment-bar"] {{
            border-radius: 10px;
            padding: 6px 14px;
            min-height: 28px;
            border: 1px solid {border};
            background: {sub};
            font-weight: 500;
        }}
        QToolButton[class="segment"] {{
            background: transparent;
            color: {mute};
            font-size: 13px;
            font-weight: 500;
            padding: 4px 14px;
            min-height: 26px;
            border: none;
            border-radius: 7px;
        }}
        QToolButton[class="segment"]:hover {{ color: {text}; }}
        QToolButton[class="segment"]:checked {{ background: {border}; color: {text}; }}

        QLabel, QCheckBox, QRadioButton, QGroupBox::title {{ background: transparent; }}

        QPushButton {{
            border-radius: 10px;
            padding: 6px 14px;
            min-height: 28px;
            border: 1px solid {border};
            background: {sub};
            font-weight: 500;
        }}
        QPushButton:hover {{ border-color: {blue}; }}
        QPushButton:pressed {{ transform: translateY(1px); }}
        QPushButton:disabled {{ color: {mute}; opacity: 0.5; }}

        QPushButton[class="primary"] {{
            background: {blue};
            color: white;
            border: none;
            font-weight: 600;
        }}
        QPushButton[class="primary"]:hover {{ filter: brightness(1.05); }}
        QPushButton[class="primary"]:disabled {{ opacity: 0.55; }}

        QPushButton[class="run-toggle"] {{
            background: {blue};
            color: white;
            border: none;
            font-weight: 600;
            padding: 10px 18px;
            min-width: 120px;
        }}
        QPushButton[class="run-toggle"]:checked {{ background: {red}; }}

        QToolButton {{
            border-radius: 10px;
            padding: 6px;
            min-height: 28px;
            min-width: 28px;
            border: 1px solid {border};
            background: {sub};
        }}
        QToolButton:hover {{ border-color: {blue}; }}

        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {{
            background: {sub};
            border: 1px solid {border};
            border-radius: 8px;
            padding: 6px 10px;
            padding-right: 24px;
            min-height: 28px;
        }}
        QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{ width: 16px; }}
        QComboBox::drop-down {{ width: 18px; }}
        QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {{ border-color: {blue}; }}

        QFrame[class="card"] QLabel {{
            color: {mute};
            font-weight: 500;
            font-size: 13px;
        }}

        QSlider::groove:horizontal {{
            border: 1px solid {border}; height: 6px; border-radius: 4px; background: {sub};
        }}
        QSlider::handle:horizontal {{
            background: white; width: 14px; height:14px; margin: -5px 0; border-radius: 7px;
        }}

        QCheckBox::indicator {{ width: 18px; height: 18px; }}
        QCheckBox::indicator:checked {{ border: 2px solid {blue}; border-radius: 6px; background: {blue}; }}
        QCheckBox::indicator:unchecked {{ border: 2px solid {border}; border-radius: 6px; background: {sub}; }}

        QMenu {{
            background: {card};
            border: 1px solid {border};
            border-radius: 8px;
            padding: 4px;
        }}
        QMenu::item {{ padding: 6px 20px; border-radius: 5px; }}
        QMenu::item:selected {{ background: {blue}; color: white; }}
        QMenu::separator {{ height: 1px; background: {border}; margin: 4px 0px; }}

        QStatusBar {{ background: transparent; border-top: 1px solid {border}; padding-top: 4px; }}
        QStatusBar QLabel {{ background: transparent; }}

        QTabWidget::pane {{ border: none; margin-top: 8px; }}
        QTabWidget::tab-bar {{ alignment: left; }}
        QTabBar {{
            qproperty-drawBase: 0;
            background: {sub};
            border: 1px solid {border};
            border-radius: 10px;
            padding: 6px;
        }}
        QTabBar::tab {{
            background: transparent;
            color: {mute};
            padding: 4px 14px;
            margin-right: 6px;
            border: none;
            border-radius: 7px;
            min-height: 26px;
            font-weight: 500;
        }}
        QTabBar::tab:hover {{ color: {text}; }}
        QTabBar::tab:selected {{ background: {blue}; color: {text}; }}
        QTabBar::tab:!selected {{ margin-top: 1px; }}
    """)

# ---------- Profiles ----------
def _default_profiles_path() -> str:
    base = QStandardPaths.writableLocation(QStandardPaths.AppDataLocation)
    if not base:
        base = os.getcwd()
    base = os.path.join(base, "vjoy-ffb", "controller-ui")
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, "profiles.json")

class ProfilesStore:
    def __init__(self, path: Optional[str] = None):
        self.path = path or _default_profiles_path()
        self.db: Dict[str, Dict[str, Any]] = {}
        self._load()

    def _load(self):
        if os.path.isfile(self.path):
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    self.db = json.load(f)
            except Exception:
                self.db = {}

    def save(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self.db, f, indent=2)
        except Exception as e:
            print("Profile save failed:", e)

    def names(self):
        return sorted(self.db.keys())

    def get(self, name: str) -> Dict[str, Any] | None:
        return self.db.get(name)

    def put(self, name: str, payload: Dict[str, Any]):
        self.db[name] = payload
        self.save()

    def delete(self, name: str):
        if name in self.db:
            del self.db[name]
            self.save()


# ---------- Wheel visualization ----------
class WheelWidget(QWidget):
    """
    Draws a steering wheel rotated to current angle (deg) and a bar under it
    that grows left/right from center mapping [-L2L/2, +L2L/2].
    Adds center and end-stop tick marks and simple smoothing.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        # allow smaller window sizes; scroll areas handle overflow
        self.setMinimumSize(120, 120)
        self.angle_deg = 0.0
        self.lock_to_lock_deg = 900.0
        self._smooth = deque(maxlen=5)

    def setAngle(self, deg: float):
        self._smooth.append(float(deg))
        self.angle_deg = sum(self._smooth) / max(1, len(self._smooth))
        self.update()

    def setLockToLock(self, deg: float):
        self.lock_to_lock_deg = max(10.0, float(deg))
        self.update()

    def sizeHint(self):
        return QSize(480, 440)

    def paintEvent(self, ev):  # type: ignore[override]
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()

        # ---- Layout constants ----
        TOP_MARGIN = 10
        BOTTOM_MARGIN = 18
        BAR_W_RATIO = 0.84
        BAR_H = 8
        GAP_WHEEL_TO_TEXT = 18
        TEXT_H = 22
        GAP_TEXT_TO_BAR = 18

        cx = w / 2.0
        cy = h / 2.0 - 6

        reserved_bottom = GAP_WHEEL_TO_TEXT + TEXT_H + GAP_TEXT_TO_BAR + BAR_H + BOTTOM_MARGIN

        base_radius = min(w, h) * 0.34
        max_radius_by_bottom = max(28.0, h - cy - reserved_bottom)
        max_radius_by_top = max(28.0, cy - TOP_MARGIN)
        radius = max(28.0, min(base_radius, max_radius_by_bottom, max_radius_by_top))

        ring = QColor("#3A3A3D")
        spoke = QColor("#F2F2F7")
        accent = QColor("#0A84FF")
        ticks = QColor("#8E8E93")
        bar_bg = QColor("#2B2B2E")
        bar_border = QColor("#4A4A4C")

        p.setPen(QPen(ring, 10))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self.angle_deg)
        p.setPen(QPen(spoke, 6))
        p.drawLine(0, 0, 0, -radius + 10)
        for a_deg in (210, 330):
            a = math.radians(a_deg)
            p.drawLine(0, 0, math.cos(a) * (radius - 12), math.sin(a) * (radius - 12))
        p.restore()

        p.setPen(QPen(spoke, 2))
        p.setBrush(spoke)
        p.drawEllipse(QPointF(cx, cy), 10, 10)

        half = max(1.0, self.lock_to_lock_deg / 2.0)
        for d in (-half, 0.0, half):
            p.save()
            p.translate(cx, cy)
            p.rotate(-d)
            p.setPen(QPen(ticks, 2))
            p.drawLine(0, -radius - 4, 0, -radius - 12)
            p.restore()

        wheel_bottom = cy + radius
        bar_w = w * BAR_W_RATIO
        bx = int((w - bar_w) / 2)
        by = int(wheel_bottom + GAP_WHEEL_TO_TEXT + TEXT_H + GAP_TEXT_TO_BAR)
        by = min(by, h - BOTTOM_MARGIN - BAR_H)

        rect = QRectF(bx, by, bar_w, BAR_H)

        p.setPen(Qt.NoPen)
        p.setBrush(bar_bg)
        p.drawRoundedRect(rect, 6, 6)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(bar_border, 1))
        p.drawRoundedRect(rect.adjusted(0.5, 0.5, -0.5, -0.5), 6, 6)

        frac = max(-1.0, min(1.0, self.angle_deg / half))
        if abs(frac) > 1e-3:
            p.setPen(Qt.NoPen)
            p.setBrush(accent)
            if frac >= 0:
                p.drawRoundedRect(QRectF(bx + bar_w / 2, by, (bar_w / 2) * frac, BAR_H), 6, 6)
            else:
                p.drawRoundedRect(QRectF(bx + (bar_w / 2) * (1 + frac), by, -(bar_w / 2) * frac, BAR_H), 6, 6)

        p.setPen(spoke)
        p.setFont(_font(20, QFont.DemiBold))
        y_text = by - GAP_TEXT_TO_BAR - TEXT_H
        p.drawText(0, int(y_text), w, TEXT_H,
                   Qt.AlignHCenter | Qt.AlignVCenter,
                   f"{self.angle_deg:+.1f}°")
        p.end()



# ---------- Main Window ----------
class MainWindow(QMainWindow):
    EFFECTS = [
        "Constant", "Ramp", "Square", "Sine", "Triangle",
        "SawUp", "SawDown", "Spring", "Damper", "Inertia", "Friction", "Custom"
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenGrip FFB")
        # no aggressive minimum; you can resize freely
        # self.setMinimumWidth(660)  # removed

        # --- Icons: app + reusable ---
        if (ico := _maybe_logo_icon()):
            self.setWindowIcon(ico)
        st = self.style()
        self._icPlay = _std_icon(st, QStyle.SP_MediaPlay)
        self._icStop = _std_icon(st, QStyle.SP_MediaStop)

        # Runtime state
        self.monitor: Optional[VJoyFfbMonitor] = None  # type: ignore
        self.controller: Optional[ODriveController] = None  # type: ignore
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.task: Optional[asyncio.Task] = None
        self.thread: Optional[threading.Thread] = None
        self._loop_ready = threading.Event()
        self.running = False

        self._recenter_countdown_on_start = 10
        self.profiles = ProfilesStore()

        # ---------- UI ----------
        root = QWidget(); self.setCentralWidget(root)
        rootL = QVBoxLayout(root)
        rootL.setContentsMargins(12,12,12,8)
        rootL.setSpacing(10)
        # rootL.setSizeConstraint(QLayout.SetMinimumSize)  # removed

        # Header: profiles on the left; Run toggle on the right
        hdr = QHBoxLayout(); hdr.setSpacing(8)
        hdr.addWidget(QLabel("Profile"))
        self.cmbProfiles = QComboBox()
        self.cmbProfiles.setEditable(True)
        self.cmbProfiles.setInsertPolicy(QComboBox.NoInsert)
        self.cmbProfiles.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        hdr.addWidget(self.cmbProfiles, 1)
        self.btnLoad = QPushButton("Load")
        self.btnSave = QPushButton("Save")
        self.btnSaveAs = QPushButton("Save As…")
        self.btnDelete = QPushButton("Delete")
        hdr.addWidget(self.btnLoad); hdr.addWidget(self.btnSave)
        hdr.addWidget(self.btnSaveAs); hdr.addWidget(self.btnDelete)
        hdr.addStretch()

        self.btnRun = QPushButton("Start")
        self.btnRun.setCheckable(True)
        self.btnRun.setProperty("class", "run-toggle")
        self.btnRun.setIcon(self._icPlay)
        self.btnRun.setIconSize(QSize(18,18))
        hdr.addWidget(self.btnRun)
        rootL.addLayout(hdr)

        # Header button icons
        self.btnLoad.setIcon(_std_icon(self.style(), QStyle.SP_DirOpenIcon))
        self.btnSave.setIcon(_std_icon(self.style(), QStyle.SP_DialogSaveButton))
        self.btnSaveAs.setIcon(_std_icon(self.style(), QStyle.SP_FileIcon))
        self.btnDelete.setIcon(_std_icon(self.style(), QStyle.SP_TrashIcon))
        for b in (self.btnLoad, self.btnSave, self.btnSaveAs, self.btnDelete):
            b.setIconSize(QSize(16,16))

        # Profile actions
        self.btnLoad.clicked.connect(self.on_load_profile)
        self.btnSave.clicked.connect(self.on_save_profile)
        self.btnSaveAs.clicked.connect(self.on_saveas_profile)
        self.btnDelete.clicked.connect(self.on_delete_profile)

        # Tabs (each page will be wrapped in a QScrollArea)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.tabBar().setExpanding(False)
        self.tabs.setTabPosition(QTabWidget.North)
        rootL.addWidget(self.tabs)

        self._build_tab_main()
        self._build_tab_effects()
        self._build_tab_flight_stick()
        self._build_tab_about()
        # Status bar
        sb = QStatusBar(self); self.setStatusBar(sb)
        self.lblStatus = QLabel("ODrive: Disconnected")
        self.lblAxis = QLabel("Axis: —")
        sb.addPermanentWidget(self.lblAxis)
        sb.addWidget(self.lblStatus, 1)

        # Signals
        self.btnRun.toggled.connect(self.on_run_toggled)

        # Poll timer
        self.tmr = QTimer(self)
        self.tmr.timeout.connect(self._tick_ui)
        self.tmr.start(50)

        # Profiles list
        self._reload_profiles()

        # Shortcuts
        self._install_shortcuts()

        # Restore last session
        self._restore_settings()

        # Create the persistent asyncio loop thread up-front
        self._ensure_loop_thread()

        # Fit window height to content initially (user can resize smaller later)
        QTimer.singleShot(0, self.adjustSize)

    # Helper: wrap a page in a scroll area for overflow
    def _wrap_scroll(self, widget: QWidget) -> QScrollArea:
        sa = QScrollArea()
        sa.setWidget(widget)
        sa.setWidgetResizable(True)
        sa.setFrameShape(QFrame.NoFrame)
        sa.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        sa.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        return sa

    # ---------- Persistent asyncio loop thread ----------
    def _ensure_loop_thread(self):
        if self.thread and self.thread.is_alive() and self.loop:
            return
        self._loop_ready.clear()

        def _thread_target():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self._loop_ready.set()
            try:
                self.loop.run_forever()
            finally:
                try:
                    pending = asyncio.all_tasks(self.loop)
                    for t in pending: t.cancel()
                    self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                except Exception:
                    pass
                self.loop.close()
                self.loop = None

        self.thread = threading.Thread(target=_thread_target, daemon=True)
        self.thread.start()
        self._loop_ready.wait()

    # ---------------- Main tab ----------------
    def _build_tab_main(self):
        page = QWidget(); lay = QVBoxLayout(page); lay.setSpacing(12)

        # Left card
        left = QFrame(); left.setProperty("class","card")
        left.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        leftL = QVBoxLayout(left); leftL.setContentsMargins(16,14,16,16); leftL.setSpacing(12)

        grid = QGridLayout(); grid.setHorizontalSpacing(10); grid.setVerticalSpacing(8)
        self.cmbAxis = QComboBox(); self.cmbAxis.addItems(["0","1"]); self.cmbAxis.setCurrentIndex(1)
        self.btnCalibrate = QPushButton("Re-Center")
        self.btnZero = QPushButton("Zero Center")
        self.btnAxisCal = QPushButton("Calibrate Axis")

        self.btnCalibrate.setIcon(_std_icon(self.style(), QStyle.SP_BrowserReload))
        self.btnZero.setIcon(_std_icon(self.style(), QStyle.SP_DialogResetButton))
        self.btnAxisCal.setIcon(_std_icon(self.style(), QStyle.SP_ComputerIcon))
        for b in (self.btnCalibrate, self.btnZero, self.btnAxisCal):
            b.setIconSize(QSize(16,16))

        self.btnCalibrate.clicked.connect(self.on_calibrate)
        self.btnZero.clicked.connect(self.on_zero)
        self.btnAxisCal.clicked.connect(self.on_calibrate_axis)

        grid.addWidget(QLabel("Axis"), 0,0); grid.addWidget(self.cmbAxis, 0,1)
        grid.addWidget(self.btnCalibrate, 0,2)
        grid.addWidget(self.btnZero, 0,3)
        grid.addWidget(self.btnAxisCal, 0,4)
        leftL.addLayout(grid)

        form = QFormLayout(); form.setLabelAlignment(Qt.AlignRight)
        self.spnFfbNm = QDoubleSpinBox(); self.spnFfbNm.setRange(0.2, 25.0); self.spnFfbNm.setDecimals(1); self.spnFfbNm.setValue(8.0); self.spnFfbNm.setSuffix(" Nm")
        self.spnL2L = QSpinBox(); self.spnL2L.setRange(180, 1440); self.spnL2L.setValue(900); self.spnL2L.setSuffix(" °")
        form.addRow("Max Strength", self.spnFfbNm)
        form.addRow("Lock-to-Lock", self.spnL2L)
        
        # Moved here from ODrive tab: safety/current + loop rates
        self.spnCurrentLim = QDoubleSpinBox(); self.spnCurrentLim.setRange(1.0, 200.0); self.spnCurrentLim.setDecimals(1); self.spnCurrentLim.setValue(15.0); self.spnCurrentLim.setSuffix(" A")
        self.spnForceHz    = QDoubleSpinBox(); self.spnForceHz.setRange(10.0, 5000.0); self.spnForceHz.setDecimals(1); self.spnForceHz.setValue(1000.0); self.spnForceHz.setSuffix(" Hz")
        self.spnPosHz      = QDoubleSpinBox(); self.spnPosHz.setRange(10.0, 5000.0); self.spnPosHz.setDecimals(1); self.spnPosHz.setValue(1000.0); self.spnPosHz.setSuffix(" Hz")
        form.addRow("Current Limit", self.spnCurrentLim)
        form.addRow("Force Hz", self.spnForceHz)
        form.addRow("Position Hz", self.spnPosHz)
        leftL.addLayout(form)

        lay.addWidget(left)

        # Right card: wheel visualization
        right = QFrame(); right.setProperty("class","card")
        right.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        rightL = QVBoxLayout(right); rightL.setContentsMargins(16,14,16,16); rightL.setSpacing(8)
        self.wheel = WheelWidget()
        rightL.addWidget(self.wheel)
        lay.addWidget(right)

        idx = self.tabs.addTab(self._wrap_scroll(page), "Main")
        self.tabs.setTabIcon(idx, _icon_wheel(16))

    # ---------------- Effects tab ----------------
    def _build_tab_effects(self):
        page = QWidget(); outer = QVBoxLayout(page); outer.setSpacing(12)

        topBar = QHBoxLayout();
        self.sldMaster = QSlider(Qt.Horizontal); self.sldMaster.setRange(0, 300); self.sldMaster.setValue(100)
        self.spnMaster = QDoubleSpinBox(); self.spnMaster.setRange(0.0, 3.0); self.spnMaster.setSingleStep(0.05); self.spnMaster.setValue(1.0); self.spnMaster.setMaximumWidth(90)
        self.btnResetAll = QPushButton("Reset All")
        self.btnResetAll.setIcon(_std_icon(self.style(), QStyle.SP_BrowserReload))
        self.btnResetAll.setIconSize(QSize(16,16))
        self.sldMaster.valueChanged.connect(lambda v: self.spnMaster.setValue(round(v/100.0, 3)))
        self.spnMaster.valueChanged.connect(lambda v: self.sldMaster.setValue(int(round(v*100))))
        self.btnResetAll.clicked.connect(self._reset_effects)
        topBar.addWidget(QLabel("Master")); topBar.addWidget(self.sldMaster, 1); topBar.addWidget(self.spnMaster); topBar.addStretch(); topBar.addWidget(self.btnResetAll)
        outer.addLayout(topBar)

        card = QFrame(); card.setProperty("class","card")
        grid = QGridLayout(card); grid.setContentsMargins(16,14,16,16); grid.setHorizontalSpacing(12); grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 1)

        self.sldMult: Dict[str, QSlider] = {}
        self.spnMult: Dict[str, QDoubleSpinBox] = {}
        self.chkInvert: Dict[str, QCheckBox] = {}

        for i, name in enumerate(self.EFFECTS):
            grid.addWidget(QLabel(name), i, 0)
            sld = QSlider(Qt.Horizontal); sld.setRange(0, 300); sld.setValue(100)
            spn = QDoubleSpinBox(); spn.setRange(0.0, 3.0); spn.setSingleStep(0.05); spn.setValue(1.0); spn.setMaximumWidth(90)
            inv = QCheckBox("Invert")
            sld.valueChanged.connect(lambda v, s=spn: s.setValue(round(v/100.0, 3)))
            spn.valueChanged.connect(lambda v, s=sld: s.setValue(int(round(v*100))))
            grid.addWidget(sld, i, 1)
            grid.addWidget(spn, i, 2)
            grid.addWidget(inv, i, 3)
            self.sldMult[name] = sld
            self.spnMult[name] = spn
            self.chkInvert[name] = inv
            sld.setStyleSheet("QSlider { background: transparent; }"
                              "QSlider::sub-page:horizontal, QSlider::add-page:horizontal { background: transparent; border: none; }")
            sld.setAttribute(Qt.WA_TranslucentBackground, True)

        outer.addWidget(card)
        card.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        idx = self.tabs.addTab(self._wrap_scroll(page), "Effects")
        self.tabs.setTabIcon(idx, _icon_effects(16))

    def _reset_effects(self):
        self.sldMaster.setValue(100)
        for name in self.EFFECTS:
            self.sldMult[name].setValue(100)
            self.spnMult[name].setValue(1.0)
            self.chkInvert[name].setChecked(False)
        self.statusBar().showMessage("Effects reset to defaults", 2000)

    # ---------------- Flight Stick tab ----------------
    def _build_tab_flight_stick(self):
        page = QWidget()
        lay = QVBoxLayout(page); lay.setSpacing(12); lay.setContentsMargins(16,16,16,16)
        lbl = QLabel("Coming Soon! Support the project if you want this feature!")
        lbl.setWordWrap(True)
        lbl.setAlignment(Qt.AlignCenter)
        lay.addStretch(1); lay.addWidget(lbl); lay.addStretch(1)
        self.tabs.addTab(self._wrap_scroll(page), "Flight Stick")

    # ---------------- About tab ----------------
    def _build_tab_about(self):
        page = QWidget()
        lay = QVBoxLayout(page); lay.setSpacing(12); lay.setContentsMargins(16,16,16,16)

        desc = QLabel("<h1>Version:</h1><p>0.1 ALPHA</p><h1>About:</h1><p>OpenGrip-FFB is an open-source force-feedback controller and GUI for DIY steering wheels and joysticks, built with Python, ODrive and VJoy.</p>")
        desc.setWordWrap(True)

        author = QLabel("<h1>Author:</h1> <b>Lucas Coraça Silva</b>")
        author.setTextFormat(Qt.RichText)

        gh = QLabel('<a href="https://github.com/LucasCoraca/OpenGrip-FFB">github.com/LucasCoraca/OpenGrip-FFB</a>')
        gh.setTextFormat(Qt.RichText)
        gh.setOpenExternalLinks(True)

        btn = QPushButton("Support me on Patreon!")
        btn.clicked.connect(lambda: QDesktopServices.openUrl(QUrl("https://patreon.com/lucascoraca")))

        lay.addWidget(desc)
        lay.addWidget(author)
        lay.addWidget(gh)
        lay.addSpacing(8)
        lay.addWidget(btn)
        lay.addStretch(1)

        self.tabs.addTab(self._wrap_scroll(page), "About")

    # ---------------- ODrive Settings tab ----------------
    def _build_tab_odrive(self):
        page = QWidget(); outer = QVBoxLayout(page); outer.setSpacing(12)

        # -------- Settings card --------
        card = QFrame(); card.setProperty("class","card")
        form = QFormLayout(card); form.setLabelAlignment(Qt.AlignRight)

        # Already present (kept)
        self.spnCurrentLim   = QDoubleSpinBox(); self.spnCurrentLim.setRange(1.0, 200.0); self.spnCurrentLim.setDecimals(1); self.spnCurrentLim.setValue(15.0); self.spnCurrentLim.setSuffix(" A");      self.spnCurrentLim.setMaximumWidth(110)
        self.spnMaxTorqueNm  = QDoubleSpinBox(); self.spnMaxTorqueNm.setRange(0.1, 50.0); self.spnMaxTorqueNm.setDecimals(2); self.spnMaxTorqueNm.setValue(8.0);  self.spnMaxTorqueNm.setSuffix(" Nm");    self.spnMaxTorqueNm.setMaximumWidth(110)
        self.lblKt           = QLabel("— Nm/A")
        self.spnVelLimit     = QDoubleSpinBox(); self.spnVelLimit.setRange(0.1, 1000.0); self.spnVelLimit.setDecimals(2); self.spnVelLimit.setValue(20.0); self.spnVelLimit.setSuffix(" turns/s");       self.spnVelLimit.setMaximumWidth(110)
        self.spnForceHz      = QDoubleSpinBox(); self.spnForceHz.setRange(10.0, 5000.0); self.spnForceHz.setDecimals(1); self.spnForceHz.setValue(1000.0); self.spnForceHz.setSuffix(" Hz");             self.spnForceHz.setMaximumWidth(110)
        self.spnPosHz        = QDoubleSpinBox(); self.spnPosHz.setRange(10.0, 5000.0); self.spnPosHz.setDecimals(1); self.spnPosHz.setValue(1000.0); self.spnPosHz.setSuffix(" Hz");                     self.spnPosHz.setMaximumWidth(110)

        form.addRow("Current Limit", self.spnCurrentLim)
        form.addRow("Max Torque", self.spnMaxTorqueNm)
        form.addRow("Torque Constant Kt", self.lblKt)
        form.addRow("Max Velocity", self.spnVelLimit)
        form.addRow("Force Hz", self.spnForceHz)
        form.addRow("Position Hz", self.spnPosHz)

        # ----- Controller gains -----
        self.spnPosGain  = QDoubleSpinBox(); self.spnPosGain.setRange(0.0, 5000.0); self.spnPosGain.setDecimals(3); self.spnPosGain.setValue(20.0); self.spnPosGain.setMaximumWidth(110)
        self.spnVelGain  = QDoubleSpinBox(); self.spnVelGain.setRange(0.0, 1000.0); self.spnVelGain.setDecimals(5); self.spnVelGain.setValue(0.16); self.spnVelGain.setMaximumWidth(110)
        self.spnVelKi    = QDoubleSpinBox(); self.spnVelKi.setRange(0.0, 500.0);   self.spnVelKi.setDecimals(5);   self.spnVelKi.setValue(0.32);   self.spnVelKi.setMaximumWidth(110)
        form.addRow("Pos Gain", self.spnPosGain)
        form.addRow("Vel Gain", self.spnVelGain)
        form.addRow("Vel Integrator Gain", self.spnVelKi)

        # ----- Trapezoidal trajectory -----
        self.spnTrapVel   = QDoubleSpinBox(); self.spnTrapVel.setRange(0.0, 2000.0); self.spnTrapVel.setDecimals(2); self.spnTrapVel.setValue(100.0); self.spnTrapVel.setSuffix(" turns/s"); self.spnTrapVel.setMaximumWidth(110)
        self.spnTrapAcc   = QDoubleSpinBox(); self.spnTrapAcc.setRange(0.0, 10000.0); self.spnTrapAcc.setDecimals(2); self.spnTrapAcc.setValue(200.0); self.spnTrapAcc.setSuffix(" turns/s²"); self.spnTrapAcc.setMaximumWidth(110)
        self.spnTrapDec   = QDoubleSpinBox(); self.spnTrapDec.setRange(0.0, 10000.0); self.spnTrapDec.setDecimals(2); self.spnTrapDec.setValue(200.0); self.spnTrapDec.setSuffix(" turns/s²"); self.spnTrapDec.setMaximumWidth(110)
        form.addRow("Trap Vel Limit", self.spnTrapVel)
        form.addRow("Trap Accel Limit", self.spnTrapAcc)
        form.addRow("Trap Decel Limit", self.spnTrapDec)

        # ----- Motor / Encoder config -----
        self.spnCalibCurrent   = QDoubleSpinBox(); self.spnCalibCurrent.setRange(0.5, 60.0); self.spnCalibCurrent.setDecimals(2); self.spnCalibCurrent.setValue(10.0); self.spnCalibCurrent.setSuffix(" A"); self.spnCalibCurrent.setMaximumWidth(110)
        self.spnResCalVolt     = QDoubleSpinBox(); self.spnResCalVolt.setRange(1.0, 50.0);  self.spnResCalVolt.setDecimals(2); self.spnResCalVolt.setValue(10.0); self.spnResCalVolt.setSuffix(" V");  self.spnResCalVolt.setMaximumWidth(110)
        self.spnPolePairs      = QSpinBox();        self.spnPolePairs.setRange(1, 48); self.spnPolePairs.setValue(7); self.spnPolePairs.setMaximumWidth(110)

        self.spnEncCpr         = QSpinBox(); self.spnEncCpr.setRange(1, 262144); self.spnEncCpr.setValue(8192); self.spnEncCpr.setMaximumWidth(110)
        self.chkEncUseIndex    = QCheckBox("Use Index")
        self.spnEncBandwidth   = QDoubleSpinBox(); self.spnEncBandwidth.setRange(1.0, 5000.0); self.spnEncBandwidth.setDecimals(1); self.spnEncBandwidth.setValue(1000.0); self.spnEncBandwidth.setSuffix(" Hz"); self.spnEncBandwidth.setMaximumWidth(110)

        form.addRow("Motor Calib Current", self.spnCalibCurrent)
        form.addRow("Res Calib Max Voltage", self.spnResCalVolt)
        form.addRow("Motor Pole Pairs", self.spnPolePairs)
        form.addRow("Encoder CPR", self.spnEncCpr)
        form.addRow("Encoder Index", self.chkEncUseIndex)
        form.addRow("Encoder Bandwidth", self.spnEncBandwidth)

        # ----- Startup & watchdog -----
        self.chkStartupMotorCal   = QCheckBox("Startup Motor Calibration")
        self.chkStartupIndexSearch= QCheckBox("Startup Index Search")
        self.chkStartupClosedLoop = QCheckBox("Startup Closed Loop")
        self.spnWatchdog          = QDoubleSpinBox(); self.spnWatchdog.setRange(0.0, 10_000.0); self.spnWatchdog.setDecimals(3); self.spnWatchdog.setValue(0.0); self.spnWatchdog.setSuffix(" s"); self.spnWatchdog.setMaximumWidth(110)
        form.addRow("", self.chkStartupMotorCal)
        form.addRow("", self.chkStartupIndexSearch)
        form.addRow("", self.chkStartupClosedLoop)
        form.addRow("Watchdog Timeout", self.spnWatchdog)

        # Buttons row
        btnRow = QHBoxLayout()
        self.btnReadOdrive  = QPushButton("Read From ODrive")
        self.btnApplyOdrive = QPushButton("Apply to ODrive"); self.btnApplyOdrive.setProperty("class","primary")
        self.btnReadOdrive.setIcon(_std_icon(self.style(), QStyle.SP_BrowserReload))
        self.btnApplyOdrive.setIcon(_std_icon(self.style(), QStyle.SP_DialogApplyButton))
        self.btnReadOdrive.setIconSize(QSize(16,16)); self.btnApplyOdrive.setIconSize(QSize(16,16))
        btnRow.addWidget(self.btnReadOdrive); btnRow.addStretch(); btnRow.addWidget(self.btnApplyOdrive)
        form.addRow("", btnRow)

        self.btnApplyOdrive.clicked.connect(self.on_apply_odrive)
        self.btnReadOdrive.clicked.connect(self.on_read_odrive)

        outer.addWidget(card)

        # -------- Errors / status card --------
        stat = QFrame(); stat.setProperty("class","card")
        sform = QFormLayout(stat); sform.setLabelAlignment(Qt.AlignRight)

        self.lblAxisState = QLabel("—")
        self.lblAxisErr   = QLabel("—")
        self.lblMotorErr  = QLabel("—")
        self.lblEncErr    = QLabel("—")
        self.lblCtrlErr   = QLabel("—")

        sform.addRow("Axis State", self.lblAxisState)
        sform.addRow("Axis Error", self.lblAxisErr)
        sform.addRow("Motor Error", self.lblMotorErr)
        sform.addRow("Encoder Error", self.lblEncErr)
        sform.addRow("Controller Error", self.lblCtrlErr)

        sBtns = QHBoxLayout()
        self.btnRefreshStatus = QPushButton("Refresh Status")
        self.btnClearErrors   = QPushButton("Clear Errors")
        self.btnRefreshStatus.setIcon(_std_icon(self.style(), QStyle.SP_BrowserReload))
        self.btnClearErrors.setIcon(_std_icon(self.style(), QStyle.SP_DialogResetButton))
        self.btnRefreshStatus.setIconSize(QSize(16,16)); self.btnClearErrors.setIconSize(QSize(16,16))
        sBtns.addWidget(self.btnRefreshStatus); sBtns.addStretch(); sBtns.addWidget(self.btnClearErrors)
        sform.addRow("", sBtns)

        self.btnRefreshStatus.clicked.connect(self._refresh_odrive_status)
        self.btnClearErrors.clicked.connect(self.on_clear_odrive_errors)

        outer.addWidget(stat)

        idx = self.tabs.addTab(self._wrap_scroll(page), "ODrive Settings")
        self.tabs.setTabIcon(idx, _icon_chip(16))

    # ---------- Run toggle ----------
    def on_run_toggled(self, checked: bool):
        if checked and not self.running:
            self.on_start()
        elif not checked and self.running:
            self.on_stop()
        self.btnRun.setText("Stop" if checked else "Start")
        self.btnRun.setIcon(self._icStop if checked else self._icPlay)

    # ---------- Start / Stop ----------
    def on_start(self):
        if self.running:
            self.statusBar().showMessage("Already running", 1500)
            return

        if self.task and not self.task.done():
            self.statusBar().showMessage("Still stopping… try again in a moment", 2000)
            self.btnRun.setChecked(True)
            return

        self._ensure_loop_thread()

        inversions: Set[str] = {name for name, chk in self.chkInvert.items() if chk.isChecked()}
        multipliers = {"All": float(self.spnMaster.value())}
        for name, spn in self.spnMult.items():
            v = float(spn.value())
            if abs(v - 1.0) > 1e-6:
                multipliers[name] = v

        axis = int(self.cmbAxis.currentText())
        usb_nm = float(self.spnFfbNm.value())
        l2l  = float(self.spnL2L.value())
        current_lim = float(self.spnCurrentLim.value())
        force_hz = float(self.spnForceHz.value())
        pos_hz   = float(self.spnPosHz.value())

        async def runner():
            try:
                self.monitor = VJoyFfbMonitor(  # type: ignore
                    debug=False,
                    effect_multipliers=multipliers,
                    effect_inversions=inversions,
                    usb_max_nm=usb_nm,
                )
                if hasattr(self.monitor, "start"):
                    self.monitor.start()  # type: ignore

                self.controller = ODriveController(  # type: ignore
                    ffb_monitor=self.monitor,
                    odrive_axis=axis,
                    vjoy_device_id=1,
                    ffb_gain=1.0,
                    test_sine_amp=0.0,
                    test_sine_hz=0.0,
                    debug=True,
                    min_current_limit=current_lim,
                    mode_override="auto",
                    force_hz=force_hz,
                    pos_hz=pos_hz,
                    enable_hotkeys=True,
                    recenter_key="c",
                    zero_center_key="z",
                    lock_to_lock_deg=l2l,
                    print_angle=False,
                )
                await self.controller.run()
            except asyncio.CancelledError:
                pass
            except Exception as e:
                self._set_status(f"Error: {e}")
                QMessageBox.critical(self, "Run failed", str(e))
            finally:
                try:
                    if self.monitor and hasattr(self.monitor, "stop"):
                        self.monitor.stop()  # type: ignore[attr-defined]
                except Exception:
                    pass
                self.controller = None
                self.monitor = None

        def _create_task():
            self.task = asyncio.create_task(runner())

        self.loop.call_soon_threadsafe(_create_task)
        self.running = True
        self._set_status("ODrive: Connecting…", transient=False)

        self._recenter_countdown_on_start = 10

    def on_stop(self):
        if not self.running:
            return

        self._set_status("ODrive: Disconnecting…", transient=False)

        def _shutdown():
            async def _do():
                try:
                    if self.controller and getattr(self.controller, "axis", None):
                        node = _get_attr_path(self.controller.axis, "requested_state")  # type: ignore
                        await _safe_write(node, int(AXIS_STATE_IDLE), node, "requested_state")  # type: ignore
                except Exception as e:
                    print("ODrive idle request failed:", e)
                finally:
                    try:
                        if self.monitor and hasattr(self.monitor, "stop"):
                            self.monitor.stop()  # type: ignore[attr-defined]
                    except Exception:
                        pass
                    if self.task and not self.task.done():
                        self.task.cancel()
            asyncio.create_task(_do())

        if self.loop:
            self.loop.call_soon_threadsafe(_shutdown)

        self.running = False
        self._set_status("ODrive: Disconnected")

    # ---------- Calibrate / Zero ----------
    def on_calibrate(self):
        if self.controller:
            self.controller.user_center_offset_turns = self.controller.current_pos_turns
            self._set_status("Centered at current position")

    def on_zero(self):
        if self.controller:
            self.controller.user_center_offset_turns = 0.0
            self._set_status("Center cleared")

    def on_calibrate_axis(self):
        if not self.controller or not getattr(self.controller, "axis", None) or not self.loop:
            QMessageBox.information(self, "Not running",
                                    "Start the controller first, then click Calibrate Axis.")
            return

        async def do_cal():
            try:
                self._set_status("ODrive: Calibrating…", transient=False)
                axis = self.controller.axis
                node = _get_attr_path(axis, "requested_state")  # type: ignore
                await _safe_write(node, int(AXIS_STATE_FULL_CALIBRATION_SEQUENCE), node, "requested_state")  # type: ignore
                self.statusBar().showMessage("Calibration requested (state=3).", 3000)
            except Exception as e:
                self._set_status(f"Calibration failed: {e}")

        self.loop.call_soon_threadsafe(lambda: asyncio.create_task(do_cal()))

    # ---------- Profiles actions ----------
    def _reload_profiles(self):
        self.cmbProfiles.blockSignals(True)
        cur = self.cmbProfiles.currentText()
        self.cmbProfiles.clear()
        names = self.profiles.names()
        if names:
            self.cmbProfiles.addItems(names)
        if cur and cur in names:
            self.cmbProfiles.setCurrentText(cur)
        self.cmbProfiles.blockSignals(False)

    def _collect_profile_payload(self) -> Dict[str, Any]:
        effects = {name: {"mult": float(self.spnMult[name].value()), "invert": bool(self.chkInvert[name].isChecked())}
                   for name in self.EFFECTS}
        payload = {
            "axis": int(self.cmbAxis.currentText()),
            "usb_10k_nm": float(self.spnFfbNm.value()),
            "lock_to_lock_deg": int(self.spnL2L.value()),
            "master_mult": float(self.spnMaster.value()),
            "effects": effects,
            "odrive": {
                "current_lim": float(self.spnCurrentLim.value()),
                "max_torque_nm": float(self.spnMaxTorqueNm.value()),
                "vel_limit": float(self.spnVelLimit.value()),
                "force_hz": float(self.spnForceHz.value()),
                "pos_hz": float(self.spnPosHz.value()),
                "pos_gain": float(self.spnPosGain.value()),
                "vel_gain": float(self.spnVelGain.value()),
                "vel_integrator_gain": float(self.spnVelKi.value()),
                "trap_vel_limit": float(self.spnTrapVel.value()),
                "trap_accel_limit": float(self.spnTrapAcc.value()),
                "trap_decel_limit": float(self.spnTrapDec.value()),
                "motor_calibration_current": float(self.spnCalibCurrent.value()),
                "motor_resistance_calib_max_voltage": float(self.spnResCalVolt.value()),
                "motor_pole_pairs": int(self.spnPolePairs.value()),
                "encoder_cpr": int(self.spnEncCpr.value()),
                "encoder_use_index": bool(self.chkEncUseIndex.isChecked()),
                "encoder_bandwidth": float(self.spnEncBandwidth.value()),
                "startup_motor_calibration": bool(self.chkStartupMotorCal.isChecked()),
                "startup_encoder_index_search": bool(self.chkStartupIndexSearch.isChecked()),
                "startup_closed_loop_control": bool(self.chkStartupClosedLoop.isChecked()),
                "watchdog_timeout": float(self.spnWatchdog.value()),
            }
        }
        return payload

    def _apply_profile_payload(self, payload: Dict[str, Any]):
        try:
            self.cmbAxis.setCurrentText(str(payload.get("axis", 1)))
            self.spnFfbNm.setValue(float(payload.get("usb_10k_nm", 8.0)))
            self.spnL2L.setValue(int(payload.get("lock_to_lock_deg", 900)))
            self.spnMaster.setValue(float(payload.get("master_mult", 1.0)))

            effects = payload.get("effects", {})
            for name in self.EFFECTS:
                e = effects.get(name, {})
                self.spnMult[name].setValue(float(e.get("mult", 1.0)))
                self.chkInvert[name].setChecked(bool(e.get("invert", False)))

            od = payload.get("odrive", {})
            self.spnCurrentLim.setValue(float(od.get("current_lim", 15.0)))
            self.spnMaxTorqueNm.setValue(float(od.get("max_torque_nm", 8.0)))
            self.spnVelLimit.setValue(float(od.get("vel_limit", 20.0)))
            self.spnForceHz.setValue(float(od.get("force_hz", 1000.0)))
            self.spnPosHz.setValue(float(od.get("pos_hz", 1000.0)))

            self.spnPosGain.setValue(float(od.get("pos_gain", 20.0)))
            self.spnVelGain.setValue(float(od.get("vel_gain", 0.16)))
            self.spnVelKi.setValue(float(od.get("vel_integrator_gain", 0.32)))

            self.spnTrapVel.setValue(float(od.get("trap_vel_limit", 100.0)))
            self.spnTrapAcc.setValue(float(od.get("trap_accel_limit", 200.0)))
            self.spnTrapDec.setValue(float(od.get("trap_decel_limit", 200.0)))

            self.spnCalibCurrent.setValue(float(od.get("motor_calibration_current", 10.0)))
            self.spnResCalVolt.setValue(float(od.get("motor_resistance_calib_max_voltage", 10.0)))
            self.spnPolePairs.setValue(int(od.get("motor_pole_pairs", 7)))

            self.spnEncCpr.setValue(int(od.get("encoder_cpr", 8192)))
            self.chkEncUseIndex.setChecked(bool(od.get("encoder_use_index", False)))
            self.spnEncBandwidth.setValue(float(od.get("encoder_bandwidth", 1000.0)))

            self.chkStartupMotorCal.setChecked(bool(od.get("startup_motor_calibration", False)))
            self.chkStartupIndexSearch.setChecked(bool(od.get("startup_encoder_index_search", False)))
            self.chkStartupClosedLoop.setChecked(bool(od.get("startup_closed_loop_control", False)))
            self.spnWatchdog.setValue(float(od.get("watchdog_timeout", 0.0)))
        except Exception as e:
            QMessageBox.warning(self, "Profile load", f"Some values could not be applied:\n{e}")

    def on_load_profile(self):
        name = self.cmbProfiles.currentText().strip()
        if not name:
            QMessageBox.information(self, "Load Profile", "Choose a profile from the list.")
            return
        payload = self.profiles.get(name)
        if payload:
            self._apply_profile_payload(payload)
            self.statusBar().showMessage(f"Loaded profile '{name}'", 1500)
        else:
            QMessageBox.information(self, "Load Profile", f"Profile '{name}' not found.")

    def on_save_profile(self):
        name = self.cmbProfiles.currentText().strip()
        if not name:
            return self.on_saveas_profile()
        self.profiles.put(name, self._collect_profile_payload())
        self._reload_profiles()
        self.statusBar().showMessage(f"Saved profile '{name}'", 1500)

    def on_saveas_profile(self):
        suggested = self.cmbProfiles.currentText().strip()
        name, ok2 = QInputDialog.getText(self, "Save Profile As", "Profile name:", text=suggested)
        if not ok2 or not name.strip():
            return
        base = os.path.splitext(os.path.basename(name.strip()))[0]
        self.profiles.put(base, self._collect_profile_payload())
        self._reload_profiles()
        self.cmbProfiles.setCurrentText(base)
        self.statusBar().showMessage(f"Saved profile '{base}'", 1500)

    def on_delete_profile(self):
        name = self.cmbProfiles.currentText().strip()
        if name and QMessageBox.question(self, "Delete Profile", f"Delete '{name}'?") == QMessageBox.Yes:
            self.profiles.delete(name)
            self._reload_profiles()
            self.statusBar().showMessage(f"Deleted profile '{name}'", 1500)

    # ---------- Read helpers ----------
    async def _safe_read(self, node, default=None):
        try:
            if hasattr(node, "read"):
                val = await node.read()
                return val
            if hasattr(node, "read_value"):
                return await node.read_value()
            if hasattr(node, "value"):
                return node.value
        except Exception as e:
            print("Read failed:", e)
        return default

    def on_read_odrive(self):
        if not self.controller or not getattr(self.controller, "axis", None) or not self.loop:
            QMessageBox.information(self, "Not running", "Start the controller first to read ODrive settings.")
            return
        self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self._read_odrive_async()))

    async def _read_odrive_async(self):
        axis = self.controller.axis
        try:
            pos_gain = await self._safe_read(_get_attr_path(axis, "controller.config.pos_gain"))      # type: ignore
            vel_gain = await self._safe_read(_get_attr_path(axis, "controller.config.vel_gain"))      # type: ignore
            vel_ki   = await self._safe_read(_get_attr_path(axis, "controller.config.vel_integrator_gain"))  # type: ignore
            vel_limit= await self._safe_read(_get_attr_path(axis, "controller.config.vel_limit"))     # type: ignore

            trap_vel = await self._safe_read(_get_attr_path(axis, "trap_traj.config.vel_limit"))      # type: ignore
            trap_acc = await self._safe_read(_get_attr_path(axis, "trap_traj.config.accel_limit"))    # type: ignore
            trap_dec = await self._safe_read(_get_attr_path(axis, "trap_traj.config.decel_limit"))    # type: ignore

            cal_i   = await self._safe_read(_get_attr_path(axis, "motor.config.calibration_current")) # type: ignore
            rc_v    = await self._safe_read(_get_attr_path(axis, "motor.config.resistance_calib_max_voltage")) # type: ignore
            poles   = await self._safe_read(_get_attr_path(axis, "motor.config.pole_pairs"))          # type: ignore

            enc_cpr = await self._safe_read(_get_attr_path(axis, "encoder.config.cpr"))               # type: ignore
            enc_idx = await self._safe_read(_get_attr_path(axis, "encoder.config.use_index"))         # type: ignore
            enc_bw  = await self._safe_read(_get_attr_path(axis, "encoder.config.bandwidth"))         # type: ignore

            st_mcal = await self._safe_read(_get_attr_path(axis, "config.startup_motor_calibration")) # type: ignore
            st_idx  = await self._safe_read(_get_attr_path(axis, "config.startup_encoder_index_search")) # type: ignore
            st_cl   = await self._safe_read(_get_attr_path(axis, "config.startup_closed_loop_control"))  # type: ignore
            wd_s    = await self._safe_read(_get_attr_path(axis, "config.watchdog_timeout"))          # type: ignore

            def _apply():
                if pos_gain is not None: self.spnPosGain.setValue(float(pos_gain))
                if vel_gain is not None: self.spnVelGain.setValue(float(vel_gain))
                if vel_ki   is not None: self.spnVelKi.setValue(float(vel_ki))
                if vel_limit is not None: self.spnVelLimit.setValue(float(vel_limit))

                if trap_vel is not None: self.spnTrapVel.setValue(float(trap_vel))
                if trap_acc is not None: self.spnTrapAcc.setValue(float(trap_acc))
                if trap_dec is not None: self.spnTrapDec.setValue(float(trap_dec))

                if cal_i   is not None: self.spnCalibCurrent.setValue(float(cal_i))
                if rc_v    is not None: self.spnResCalVolt.setValue(float(rc_v))
                if poles   is not None: self.spnPolePairs.setValue(int(poles))

                if enc_cpr is not None: self.spnEncCpr.setValue(int(enc_cpr))
                if enc_idx is not None: self.chkEncUseIndex.setChecked(bool(enc_idx))
                if enc_bw  is not None: self.spnEncBandwidth.setValue(float(enc_bw))

                if st_mcal is not None: self.chkStartupMotorCal.setChecked(bool(st_mcal))
                if st_idx  is not None: self.chkStartupIndexSearch.setChecked(bool(st_idx))
                if st_cl   is not None: self.chkStartupClosedLoop.setChecked(bool(st_cl))
                if wd_s    is not None: self.spnWatchdog.setValue(float(wd_s))

                self.statusBar().showMessage("Read settings from ODrive", 1500)

            QTimer.singleShot(0, _apply)
        except Exception as e:
            QTimer.singleShot(0, lambda: self._set_status(f"Read failed: {e}"))

    # ---------- Status / errors ----------
    def _refresh_odrive_status(self):
        if not self.controller or not getattr(self.controller, "axis", None) or not self.loop:
            return
        self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self._refresh_odrive_status_async()))

    async def _refresh_odrive_status_async(self):
        axis = self.controller.axis
        try:
            st    = await self._safe_read(_get_attr_path(axis, "current_state"))  # type: ignore
            aerr  = await self._safe_read(_get_attr_path(axis, "error"))          # type: ignore
            merr  = await self._safe_read(_get_attr_path(axis, "motor.error"))    # type: ignore
            eerr  = await self._safe_read(_get_attr_path(axis, "encoder.error"))  # type: ignore
            cerr  = await self._safe_read(_get_attr_path(axis, "controller.error")) # type: ignore

            def _apply():
                if not hasattr(self, 'lblAxisState'):
                    return
                name = AXIS_STATE_NAMES.get(int(st) if st is not None else -1, "UNKNOWN")
                self.lblAxisState.setText(f"{name} ({int(st) if st is not None else '—'})")
                self.lblAxisErr.setText(_fmt_hex(aerr))
                self.lblMotorErr.setText(_fmt_hex(merr))
                self.lblEncErr.setText(_fmt_hex(eerr))
                self.lblCtrlErr.setText(_fmt_hex(cerr))
            QTimer.singleShot(0, _apply)
        except Exception as e:
            QTimer.singleShot(0, lambda: self.statusBar().showMessage(f"Status read failed: {e}", 2500))

    def on_clear_odrive_errors(self):
        if not self.controller or not getattr(self.controller, "axis", None) or not self.loop:
            QMessageBox.information(self, "Not running", "Start the controller first to clear errors.")
            return
        self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self._clear_errors_async()))

    async def _clear_errors_async(self):
        axis = self.controller.axis
        try:
            for path, parent in [
                ("error",                       "axis"),
                ("motor.error",                 "motor"),
                ("encoder.error",               "encoder"),
                ("controller.error",            "controller"),
            ]:
                node = _get_attr_path(axis, path)  # type: ignore
                await _safe_write(node, int(0), _get_attr_path(axis, parent), path.split(".")[-1])  # type: ignore

            QTimer.singleShot(0, lambda: self.statusBar().showMessage("ODrive errors cleared", 2000))
            await self._refresh_odrive_status_async()
        except Exception as e:
            QTimer.singleShot(0, lambda: self._set_status(f"Clear errors failed: {e}"))

    # ---------- Apply ODrive settings ----------
    def on_apply_odrive(self):
        if not self.controller or not getattr(self.controller, "axis", None) or not self.loop:
            QMessageBox.information(self, "Not running", "Start the controller first to write ODrive settings.")
            return

        desired_cur = float(self.spnCurrentLim.value())
        kt = getattr(self.controller, "_torque_constant", None)
        if isinstance(kt, (int, float)) and kt > 0:
            max_torque = float(self.spnMaxTorqueNm.value())
            desired_cur = max(desired_cur, max_torque / kt)

        vel_limit  = float(self.spnVelLimit.value())
        pos_gain   = float(self.spnPosGain.value())
        vel_gain   = float(self.spnVelGain.value())
        vel_ki     = float(self.spnVelKi.value())

        trap_vel   = float(self.spnTrapVel.value())
        trap_acc   = float(self.spnTrapAcc.value())
        trap_dec   = float(self.spnTrapDec.value())

        cal_i      = float(self.spnCalibCurrent.value())
        rc_v       = float(self.spnResCalVolt.value())
        poles      = int(self.spnPolePairs.value())

        enc_cpr    = int(self.spnEncCpr.value())
        enc_idx    = bool(self.chkEncUseIndex.isChecked())
        enc_bw     = float(self.spnEncBandwidth.value())

        st_mcal    = bool(self.chkStartupMotorCal.isChecked())
        st_idx     = bool(self.chkStartupIndexSearch.isChecked())
        st_cl      = bool(self.chkStartupClosedLoop.isChecked())
        wd_s       = float(self.spnWatchdog.value())

        async def writer():
            axis = self.controller.axis
            try:
                await _safe_write(_get_attr_path(axis, "motor.config.current_lim"), float(desired_cur), _get_attr_path(axis, "motor.config"), "current_lim")  # type: ignore
                await _safe_write(_get_attr_path(axis, "controller.config.vel_limit"), float(vel_limit), _get_attr_path(axis, "controller.config"), "vel_limit")  # type: ignore

                await _safe_write(_get_attr_path(axis, "controller.config.pos_gain"), float(pos_gain), _get_attr_path(axis, "controller.config"), "pos_gain")  # type: ignore
                await _safe_write(_get_attr_path(axis, "controller.config.vel_gain"), float(vel_gain), _get_attr_path(axis, "controller.config"), "vel_gain")  # type: ignore
                await _safe_write(_get_attr_path(axis, "controller.config.vel_integrator_gain"), float(vel_ki), _get_attr_path(axis, "controller.config"), "vel_integrator_gain")  # type: ignore

                await _safe_write(_get_attr_path(axis, "trap_traj.config.vel_limit"), float(trap_vel), _get_attr_path(axis, "trap_traj.config"), "vel_limit")  # type: ignore
                await _safe_write(_get_attr_path(axis, "trap_traj.config.accel_limit"), float(trap_acc), _get_attr_path(axis, "trap_traj.config"), "accel_limit")  # type: ignore
                await _safe_write(_get_attr_path(axis, "trap_traj.config.decel_limit"), float(trap_dec), _get_attr_path(axis, "trap_traj.config"), "decel_limit")  # type: ignore

                await _safe_write(_get_attr_path(axis, "motor.config.calibration_current"), float(cal_i), _get_attr_path(axis, "motor.config"), "calibration_current")  # type: ignore
                await _safe_write(_get_attr_path(axis, "motor.config.resistance_calib_max_voltage"), float(rc_v), _get_attr_path(axis, "motor.config"), "resistance_calib_max_voltage")  # type: ignore
                await _safe_write(_get_attr_path(axis, "motor.config.pole_pairs"), int(poles), _get_attr_path(axis, "motor.config"), "pole_pairs")  # type: ignore

                await _safe_write(_get_attr_path(axis, "encoder.config.cpr"), int(enc_cpr), _get_attr_path(axis, "encoder.config"), "cpr")  # type: ignore
                await _safe_write(_get_attr_path(axis, "encoder.config.use_index"), bool(enc_idx), _get_attr_path(axis, "encoder.config"), "use_index")  # type: ignore
                await _safe_write(_get_attr_path(axis, "encoder.config.bandwidth"), float(enc_bw), _get_attr_path(axis, "encoder.config"), "bandwidth")  # type: ignore

                await _safe_write(_get_attr_path(axis, "config.startup_motor_calibration"), bool(st_mcal), _get_attr_path(axis, "config"), "startup_motor_calibration")  # type: ignore
                await _safe_write(_get_attr_path(axis, "config.startup_encoder_index_search"), bool(st_idx), _get_attr_path(axis, "config"), "startup_encoder_index_search")  # type: ignore
                await _safe_write(_get_attr_path(axis, "config.startup_closed_loop_control"), bool(st_cl), _get_attr_path(axis, "config"), "startup_closed_loop_control")  # type: ignore
                await _safe_write(_get_attr_path(axis, "config.watchdog_timeout"), float(wd_s), _get_attr_path(axis, "config"), "watchdog_timeout")  # type: ignore

                self._set_status("ODrive: Settings applied.")
            except Exception as e:
                self._set_status(f"Apply failed: {e}")

        self.loop.call_soon_threadsafe(lambda: asyncio.create_task(writer()))

    # ---------- UI tick ----------
    def _tick_ui(self):
        if self.controller and self.monitor:
            try:
                self.controller.lock_to_lock_deg = float(self.spnL2L.value())
                self.controller.lock_to_lock_turns = self.controller.lock_to_lock_deg / 360.0

                val = float(self.spnFfbNm.value())
                if hasattr(self.monitor, "usb_10k_to_nm"):
                    self.monitor.usb_10k_to_nm = val
                if hasattr(self.monitor, "usb_max_nm"):
                    self.monitor.usb_max_nm = val

                
                kt = getattr(self.controller, "_torque_constant", None)
                if hasattr(self, "lblKt"):
                    if isinstance(kt, (int, float)) and kt > 0:
                        self.lblKt.setText(f"{kt:.3f} Nm/A")
                    else:
                        self.lblKt.setText("— Nm/A")
                ang = float(getattr(self.controller, "relative_pos_deg", 0.0))
                self.wheel.setAngle(ang)
                self.wheel.setLockToLock(float(self.spnL2L.value()))

                self._set_status("ODrive: Connected", transient=False)
                self.lblAxis.setText(f"Axis: {getattr(self.controller, 'axis_num', 1)}")

                if self._recenter_countdown_on_start > 0:
                    self._recenter_countdown_on_start -= 1
                    if self._recenter_countdown_on_start == 0:
                        try:
                            self.on_calibrate()
                            self.statusBar().showMessage("Centered at start", 2000)
                        except Exception as e:
                            print(f"Auto-recenter on start failed: {e}")
            except Exception:
                pass

        self._status_poll_ctr = getattr(self, "_status_poll_ctr", 0) + 1
        if self._status_poll_ctr % 10 == 0:
            if hasattr(self, 'lblAxisState'):
                self._refresh_odrive_status()

    def _set_status(self, text: str, transient: bool = True):
        self.lblStatus.setText(text)
        if transient:
            self.statusBar().showMessage(text, 1500)

    # ---------- Shortcuts & Settings ----------
    def _install_shortcuts(self):
        actRun = QAction("Start/Stop", self); actRun.setShortcut("Ctrl+R"); actRun.triggered.connect(lambda: self.btnRun.toggle())
        self.addAction(actRun)
        actCenter = QAction("Calibrate Center", self); actCenter.setShortcut("C"); actCenter.triggered.connect(self.on_calibrate); self.addAction(actCenter)
        actZero = QAction("Zero Center", self); actZero.setShortcut("Z"); actZero.triggered.connect(self.on_zero); self.addAction(actZero)

    def _restore_settings(self):
        s = QSettings("vjoy-ffb", "controller-ui")
        if (geo := s.value("geometry")):
            self.restoreGeometry(geo)
        if (state := s.value("windowState")):
            self.restoreState(state)
        if (tab := s.value("lastTab", 0, int)):
            self.tabs.setCurrentIndex(tab)
        last_profile = s.value("lastProfile", "", str)
        if last_profile:
            self.cmbProfiles.setCurrentText(last_profile)
        self.tabs.currentChanged.connect(lambda i: s.setValue("lastTab", i))

    def closeEvent(self, e):  # type: ignore[override]
        s = QSettings("vjoy-ffb", "controller-ui")
        s.setValue("geometry", self.saveGeometry())
        s.setValue("windowState", self.saveState())
        s.setValue("lastProfile", self.cmbProfiles.currentText())

        try:
            self.on_stop()
        except Exception:
            pass

        if self.loop:
            self.loop.call_soon_threadsafe(lambda: self.loop.stop() if self.loop and self.loop.is_running() else None)
        if self.thread:
            self.thread.join(timeout=0.75)

        super().closeEvent(e)


# ---------- Main ----------
def main():
    global VJoyFfbMonitor, ODriveController, _get_attr_path, _safe_write
    if VJoyFfbMonitor is None or ODriveController is None:
        print("Using mock VJoyFfbMonitor/ODriveController (real modules not found).")
        class VJoyFfbMonitor:  # type: ignore
            def __init__(self, **kwargs): print(f"VJoyFfbMonitor({kwargs})")
            def start(self): print("Monitor.start()")
            def stop(self): print("Monitor.stop()")
        class ODriveController:  # type: ignore
            def __init__(self, **kwargs):
                print(f"ODriveController({kwargs})")
                self.relative_pos_deg = 0.0
                self.current_pos_turns = 0.0
                self.axis_num = kwargs.get("odrive_axis", 1)
                self._torque_constant = 0.0827
                self.axis = "mock_axis_obj"
                self.lock_to_lock_deg = kwargs.get("lock_to_lock_deg", 900.0)
                self.user_center_offset_turns = 0.0
            async def run(self):
                print("Controller.run() - mocking connection")
                try:
                    while True:
                        self.relative_pos_deg = math.sin(time.time()) * (self.lock_to_lock_deg/2)
                        await asyncio.sleep(0.05)
                except asyncio.CancelledError:
                    print("Controller.run() cancelled.")
        def _get_attr_path(a, b): return f"mock.path.{b}"  # type: ignore
        async def _safe_write(n, v, p, l): print(f"SafeWrite: {n}={v}")  # type: ignore

    app = QApplication(sys.argv)
    _apply_qss(app)
    QSettings.setDefaultFormat(QSettings.IniFormat)
    QSettings.setPath(QSettings.IniFormat, QSettings.UserScope,
                      QStandardPaths.writableLocation(QStandardPaths.AppDataLocation))
    app.setOrganizationName("vjoy-ffb")
    app.setApplicationName("controller-ui")

    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
