import ctypes
import math
import os
import sys
import time
from ctypes import wintypes

# ---------- Type Aliases and Constants ----------
BYTE = ctypes.c_ubyte
WORD = ctypes.c_ushort
LONG = ctypes.c_long
UINT = ctypes.c_uint
DWORD = wintypes.DWORD
BOOL = wintypes.BOOL
PVOID = ctypes.c_void_p
FFB_CALLBACK = ctypes.WINFUNCTYPE(None, PVOID, PVOID)  # __stdcall cb(void*, void*)
ERROR_SUCCESS = 0

FFBEType_map = {
    1: "Constant", 2: "Ramp", 3: "Square", 4: "Sine", 5: "Triangle",
    6: "SawUp", 7: "SawDown", 8: "Spring", 9: "Damper", 10: "Inertia",
    11: "Friction", 12: "Custom"
}
EFFOP_NAMES = {1: "Start", 2: "Solo", 3: "Stop"}
DEVCTRL_NAMES = {1: "EnableActuators", 2: "DisableActuators", 3: "StopAll", 4: "Reset", 5: "Pause", 6: "Continue"}

# ---------- Ctypes Structures: Packed & Aligned Twin-Views ----------
class FFB_EFF_CONSTANT_P(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE), ("Magnitude", LONG)]
class FFB_EFF_CONSTANT_A(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("_pad", BYTE * 3), ("Magnitude", LONG)]

class FFB_EFF_PERIOD_P(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE),
                ("Magnitude", DWORD), ("Offset", LONG), ("Phase", DWORD), ("Period", DWORD)]
class FFB_EFF_PERIOD_A(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("_pad", BYTE * 3),
                ("Magnitude", DWORD), ("Offset", LONG), ("Phase", DWORD), ("Period", DWORD)]

class FFB_EFF_COND_P(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE), ("isY", BYTE),
                ("CenterPointOffset", LONG), ("PosCoeff", LONG), ("NegCoeff", LONG),
                ("PosSatur", DWORD), ("NegSatur", DWORD), ("DeadBand", LONG)]
class FFB_EFF_COND_A(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("isY", BOOL),
                ("CenterPointOffset", LONG), ("PosCoeff", LONG), ("NegCoeff", LONG),
                ("PosSatur", DWORD), ("NegSatur", DWORD), ("DeadBand", LONG)]

class FFB_EFF_REPORT_P(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE),
                ("EffectType", BYTE),
                ("Duration", WORD), ("TrigerRpt", WORD), ("SamplePrd", WORD),
                ("Gain", BYTE), ("TrigerBtn", BYTE),
                ("Polar", BOOL),
                ("DirX_or_Direction", BYTE), ("DirY", BYTE)]
class FFB_EFF_REPORT_A(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("_pad", BYTE * 3),
                ("EffectType", ctypes.c_int),
                ("Duration", WORD), ("TrigerRpt", WORD), ("SamplePrd", WORD),
                ("Gain", BYTE), ("TrigerBtn", BYTE),
                ("Polar", BOOL),
                ("DirX_or_Direction", BYTE), ("DirY", BYTE)]

class FFB_EFF_RAMP_P(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE), ("Start", LONG), ("End", LONG)]
class FFB_EFF_RAMP_A(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("_pad", BYTE * 3), ("Start", LONG), ("End", LONG)]

class FFB_EFF_ENV_P_W(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE), ("AttackLevel", WORD), ("FadeLevel", WORD),
                ("AttackTime", DWORD), ("FadeTime", DWORD)]
class FFB_EFF_ENV_A_W(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("_pad", BYTE * 3), ("AttackLevel", WORD), ("FadeLevel", WORD),
                ("AttackTime", DWORD), ("FadeTime", DWORD)]
class FFB_EFF_ENV_P_D(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("EffectBlockIndex", BYTE), ("AttackLevel", DWORD), ("FadeLevel", DWORD),
                ("AttackTime", DWORD), ("FadeTime", DWORD)]
class FFB_EFF_ENV_A_D(ctypes.Structure):
    _fields_ = [("EffectBlockIndex", BYTE), ("_pad", BYTE * 3), ("AttackLevel", DWORD), ("FadeLevel", DWORD),
                ("AttackTime", DWORD), ("FadeTime", DWORD)]

# ---------- Data Helpers ----------
def _u8(b, i): return b[i]
def _s16(b, i): return int.from_bytes(bytes((b[i], b[i + 1])), "little", signed=True)
def _in_range(v, lo, hi): return lo <= v <= hi

# ---------- Monitor ----------
class VJoyFfbMonitor:
    """
    Monitors a vJoy device for FFB commands, maintains the state of active effects,
    and calculates the total torque output.
    """
    def __init__(self, debug=False, effect_multipliers=None, effect_inversions=None,
                 usb_10k_to_nm: float = 2.0, **kwargs):
        """
        usb_10k_to_nm: mapping so that 10000 USB units corresponds to this many Nm.
        Backwards-compatible alias supported: usb_max_nm (same meaning).
        """
        # Back-compat alias
        if "usb_max_nm" in kwargs and kwargs["usb_max_nm"] is not None:
            try:
                usb_10k_to_nm = float(kwargs["usb_max_nm"])
            except Exception:
                pass

        self.debug = bool(debug)
        self.effect_multipliers = effect_multipliers if effect_multipliers is not None else {}
        self.effect_inversions = effect_inversions if effect_inversions is not None else set()

        self.dll = self._load_vjoy_dll()
        self._setup_ctypes()

        # State
        self.active_effects = {}   # block_index -> effect_state_dict
        self.packet_count = 0
        self._callback_ref = FFB_CALLBACK(self._ffb_callback)

        self.dev_enabled = True
        self.overall_gain = 255   # device gain (0..255)
        self.usb_10k_to_nm = float(usb_10k_to_nm)

    # ----- Public API -----
    def start(self):
        """Registers the FFB callback to start receiving events."""
        if not self.dll.vJoyEnabled():
            raise RuntimeError("vJoy driver not enabled or installed.")
        self.dll.FfbRegisterGenCB(self._callback_ref, None)
        if self.debug:
            print("[FFB] Callback registered (debug printing ON).")

    def get_current_torque(self, current_pos_turns):
        """
        Calculates the total FFB torque from all active effects.
        Returns torque in Newton-meters.
        """
        total_usb = 0.0
        now = time.perf_counter()

        for block_index, eff in list(self.active_effects.items()):
            if not eff.get("is_active", False):
                continue

            # Lifetime (Duration)
            duration_ms = eff.get("duration", 0xFFFF)
            if duration_ms != 0xFFFF:
                elapsed_ms = (now - eff.get("start_time", now)) * 1000.0
                if elapsed_ms > duration_ms:
                    del self.active_effects[block_index]
                    continue

            name = eff.get("name", "Unknown")
            usb = 0.0

            if name == "Constant":
                usb = eff.get("magnitude", 0)

            elif name == "Spring":
                pos_delta = current_pos_turns - eff.get("center_point_offset", 0.0)
                k = eff.get("pos_coeff", 0) if pos_delta >= 0 else eff.get("neg_coeff", 0)
                usb = k * pos_delta
                pos_sat = eff.get("pos_satur", 10000)
                neg_sat = eff.get("neg_satur", 10000)
                if usb > pos_sat: usb = pos_sat
                if usb < -neg_sat: usb = -neg_sat

            # Per-effect scaling: multiplier + inversion + per-effect gain (treat 0 as full)
            scale = self._get_effect_scale(name)
            eff_gain = eff.get("gain", 255)
            if eff_gain == 0:
                eff_gain = 255
            usb *= scale * (eff_gain / 255.0)

            total_usb += usb

        # Device-wide gain
        total_usb *= (self.overall_gain / 255.0)

        # Convert to Nm
        nm = total_usb * (self.usb_10k_to_nm / 10000.0)

        if self.debug:
            actives = sum(1 for e in self.active_effects.values() if e.get("is_active", False))
            print(f"[FFBsum] active={actives} usb={total_usb:+.0f} -> {nm:+.3f} Nm (dev_gain={self.overall_gain}/255, usb_max_nm={self.usb_10k_to_nm})")

        return nm

    # ----- Internals -----
    def _get_effect_scale(self, effect_name):
        """Gets the final scaling factor for an effect (multiplier * inversion)."""
        multiplier = self.effect_multipliers.get(effect_name, self.effect_multipliers.get("All", 1.0))
        try:
            scale = float(multiplier)
        except Exception:
            scale = 1.0
        scale = max(0.0, min(10.0, scale))
        if effect_name in self.effect_inversions:
            scale *= -1.0
        return scale

    def _load_vjoy_dll(self):
        """Finds and loads the vJoyInterface.dll."""
        here = os.path.abspath(os.path.dirname(__file__))
        candidates = [
            os.path.join(here, "vJoyInterface.dll"),
            os.environ.get("VJOY_DLL", ""),
            r"C:\Program Files\vJoy\x64\vJoyInterface.dll",
            r"C:\Program Files\vJoy\x86\vJoyInterface.dll",
        ]
        for path in candidates:
            if path and os.path.isfile(path):
                try:
                    dll = ctypes.WinDLL(path)
                    print(f"[init] Loaded vJoyInterface.dll from: {path}")
                    return dll
                except OSError:
                    continue
        try:
            dll = ctypes.WinDLL("vJoyInterface.dll")
            print("[init] Loaded vJoyInterface.dll from system PATH")
            return dll
        except OSError:
            raise FileNotFoundError("Could not load vJoyInterface.dll. Please ensure vJoy is installed.")

    def _setup_ctypes(self):
        """Sets up the argument and return types for DLL functions."""
        self.dll.vJoyEnabled.restype = BOOL
        self.dll.FfbRegisterGenCB.argtypes = [FFB_CALLBACK, PVOID]
        self.dll.FfbRegisterGenCB.restype = None

        ffb_handlers = [
            "Ffb_h_DevCtrl", "Ffb_h_DevGain", "Ffb_h_EffNew", "Ffb_h_EffOp",
            "Ffb_h_Eff_Report", "Ffb_h_Eff_Constant", "Ffb_h_Eff_Period", "Ffb_h_Eff_Cond",
            "Ffb_h_Eff_Envelope", "Ffb_h_Eff_Env", "Ffb_h_Eff_Ramp"
        ]
        self.ffb_procs = {}
        for name in ffb_handlers:
            proc = getattr(self.dll, name, None)
            if proc:
                proc.argtypes = [PVOID, PVOID]
                proc.restype = DWORD
                self.ffb_procs[name] = proc

    # ---------- Packet helpers ----------
    def _choose_report(self, raw):
        p = FFB_EFF_REPORT_P.from_buffer_copy(raw)
        a = FFB_EFF_REPORT_A.from_buffer_copy(raw)
        def is_valid(t): return 1 <= int(t) <= 12
        if is_valid(p.EffectType): return ("P", p)
        if is_valid(a.EffectType): return ("A", a)
        return ("P?", p)

    def _choose_constant(self, raw):
        p = FFB_EFF_CONSTANT_P.from_buffer_copy(raw)
        a = FFB_EFF_CONSTANT_A.from_buffer_copy(raw)
        candidates = {
            "A16": (a.EffectBlockIndex, _s16(raw, 4)),
            "P16": (p.EffectBlockIndex, _s16(raw, 1)),
            "A": (a.EffectBlockIndex, a.Magnitude),
            "P": (p.EffectBlockIndex, p.Magnitude),
        }
        def _in(v): return -10000 <= v <= 10000
        good = {k: v for k, v in candidates.items() if _in(v[1])}
        if good:
            for key in ["A16", "P16", "A", "P"]:
                if key in good and good[key][1] != 0:
                    blk, mag = good[key]
                    return (key, blk, mag)
            for key in ["A16", "P16", "A", "P"]:
                if key in good:
                    blk, mag = good[key]
                    return (key, blk, mag)
        return ("P?", p.EffectBlockIndex, max(-10000, min(10000, p.Magnitude)))

    def _choose_periodic(self, raw):
        p = FFB_EFF_PERIOD_P.from_buffer_copy(raw)
        a = FFB_EFF_PERIOD_A.from_buffer_copy(raw)
        def is_ok(x):
            return (0 <= int(x.Magnitude) <= 10000 and
                    -10000 <= int(x.Offset) <= 10000 and
                    0 <= int(x.Phase) <= 35999)
        if is_ok(p): return ("P", p)
        if is_ok(a): return ("A", a)
        return ("P?", p)

    def _choose_condition(self, raw):
        p = FFB_EFF_COND_P.from_buffer_copy(raw)
        a = FFB_EFF_COND_A.from_buffer_copy(raw)
        def is_ok(x):
            return (-10000 <= int(x.CenterPointOffset) <= 10000 and
                    -10000 <= int(x.PosCoeff) <= 10000 and -10000 <= int(x.NegCoeff) <= 10000 and
                    0 <= int(x.PosSatur) <= 10000 and 0 <= int(x.NegSatur) <= 10000 and
                    -10000 <= int(x.DeadBand) <= 10000)
        if is_ok(p): return ("P", p, (p.isY != 0))
        if is_ok(a): return ("A", a, bool(a.isY))
        return ("A?", a, bool(getattr(a, "isY", 0)))

    def _choose_ramp(self, raw):
        p = FFB_EFF_RAMP_P.from_buffer_copy(raw)
        a = FFB_EFF_RAMP_A.from_buffer_copy(raw)
        def is_ok(x): return -10000 <= int(x.Start) <= 10000 and -10000 <= int(x.End) <= 10000
        if is_ok(p): return ("P", p)
        if is_ok(a): return ("A", a)
        return ("P?", p)

    def _choose_envelope(self, raw):
        for cls, tag in [(FFB_EFF_ENV_P_W,"PW"), (FFB_EFF_ENV_A_W,"AW"), (FFB_EFF_ENV_P_D,"PD"), (FFB_EFF_ENV_A_D,"AD")]:
            v = cls.from_buffer_copy(raw)
            if (0 <= int(v.AttackLevel) <= 10000 and 0 <= int(v.FadeLevel) <= 10000 and
                0 <= int(v.AttackTime) <= 300000 and 0 <= int(v.FadeTime) <= 300000):
                return (tag, v)
        return ("PW?", FFB_EFF_ENV_P_W.from_buffer_copy(raw))

    # ---------- Callback ----------
    def _ffb_callback(self, pData, _):
        try:
            self.packet_count += 1
            buf = (BYTE * 64)()

            # Device control / gain
            if self.ffb_procs.get("Ffb_h_DevCtrl") and self.ffb_procs["Ffb_h_DevCtrl"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                cmd = _u8(buf, 0)
                if self.debug:
                    print(f"[FFB] DevCtrl={DEVCTRL_NAMES.get(cmd, cmd)}")
                self.dev_enabled = (cmd == 1 or cmd == 6)  # EnableActuators or Continue
                return

            if self.ffb_procs.get("Ffb_h_DevGain") and self.ffb_procs["Ffb_h_DevGain"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                self.overall_gain = _u8(buf, 0)
                if self.debug:
                    print(f"[FFB] DevGain={self.overall_gain}/255")
                return

            # New effect
            if self.ffb_procs.get("Ffb_h_EffNew") and self.ffb_procs["Ffb_h_EffNew"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                eff_type_id = _u8(buf, 0)
                effect_name = FFBEType_map.get(eff_type_id, "Unknown")
                if self.debug:
                    print(f"[FFB] NewEffect type={effect_name}({eff_type_id})")
                return

            # Effect operation (Start/Stop)
            if self.ffb_procs.get("Ffb_h_EffOp") and self.ffb_procs["Ffb_h_EffOp"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                block_index, op = _u8(buf, 0), _u8(buf, 1)
                if self.debug:
                    print(f"[FFB] EffOp idx={block_index} op={EFFOP_NAMES.get(op, op)}")
                if op == 1:  # Start
                    eff = self.active_effects.get(block_index)
                    if eff:
                        eff["is_active"] = True
                        eff["start_time"] = time.perf_counter()
                elif op == 3:  # Stop
                    eff = self.active_effects.get(block_index)
                    if eff:
                        eff["is_active"] = False
                return

            # Effect report (defines type, duration, gain)
            if self.ffb_procs.get("Ffb_h_Eff_Report") and self.ffb_procs["Ffb_h_Eff_Report"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                tag, r = self._choose_report(buf)
                blk = int(r.EffectBlockIndex)
                eff_name = FFBEType_map.get(int(r.EffectType), "Unknown")
                self.active_effects.setdefault(blk, {})
                self.active_effects[blk].update({
                    "name": eff_name,
                    "type_id": int(r.EffectType),
                    "duration": int(r.Duration),
                    "gain": int(r.Gain) if int(r.Gain) >= 0 else 255,
                    "is_active": self.active_effects[blk].get("is_active", False),
                })
                if self.debug:
                    print(f"[FFB] Report  #{blk} type={eff_name} dur={int(r.Duration)}ms gain={int(r.Gain)} dir={int(getattr(r,'DirX_or_Direction',1))}")
                return

            # Constant force update
            if self.ffb_procs.get("Ffb_h_Eff_Constant") and self.ffb_procs["Ffb_h_Eff_Constant"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                which, blk, mag = self._choose_constant(buf)
                self.active_effects.setdefault(blk, {"name": "Constant", "gain": 255, "duration": 0xFFFF})
                # NOTE: raw magnitude stored; scaling happens in get_current_torque
                self.active_effects[blk]["magnitude"] = int(mag)
                # Auto-start constants (some apps never send EffOp Start)
                if not self.active_effects[blk].get("is_active", False):
                    self.active_effects[blk]["is_active"] = True
                    self.active_effects[blk]["start_time"] = time.perf_counter()
                    if self.debug:
                        print(f"[FFB] Auto-START Constant#{blk} (no EffOp)")
                if self.debug:
                    print(f"[FFB] Constant#{blk} mag={mag} ({which})")
                return

            # Periodic update (stored; not synthesized here)
            if self.ffb_procs.get("Ffb_h_Eff_Period") and self.ffb_procs["Ffb_h_Eff_Period"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                which, p = self._choose_periodic(buf)
                blk = int(p.EffectBlockIndex)
                eff = self.active_effects.setdefault(blk, {"name": "Sine", "gain": 255, "duration": 0xFFFF})
                eff.update({
                    "magnitude": int(p.Magnitude),
                    "offset": int(p.Offset),
                    "phase_deg": int(p.Phase) / 100.0,
                    "period_ms": int(p.Period),
                })
                if self.debug:
                    print(f"[FFB] Periodic#{blk} mag={int(p.Magnitude)} off={int(p.Offset)} phase={int(p.Phase)} per={int(p.Period)} ({which})")
                return

            # Condition (treat as Spring by default)
            if self.ffb_procs.get("Ffb_h_Eff_Cond") and self.ffb_procs["Ffb_h_Eff_Cond"](pData, ctypes.byref(buf)) == ERROR_SUCCESS:
                which, c, isY = self._choose_condition(buf)
                blk = int(c.EffectBlockIndex)
                eff = self.active_effects.setdefault(blk, {"name": "Spring", "gain": 255, "duration": 0xFFFF})
                eff.update({
                    "center_point_offset": float(int(c.CenterPointOffset)) / 10000.0,  # turns
                    "pos_coeff": int(c.PosCoeff),
                    "neg_coeff": int(c.NegCoeff),
                    "pos_satur": int(c.PosSatur),
                    "neg_satur": int(c.NegSatur),
                    "isY": bool(isY),
                })
                # Auto-start Spring when condition arrives (common in test apps)
                if not eff.get("is_active", False):
                    eff["is_active"] = True
                    eff["start_time"] = time.perf_counter()
                    if self.debug:
                        print(f"[FFB] Auto-START Spring#{blk} (no EffOp)")
                if self.debug:
                    print(f"[FFB] Cond#{blk} Spring center={eff['center_point_offset']:+.3f}turn "
                          f"+K={eff['pos_coeff']} -K={eff['neg_coeff']} "
                          f"+Sat={eff['pos_satur']} -Sat={eff['neg_satur']} ({which})")
                return

        except Exception as e:
            print(f"[FFB Monitor Error] Callback failed: {e}", file=sys.stderr)
