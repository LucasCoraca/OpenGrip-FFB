import asyncio
import time
import ctypes
from ctypes import wintypes
import sys
import math

# Windows-only hotkeys
try:
    import msvcrt  # for non-blocking keyboard in console
    _HAS_MS = True
except Exception:
    _HAS_MS = False

# ---- ODrive imports ----
try:
    import odrive
    from odrive.enums import AxisState, ControlMode
    try:
        from odrive.enums import InputMode
    except Exception:
        InputMode = None
except ImportError:
    print("ERROR: ODrive library not found. Please install it with 'pip install odrive'", file=sys.stderr)
    sys.exit(1)

# ---------- vJoy out ----------
class VJoyOut:
    HID_USAGE_X = 0x30
    def __init__(self, device_id: int):
        self.device_id = device_id
        self.dll = None
        self.min_axis = 0
        self.max_axis = 32767

    def init(self) -> bool:
        try:
            self.dll = ctypes.WinDLL('vJoyInterface.dll')
            self.dll.AcquireVJD.argtypes = [wintypes.UINT]
            self.dll.AcquireVJD.restype = wintypes.BOOL
            self.dll.SetAxis.argtypes = [wintypes.LONG, wintypes.UINT, wintypes.UINT]
            self.dll.SetAxis.restype = wintypes.BOOL
        except (OSError, FileNotFoundError):
            print("ERROR: Could not load vJoyInterface.dll for position output.", file=sys.stderr)
            return False

        if not self.dll.AcquireVJD(self.device_id):
            print(f"ERROR: Could not acquire vJoy device {self.device_id} for output.", file=sys.stderr)
            return False

        print(f"[ODrive] Acquired vJoy device {self.device_id} for axis output.")
        return True

    def set_x_from_turns(self, turns: float, lock_to_lock_turns: float):
        """Map turns into vJoy X axis given a lock-to-lock in turns."""
        span_half = max(0.001, lock_to_lock_turns / 2.0)
        norm = max(-1.0, min(1.0, turns / span_half))
        rng = self.max_axis - self.min_axis
        v = int(((norm + 1.0) * 0.5) * rng + self.min_axis)
        self.dll.SetAxis(wintypes.LONG(v), wintypes.UINT(self.device_id), wintypes.UINT(self.HID_USAGE_X))

# ---------- helpers ----------
def _get_attr_path(base, path):
    try:
        obj = base
        for part in path.split('.'):
            obj = getattr(obj, part)
        return obj
    except Exception:
        return None

async def _safe_write(node, value, direct_target=None, direct_attr_name=None):
    try:
        if hasattr(node, "write"):
            return await node.write(value)
    except Exception:
        pass
    if direct_target is not None and direct_attr_name:
        try:
            setattr(direct_target, direct_attr_name, value)
            return
        except Exception:
            pass
    raise RuntimeError(f"Failed to write {direct_attr_name or 'property'}")

async def _safe_read(node, default=None):
    try:
        if hasattr(node, "read"):
            return await node.read()
    except Exception:
        pass
    try:
        return node
    except Exception:
        return default

# ---------- main ----------
class ODriveController:
    def __init__(
        self,
        ffb_monitor,
        odrive_axis: int,
        vjoy_device_id: int,
        ffb_gain: float = 1.0,
        test_sine_amp: float = 0.0,   # Nm
        test_sine_hz: float = 0.0,
        debug: bool = False,
        min_current_limit: float = 5.0,
        mode_override: str = "auto",
        force_hz: float = 800.0,
        pos_hz: float = 400.0,
        enable_hotkeys: bool = True,
        recenter_key: str = "c",
        zero_center_key: str = "z",
        lock_to_lock_deg: float = 900.0,   # NEW: L2L angle
        print_angle: bool = False,         # NEW: print angle
        # ---- NEW (optional) ----
        torque_polarity: int = +1,         # multiply all commanded torque by +1 or -1
        invert_torque_key: str = "i",      # hotkey to flip torque polarity at runtime
    ):
        self.ffb_monitor = ffb_monitor
        self.axis_num = odrive_axis
        self.vjoy = VJoyOut(vjoy_device_id)

        self.ffb_gain = float(ffb_gain)
        self.test_sine_amp = float(test_sine_amp)
        self.test_sine_hz = float(test_sine_hz)
        self.debug = bool(debug)
        self.min_current_limit = float(min_current_limit)
        self.mode_override = (mode_override or "auto").lower()

        self.force_hz = max(10.0, float(force_hz))
        self.pos_hz   = max(10.0, float(pos_hz))

        self.enable_hotkeys = bool(enable_hotkeys) and _HAS_MS
        self.recenter_key = (recenter_key or "c")[0]
        self.zero_center_key = (zero_center_key or "z")[0]
        self.invert_torque_key = (invert_torque_key or "i")[0]

        # Angle mapping
        self.lock_to_lock_deg = max(10.0, float(lock_to_lock_deg))
        self.lock_to_lock_turns = self.lock_to_lock_deg / 360.0  # used for vJoy mapping
        self.print_angle = bool(print_angle)

        self.odrv = None
        self.axis = None

        # Position state
        self.current_pos_turns = 0.0               # absolute from ODrive (turns)
        self.user_center_offset_turns = 0.0        # user-defined center (turns)
        self.relative_pos_turns = 0.0              # current_pos - center (turns)
        self.relative_pos_deg = 0.0                # derived degrees

        self._t0 = time.perf_counter()
        self._angle_last_print = 0.0

        # Torque command endpoints
        self._torque_prop_primary = None
        self._torque_prop_secondary = None
        self._torque_parent = None
        self._torque_attr_primary = None
        self._torque_attr_secondary = None

        # Current endpoints (for fallback)
        self._current_prop = None
        self._current_parent = None

        self._cmd_mode = "TORQUE"
        self._read_pos_fn = None
        self._torque_constant = None

        # efficacy detector
        self._eff_last_check = 0.0
        self._eff_time_above_threshold = 0.0

        # NEW: global torque polarity (+1 or -1) and debug throttle
        self.torque_polarity = -1 if int(torque_polarity) < 0 else +1
        self._ffb_dbg_last = 0.0  # throttle for debug prints

    # Public helper (non-breaking addition)
    def set_torque_polarity(self, polarity: int):
        """Set global torque polarity (+1 or -1)."""
        self.torque_polarity = -1 if int(polarity) < 0 else +1
        print(f"[Torque] Polarity set to {self.torque_polarity:+d}")

    # ---------- setup ----------
    async def _resolve_position_reader(self):
        async def _read_encoder_pos():
            node = _get_attr_path(self.axis, "encoder.pos_estimate")
            v = await _safe_read(node, default=None)
            return float(v) if v is not None else 0.0
        try:
            _ = await _safe_read(_get_attr_path(self.axis, "encoder.pos_estimate"), default=None)
            self._read_pos_fn = _read_encoder_pos
        except Exception:
            async def _read_axis_pos():
                node2 = _get_attr_path(self.axis, "pos_estimate")
                v2 = await _safe_read(node2, default=None)
                return float(v2) if v2 is not None else 0.0
            self._read_pos_fn = _read_axis_pos

    async def _ensure_current_limit(self):
        try:
            node = _get_attr_path(self.axis, "motor.config.current_lim")
            cur_lim = await _safe_read(node)
            if cur_lim is not None and cur_lim < self.min_current_limit:
                await _safe_write(node, self.min_current_limit, _get_attr_path(self.axis, "motor.config"), "current_lim")
                print(f"[ODrive] Raised motor.config.current_lim from {cur_lim} A to {self.min_current_limit} A")
            else:
                print(f"[ODrive] Current limit OK: {cur_lim} A")
            if cur_lim is not None and cur_lim >= 100.0:
                print("[ODrive] WARNING: Very high current limit configured; ensure this matches your hardware.")
        except Exception as e:
            print(f"[ODrive] Could not verify/set current limit: {e}", file=sys.stderr)

    async def _resolve_command_channels(self):
        # Decide control mode from override
        desired_mode = self.mode_override
        if desired_mode == "current":
            node = _get_attr_path(self.axis, "controller.config.control_mode")
            await _safe_write(node, ControlMode.CURRENT_CONTROL,
                              _get_attr_path(self.axis, "controller.config"), "control_mode")
            self._cmd_mode = "CURRENT"
        else:
            node = _get_attr_path(self.axis, "controller.config.control_mode")
            await _safe_write(node, ControlMode.TORQUE_CONTROL,
                              _get_attr_path(self.axis, "controller.config"), "control_mode")
            self._cmd_mode = "TORQUE"

        # InputMode PASSTHROUGH (0.5.x path; fallback to controller.input_mode)
        if self._cmd_mode == "TORQUE" and InputMode is not None:
            wrote_any = False
            node_cfg = _get_attr_path(self.axis, "controller.config.input_mode")
            try:
                if node_cfg is not None:
                    await _safe_write(node_cfg, InputMode.PASSTHROUGH,
                                      _get_attr_path(self.axis, "controller.config"), "input_mode")
                    wrote_any = True
            except Exception:
                pass
            if not wrote_any:
                node2 = _get_attr_path(self.axis, "controller.input_mode")
                try:
                    if node2 is not None:
                        await _safe_write(node2, InputMode.PASSTHROUGH,
                                          _get_attr_path(self.axis, "controller"), "input_mode")
                        wrote_any = True
                except Exception:
                    pass
            if wrote_any:
                print("[ODrive] InputMode PASSTHROUGH set.")

        # Resolve endpoints
        self._torque_parent = _get_attr_path(self.axis, "controller")
        p = _get_attr_path(self.axis, "controller.input_torque")
        s = _get_attr_path(self.axis, "controller.torque_setpoint")
        if p is None and s is None and self._cmd_mode == "TORQUE":
            raise RuntimeError("No torque endpoint found (controller.input_torque / torque_setpoint).")
        if p is not None:
            self._torque_prop_primary = p
            self._torque_attr_primary = "input_torque"
        elif s is not None:
            self._torque_prop_primary = s
            self._torque_attr_primary = "torque_setpoint"
        if s is not None and self._torque_attr_primary != "torque_setpoint":
            self._torque_prop_secondary = s
            self._torque_attr_secondary = "torque_setpoint"

        self._current_parent = _get_attr_path(self.axis, "controller")
        self._current_prop = _get_attr_path(self.axis, "controller.input_current") \
                             or _get_attr_path(self.axis, "controller.current_setpoint")

        # Torque constant
        try:
            kt = await _safe_read(_get_attr_path(self.axis, "motor.config.torque_constant"))
            self._torque_constant = kt
        except Exception:
            self._torque_constant = None

    async def _enter_closed_loop(self):
        node = _get_attr_path(self.axis, "requested_state")
        await _safe_write(node, AxisState.CLOSED_LOOP_CONTROL, self.axis, "requested_state")
        await asyncio.sleep(0.2)
        st = await _safe_read(_get_attr_path(self.axis, "current_state"))
        print(f"[ODrive] Axis state now: {st}")
        if st != AxisState.CLOSED_LOOP_CONTROL:
            print("WARNING: Axis did not enter CLOSED_LOOP_CONTROL.", file=sys.stderr)

    # ---------- tasks ----------
    async def _pos_reader_task(self):
        HZ = self.pos_hz
        per = 1.0 / HZ
        tnext = time.perf_counter()
        print(f"[ODrive] Starting position reader at {HZ:.0f} Hz.")
        while True:
            try:
                if self._read_pos_fn:
                    self.current_pos_turns = await self._read_pos_fn()
                    self.relative_pos_turns = self.current_pos_turns - self.user_center_offset_turns
                    # degrees from turns
                    self.relative_pos_deg = self.relative_pos_turns * 360.0
                    # update vJoy mapping using configured lock-to-lock
                    self.vjoy.set_x_from_turns(self.relative_pos_turns, self.lock_to_lock_turns)
                    # optional angle prints (throttled ~10 Hz)
                    if self.print_angle:
                        now = time.perf_counter()
                        if now - self._angle_last_print >= 0.1:
                            print(f"[Angle] {self.relative_pos_deg:+.1f}°  ({self.relative_pos_turns:+.4f} turns)")
                            self._angle_last_print = now
            except Exception as e:
                print(f"[ODrive Position Error] {e}", file=sys.stderr)
            tnext += per
            await asyncio.sleep(max(0.0, tnext - time.perf_counter()))

    async def _force_writer_task(self):
        HZ = self.force_hz
        per = 1.0 / HZ
        tnext = time.perf_counter()
        print(f"[ODrive] Starting force writer at {HZ:.0f} Hz (mode: {self._cmd_mode}).")

        while True:
            # Compose desired torque (Nm)
            if self.test_sine_amp > 0.0 and self.test_sine_hz > 0.0:
                t = time.perf_counter() - self._t0
                torque_nm = self.test_sine_amp * math.sin(2.0 * math.pi * self.test_sine_hz * t)
            else:
                # Use relative position so springs center at the user-defined zero
                torque_nm = float(self.ffb_monitor.get_current_torque(self.relative_pos_turns)) * self.ffb_gain

            # Apply global polarity (lets us correct sign issues fast)
            torque_nm *= self.torque_polarity

            # Lightweight sign debug: prints occasionally when commanding negative torque
            # (throttled to ~10 Hz) and shows Iq if available.
            if self.debug and torque_nm < 0:
                now_dbg = time.perf_counter()
                if now_dbg - self._ffb_dbg_last >= 0.1:
                    try:
                        iq_meas_dbg = await _safe_read(_get_attr_path(self.axis, "motor.current_control.Iq_measured"))
                        if isinstance(iq_meas_dbg, (int, float)):
                            print(f"[FFB] torque_cmd={torque_nm:+.3f} Nm  Iq_meas={iq_meas_dbg:+.2f} A")
                        else:
                            print(f"[FFB] torque_cmd={torque_nm:+.3f} Nm")
                    except Exception:
                        pass
                    self._ffb_dbg_last = now_dbg

            try:
                if self._cmd_mode == "TORQUE":
                    await _safe_write(self._torque_prop_primary, float(torque_nm), self._torque_parent, self._torque_attr_primary)
                    if self._torque_prop_secondary is not None:
                        try:
                            await _safe_write(self._torque_prop_secondary, float(torque_nm), self._torque_parent, self._torque_attr_secondary)
                        except Exception:
                            pass
                else:  # CURRENT
                    if self._torque_constant and self._torque_constant > 0 and self._current_prop is not None:
                        amps = float(torque_nm / self._torque_constant)
                        await _safe_write(self._current_prop, amps, self._current_parent, "input_current")
            except Exception as e:
                print(f"[ODrive Force Error] {e}", file=sys.stderr)

            # Efficacy detector (only when in TORQUE mode)
            try:
                if self._cmd_mode == "TORQUE" and isinstance(self._torque_constant, (int, float)) and self._torque_constant > 0:
                    expected_abs_A = abs(torque_nm) / self._torque_constant
                    now = time.perf_counter()
                    if expected_abs_A > 1.0:
                        if now - self._eff_last_check > 0.05:
                            iq_meas = await _safe_read(_get_attr_path(self.axis, "motor.current_control.Iq_measured"))
                            if isinstance(iq_meas, (int, float)) and abs(iq_meas) < 0.2:
                                self._eff_time_above_threshold += (now - self._eff_last_check)
                            else:
                                self._eff_time_above_threshold = 0.0
                            self._eff_last_check = now
                        if self._eff_time_above_threshold > 1.0 and self._current_prop is not None:
                            print("[ODrive] Torque path seems ineffective (Iq_meas ~ 0 A). Switching to CURRENT_CONTROL fallback.")
                            try:
                                node = _get_attr_path(self.axis, "controller.config.control_mode")
                                await _safe_write(node, ControlMode.CURRENT_CONTROL,
                                                  _get_attr_path(self.axis, "controller.config"), "control_mode")
                                self._cmd_mode = "CURRENT"
                            except Exception as e:
                                print(f"[ODrive] Could not enter CURRENT_CONTROL: {e}", file=sys.stderr)
                                self._eff_time_above_threshold = 0.0
                    else:
                        self._eff_time_above_threshold = 0.0
            except Exception:
                pass

            await asyncio.sleep(max(0.0, tnext + per - time.perf_counter()))
            tnext += per

    async def _hotkey_task(self):
        if not self.enable_hotkeys:
            return
        print(f"[Hotkeys] Press '{self.recenter_key.upper()}' to recenter (center = current), "
              f"'{self.zero_center_key.upper()}' to clear offset, "
              f"'{self.invert_torque_key.upper()}' to invert torque polarity.")
        while True:
            try:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    if not ch:
                        await asyncio.sleep(0.05)
                        continue
                    lo = ch.lower()
                    if lo == self.recenter_key.lower():
                        self.user_center_offset_turns = self.current_pos_turns
                        self.relative_pos_turns = self.current_pos_turns - self.user_center_offset_turns
                        self.relative_pos_deg = self.relative_pos_turns * 360.0
                        deg = self.user_center_offset_turns * 360.0
                        print(f"[Center] Recentered at {self.user_center_offset_turns:+.4f} turns ({deg:+.1f}°).")
                    elif lo == self.zero_center_key.lower():
                        self.user_center_offset_turns = 0.0
                        self.relative_pos_turns = self.current_pos_turns
                        self.relative_pos_deg = self.relative_pos_turns * 360.0
                        print("[Center] Cleared user center offset.")
                    elif lo == self.invert_torque_key.lower():
                        self.torque_polarity *= -1
                        print(f"[Torque] Polarity flipped. Now torque_polarity={self.torque_polarity:+d}")
                await asyncio.sleep(0.05)
            except Exception as e:
                print(f"[Hotkeys Error] {e}", file=sys.stderr)
                await asyncio.sleep(0.25)

    async def _telemetry_task(self):
        if not self.debug:
            return
        print("[ODrive] Telemetry enabled.")
        first = True
        while True:
            try:
                cur_lim = await _safe_read(_get_attr_path(self.axis, "motor.config.current_lim"))
                kt = await _safe_read(_get_attr_path(self.axis, "motor.config.torque_constant"))
                iq_meas = await _safe_read(_get_attr_path(self.axis, "motor.current_control.Iq_measured"))
                iq_set = await _safe_read(_get_attr_path(self.axis, "motor.current_control.Iq_setpoint"))
                vbus = await _safe_read(_get_attr_path(self.odrv, "vbus_voltage"))
                st = await _safe_read(_get_attr_path(self.axis, "current_state"))
                ax_err = await _safe_read(_get_attr_path(self.axis, "error"), default=0)
                m_err = await _safe_read(_get_attr_path(self.axis, "motor.error"), default=0)
                e_err = await _safe_read(_get_attr_path(self.axis, "encoder.error"), default=0)

                if first:
                    self._torque_constant = kt
                    print(f"[ODriveCfg] current_lim={cur_lim} A  torque_constant={kt} Nm/A")
                    if kt is None or (isinstance(kt, (int, float)) and kt <= 0.005):
                        print("[ODriveCfg] WARNING: torque_constant looks invalid/small; torque commands may have no effect.")
                    if cur_lim is not None and cur_lim >= 100.0:
                        print("[ODriveCfg] WARNING: Very high current limit configured; ensure this matches your hardware.")
                    first = False

                iq_meas_s = f"{iq_meas:+.2f}" if isinstance(iq_meas, (int, float)) else "n/a"
                iq_set_s = f"{iq_set:+.2f}" if isinstance(iq_set, (int, float)) else "n/a"
                vbus_s = f"{vbus:.2f}" if isinstance(vbus, (int, float)) else "n/a"
                print(f"[ODriveDbg] state={st}  vbus={vbus_s} V  Iq_set={iq_set_s} A  Iq_meas={iq_meas_s} A  "
                      f"axis_err=0x{int(ax_err or 0):08X} motor_err=0x{int(m_err or 0):08X} enc_err=0x{int(e_err or 0):08X}")
            except Exception as e:
                print(f"[ODriveDbg Error] {e}", file=sys.stderr)

            await asyncio.sleep(0.5)

    # ---------- run ----------
    async def run(self):
        if not self.vjoy.init():
            return

        print("[ODrive] Finding ODrive...")
        try:
            self.odrv = await odrive.find_async()
            self.axis = getattr(self.odrv, f"axis{self.axis_num}")
        except Exception as e:
            print(f"ERROR: Could not connect to ODrive or find axis{self.axis_num}: {e}", file=sys.stderr)
            return

        print(f"[ODrive] Connected. Configuring axis {self.axis_num} for force control.")
        try:
            await self._resolve_position_reader()
            await self._ensure_current_limit()
            await self._resolve_command_channels()
            await self._enter_closed_loop()
        except Exception as e:
            print(f"ERROR: Could not configure ODrive axis: {e}", file=sys.stderr)
            return

        print("[ODrive] Starting control loops...")
        try:
            await asyncio.gather(
                self._force_writer_task(),
                self._pos_reader_task(),
                self._hotkey_task(),
                self._telemetry_task()
            )
        except asyncio.CancelledError:
            print("[ODrive] Control loops cancelled.")
        finally:
            try:
                print("[ODrive] Ramping down and requesting IDLE.")
                # zero torque/current
                try:
                    if self._torque_prop_primary is not None:
                        await _safe_write(self._torque_prop_primary, 0.0, self._torque_parent, self._torque_attr_primary)
                except Exception:
                    pass
                try:
                    if self._torque_prop_secondary is not None:
                        await _safe_write(self._torque_prop_secondary, 0.0, self._torque_parent, self._torque_attr_secondary)
                except Exception:
                    pass
                try:
                    if self._current_prop is not None:
                        await _safe_write(self._current_prop, 0.0, self._current_parent, "input_current")
                except Exception:
                    pass
                # go idle
                try:
                    node = _get_attr_path(self.axis, "requested_state")
                    await _safe_write(node, AxisState.IDLE, self.axis, "requested_state")
                except Exception:
                    pass
            except Exception as e:
                print(f"[ODrive Shutdown Warning] {e}", file=sys.stderr)
