import argparse
import asyncio
import sys

from vjoy_ffb_monitor import VJoyFfbMonitor
from ffb_odrive_controller import ODriveController

def parse_multipliers(arg_list):
    """Helper to parse key=value pairs for effect multipliers."""
    if not arg_list:
        return {}
    d = {}
    for item in arg_list:
        try:
            key, value = item.split('=')
            d[key] = float(value)
        except ValueError:
            print(f"WARNING: Invalid format for multiplier '{item}'. Should be EffectName=Value. Skipping.")
    return d

async def main():
    ap = argparse.ArgumentParser(
        description="vJoy FFB to ODrive real-time controller.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    ap.add_argument("--vjoy-id", type=int, default=1, help="vJoy device ID to monitor and output to.")
    ap.add_argument("--odrive-axis", type=int, choices=[0, 1], default=1, help="Which ODrive axis to control (0 or 1).")

    # FFB shaping
    ap.add_argument("--strength", nargs='*', metavar="EffectName=Value",
                    help="Set strength multipliers for effects (0.0 to 10.0).\n"
                         "Example: --strength Constant=0.8 Spring=1.2 All=1.5")
    ap.add_argument("--invert", nargs='*', metavar="EffectName",
                    help="Invert the force direction for specific effects.\n"
                         "Example: --invert Constant Spring")
    ap.add_argument("--ffb-max-nm", type=float, default=2.0,
                    help="Mapping: 10000 USB units equals this many Nm (default: 2.0).")

    # ODrive control / test
    ap.add_argument("--gain", type=float, default=1.0, help="Global gain applied after FFB mix (default: 1.0).")
    ap.add_argument("--test-sine-amp", type=float, default=0.0, help="Test sine amplitude in Nm (0 to disable).")
    ap.add_argument("--test-sine-hz", type=float, default=0.0, help="Test sine frequency in Hz (0 to disable).")
    ap.add_argument("--debug-ffb", action="store_true", help="Print detailed FFB state changes.")
    ap.add_argument("--debug-force", action="store_true", help="Print ODrive telemetry periodically.")
    ap.add_argument("--min-current", type=float, default=8.0, help="Ensure motor.config.current_lim >= this (A).")
    ap.add_argument("--mode", choices=["auto", "torque", "current"], default="auto",
                    help="Force path: auto (default), torque, or current.")

    # Loop rates
    ap.add_argument("--force-hz", type=float, default=800.0,
                    help="Force/torque write rate in Hz (default: 800).")
    ap.add_argument("--pos-hz", type=float, default=400.0,
                    help="Position read + vJoy update rate in Hz (default: 400).")

    # Angle / center options
    ap.add_argument("--lock-to-lock-deg", type=float, default=900.0,
                    help="Your wheel's lock-to-lock rotation in degrees (default: 900).")
    ap.add_argument("--print-angle", action="store_true",
                    help="Print current steering angle (degrees) periodically.")
    ap.add_argument("--recenter-key", type=str, default="c",
                    help="Hotkey to recenter (center = current position). Default: 'c'")
    ap.add_argument("--zero-center-key", type=str, default="z",
                    help="Hotkey to clear user center offset (back to zero). Default: 'z'")
    ap.add_argument("--no-hotkeys", action="store_true",
                    help="Disable keyboard hotkeys for recenter/clear.")

    args = ap.parse_args()

    # --- Parse CLI arguments ---
    multipliers = parse_multipliers(args.strength)
    inversions = set(args.invert) if args.invert else set()

    print("--- FFB Configuration ---")
    print(f"Multipliers: {multipliers or 'Default'}")
    print(f"Inversions: {inversions or 'None'}")
    print(f"USB 10k -> {args.ffb_max_nm:.1f} Nm")
    print(f"Gain: {args.gain}")
    if args.test_sine_amp > 0 and args.test_sine_hz > 0:
        print(f"Test Sine: {args.test_sine_amp} @ {args.test_sine_hz} Hz (enabled)")
    else:
        print("Test Sine: disabled")
    print(f"Mode: {args.mode}")
    print(f"Force loop: {args.force_hz:.0f} Hz   Position loop: {args.pos_hz:.0f} Hz")
    print(f"Lock-to-lock: {args.lock_to_lock_deg:.0f}°")
    if not args.no_hotkeys:
        print(f"Hotkeys: Recenter='{args.recenter_key.upper()}', Clear='{args.zero_center_key.upper()}'")
    else:
        print("Hotkeys: disabled")
    if args.print_angle:
        print("Angle print: enabled")
    print("-------------------------")

    # --- Setup Components ---
    try:
        monitor = VJoyFfbMonitor(
            debug=args.debug_ffb,
            effect_multipliers=multipliers,
            effect_inversions=inversions,
            usb_max_nm=args.ffb_max_nm,   # alias supported by our monitor
        )
        monitor.start()
        print("[Main] FFB callback registered. Start a game to send FFB commands.")
    except (RuntimeError, FileNotFoundError) as e:
        print(f"ERROR: Could not start FFB monitor: {e}", file=sys.stderr)
        sys.exit(1)

    controller = ODriveController(
        ffb_monitor=monitor,
        odrive_axis=args.odrive_axis,
        vjoy_device_id=args.vjoy_id,
        ffb_gain=args.gain,
        test_sine_amp=args.test_sine_amp,
        test_sine_hz=args.test_sine_hz,
        debug=args.debug_force,
        min_current_limit=args.min_current,
        mode_override=args.mode,
        force_hz=args.force_hz,
        pos_hz=args.pos_hz,
        enable_hotkeys=(not args.no_hotkeys),
        recenter_key=args.recenter_key,
        zero_center_key=args.zero_center_key,
        lock_to_lock_deg=args.lock_to_lock_deg,
        print_angle=args.print_angle,
    )

    # --- Run the main async loop ---
    try:
        await controller.run()
    except KeyboardInterrupt:
        print("\n[Main] Shutdown signal received.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Main] Exiting.")
