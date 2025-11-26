#!/usr/bin/env python3

import serial
import time
import re
from pathlib import Path

ARDUINO_PORT = "/dev/cu.usbmodem1101";
ARDUINO_BAUD_RATE = 9600;
CALIBRATION_FILE = "calibration.json"

MOVE_RE = re.compile(r"^\s*([LR])\s*([+-]?\d+)\s*$", re.IGNORECASE)

arduino = serial.Serial(port=ARDUINO_PORT, baudrate=9600, timeout=.1)

def send_command(ser: serial.Serial, cmd: str) -> None:
    line = cmd.strip() + "\n"
    ser.write(line.encode("ascii", errors="ignore"))


def prompt_for_corner(
    corner_name: str,
    current_L: int,
    current_R: int,
):

    print()
    print("=" * 60)
    print(f"Calibrating corner: {corner_name}")
    print("Commands:")
    print("  - L<n> : move linear axis by n steps (e.g. L100, L-20)")
    print("  - R<n> : rotate by n steps (e.g. R50, R-10)")
    print("  - reset: reset step counters to (0, 0) (NO actual movement)")
    print("  - ok   : when pen is exactly at this corner, record and continue")
    print("  - show : print current (L_steps, R_steps)")
    print("  - quit : abort calibration")
    print()

    while True:
        cmd = input(f"[{corner_name}] enter command (L/R/ok/reset/show/quit): ").strip()

        if not cmd:
            continue

        if cmd.lower() == "quit":
            print("Aborting calibration.")
            sys.exit(0)

        if cmd.lower() == "show":
            print(f"Current position: L={current_L} steps, R={current_R} steps")
            continue

        if cmd.lower() == "reset":
            current_L = 0
            current_R = 0
            print("Step counters reset to L=0, R=0 (NOTE: no motor movement sent).")
            continue

        if cmd.lower() == "ok":
            print(
                f"Recording {corner_name}: "
                f"L={current_L} steps, R={current_R} steps"
            )
            return (current_L, current_R), (current_L, current_R)

        # Otherwise, expect a movement command like L123 or R-45
        m = MOVE_RE.match(cmd)
        if not m:
            print("Invalid command format. Use L<n>, R<n>, or control words.")
            continue

        axis = m.group(1).upper()
        delta = int(m.group(2))

        # Send to device
        send_command(arduino, f"{axis}{delta}")

        # Update our software position
        if axis == "L":
            current_L += delta
        else:
            current_R += delta

        print(f"Moved {axis} by {delta} steps. New pos: L={current_L}, R={current_R}")


def main():
    with arduino:
        # (Software) home position
        current_L = 0
        current_R = 0

        corners = {}

        # Order: TL, TR, BL, BR
        order = [
            ("top_left", "Top-left"),
            ("top_right", "Top-right"),
            ("bottom_left", "Bottom-left"),
            ("bottom_right", "Bottom-right"),
        ]

        for key, label in order:
            (current_L, current_R), pos = prompt_for_corner(
                label, current_L, current_R
            )
            corners[key] = {"L_steps": pos[0], "R_steps": pos[1]}

        print()
        print("Calibration complete.")
        print("Recorded corners (in step space):")
        for key, label in order:
            pos = corners[key]
            print(
                f"  {label:12s}: "
                f"L={pos['L_steps']:7d}  R={pos['R_steps']:7d}"
            )

        # Save to JSON
        path = Path(CALIBRATION_FILE)
        with path.open("w", encoding="utf-8") as f:
            json.dump(corners, f, indent=2)

        print()
        print(f"Calibration saved to {path.resolve()}")



if __name__ == "__main__":
    main()
