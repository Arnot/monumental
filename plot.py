#!/usr/bin/env python3

"""
XY driver for a pen plotter with:
  - L<n>: linear motor steps (n can be negative)
  - R<n>: rotary motor steps (n can be negative)

Requires:
  - calibration.json from the calibration script
  - Known physical width/height of the drawing area in mm

This file gives you:
  - A mapping (x_mm, y_mm) -> (L_steps, R_steps) via bilinear interpolation
  - goto_xy(x, y) and draw_line(x0, y0, x1, y1) using small step segments
"""

import json
import re
import sys
from pathlib import Path
from typing import Union
from time import sleep

import serial


SERIAL_PORT = "/dev/cu.usbmodem1101"
BAUDRATE = 9600
TIMEOUT = 1.0  # seconds
CALIBRATION_FILE = "calibration.json"

MOVE_RE = re.compile(r"^\s*([LR])\s*([+-]?\d+)\s*$", re.IGNORECASE)
INT_RE = re.compile(r"[-+]?\d+")
POINT_RE = re.compile(r"(\d+) (\d+)")
POLY_RE = re.compile(r"p (\d+) (\d+)")

# Physical size of the drawable area (in mm)
WIDTH_MM = 200.0   # left to right
HEIGHT_MM = 200.0  # top to bottom

# Default number of segments to approximate a line
DEFAULT_SEGMENTS = 100

# ============================================================================


def send_command(ser: serial.Serial, cmd: str) -> None:
    """Send a single command to the plotter."""
    line = cmd.strip() + "\n"
    print(f"sending {cmd}")
    ser.write(line.encode("ascii", errors="ignore"))

def read_linear_position(ser: serial.Serial) -> int:
    ser.reset_input_buffer()
    send_command(ser, "p");

    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        raise RuntimeError("No response to 'p' command")

    print(line)
    m = INT_RE.search(line)
    if not m:
        raise RuntimeError("Could not find integer in 'p' response: %r" % line)

    return int(m.group(0))


class CalibrationMapping:
    """
    Maps (x_mm, y_mm) in a rectangular drawing area to (L_steps, R_steps)
    using bilinear interpolation between 4 corner measurements.

    Corners in JSON (step space):
      - top_left     (TL)
      - top_right    (TR)
      - bottom_left  (BL)
      - bottom_right (BR)

    XY coordinates:
      - origin (0,0) at top-left
      - x increases to the right
      - y increases downward
      - x in [0, WIDTH_MM], y in [0, HEIGHT_MM]
    """

    def __init__(self, corners: dict, width_mm: float, height_mm: float):
        self.width_mm = float(width_mm)
        self.height_mm = float(height_mm)

        # Extract L/R for convenience
        self.TL = (
            corners["top_left"]["L_steps"],
            corners["top_left"]["R_steps"],
        )
        self.TR = (
            corners["top_right"]["L_steps"],
            corners["top_right"]["R_steps"],
        )
        self.BL = (
            corners["bottom_left"]["L_steps"],
            corners["bottom_left"]["R_steps"],
        )
        self.BR = (
            corners["bottom_right"]["L_steps"],
            corners["bottom_right"]["R_steps"],
        )

    @classmethod
    def from_file(cls, path: Union[str, Path], width_mm: float, height_mm: float):
        with open(path, "r", encoding="utf-8") as f:
            corners = json.load(f)
        return cls(corners, width_mm, height_mm)

    def xy_to_steps(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """
        Convert XY in mm to (L_steps, R_steps).

        Uses bilinear interpolation between TL, TR, BL, BR.
        """

        # Normalize to [0,1] range
        u = x_mm / self.width_mm
        v = y_mm / self.height_mm

        # Clamp inside the rectangle just in case
        u = max(0.0, min(1.0, u))
        v = max(0.0, min(1.0, v))

        # Convenience vars
        L_TL, R_TL = self.TL
        L_TR, R_TR = self.TR
        L_BL, R_BL = self.BL
        L_BR, R_BR = self.BR

        # Bilinear interpolation
        # f(u, v) = TL*(1-u)*(1-v) + TR*u*(1-v) + BL*(1-u)*v + BR*u*v

        def bilinear(a_TL, a_TR, a_BL, a_BR):
            return (
                a_TL * (1 - u) * (1 - v)
                + a_TR * u * (1 - v)
                + a_BL * (1 - u) * v
                + a_BR * u * v
            )

        L = bilinear(L_TL, L_TR, L_BL, L_BR)
        R = bilinear(R_TL, R_TR, R_BL, R_BR)

        # Round to closest integer number of steps
        return int(round(L)), int(round(R))


class Plotter:
    """
    High-level XY driver for the plotter.

    Keeps track of:
      - current XY position (mm, logical)
      - current L/R step position (as we last commanded)
    """

    def __init__(
        self,
        ser: serial.Serial,
        mapping: CalibrationMapping,
        start_x_mm: float = 0.0,
        start_y_mm: float = 0.0,
    ):
        self.ser = ser
        self.mapping = mapping

        # Logical starting position in mm (assume we start at top-left)
        self.current_x = start_x_mm
        self.current_y = start_y_mm

        # Corresponding motor steps
        self.current_L, self.current_R = mapping.xy_to_steps(
            self.current_x, self.current_y
        )

        # Move the hardware linear axis to match our logical start (absolute)
        send_command(self.ser, f"l{self.current_L}")


    def move_LR_steps(self, target_L: int, target_R: int):
        """
        Move from current (L,R) to (target_L, target_R).

        This version just:
          - sends one L<delta>
          - then one R<delta>
        You can replace this with a Bresenham-style interleaving if needed.
        """
        dL = target_L - self.current_L
        dR = target_R - self.current_R

        # Linear: absolute
        if target_L != self.current_L:
            send_command(self.ser, f"l{target_L}")
            self.current_L = target_L

        # Rotation: relative
        dR = target_R - self.current_R
        if dR != 0:
            send_command(self.ser, f"R{dR}")
            self.current_R += dR




    def goto_xy(self, x_mm: float, y_mm: float, segments: int = DEFAULT_SEGMENTS, wait: bool = True):
        """
        Move in a line (in XY) from current (x,y) to (x_mm, y_mm),
        approximated by 'segments' small moves.

        Each small move:
          - computes intermediate (x,y) on the straight line
          - maps that to (L,R)
          - does sequential L/R movement.
        """
        x0, y0 = self.current_x, self.current_y
        dx = x_mm - x0
        dy = y_mm - y0

        if segments < 1:
            segments = 1

        print(f"goto_xy {x_mm} {y_mm} in {segments} steps")

        for i in range(1, segments + 1):
            t = i / segments
            xi = x0 + dx * t
            yi = y0 + dy * t

            target_L, target_R = self.mapping.xy_to_steps(xi, yi)
            self.move_LR_steps(target_L, target_R)

        # Update logical position
        self.current_x = x_mm
        from typing import Union
        self.current_y = y_mm
        if wait:
            input(f"Pen moved to {x_mm}, {y_mm}. Press Enter to continue...")
        else:
            # give the plotter some time to draw without spamming it
            sleep(6)

    def draw_line(
        self,
        x0_mm: float,
        y0_mm: float,
        x1_mm: float,
        y1_mm: float,
        segments: int = DEFAULT_SEGMENTS,
    ):
        # Move (in small steps) from current to start of line
        self.goto_xy(x0_mm, y0_mm, segments=segments)
        # Draw the line
        self.goto_xy(x1_mm, y1_mm, segments=segments)

def manual_home(ser):
    print("\n=== MANUAL HOMING ===")
    print("Jog the plotter to the physical HOME position (e.g. top-left).")
    print("Commands:")
    print("  L<n> or R<n>   = Move axes")
    print("  show           = Display current raw step offsets")
    print("  ok             = Confirm current position as HOME")
    print("  quit           = Abort")
    print()

    actual_L = read_linear_position(ser);

    current_L = 0
    current_R = 0

    while True:
        cmd = input("[home] Enter command: ").strip()

        if cmd.lower() == "quit":
            print("Homing aborted.")
            sys.exit(0)

        if cmd.lower() == "show":
            print(f"Current software counters: L={current_L}, R={current_R}")
            continue

        if cmd.lower() == "ok":
            print("Homing confirmed at current position.")
            return current_L, current_R

        m = MOVE_RE.match(cmd)
        if m:
            axis, val = m.group(1).upper(), int(m.group(2))
            send_command(ser, f"{axis}{val}")
            if axis == "L":
                current_L += val
            else:
                current_R += val
            continue

        print("Invalid command. Use L<n>, R<n>, ok, show, or quit.")

import math

def draw_polygon(plotter, sides, center_x, center_y, radius_mm):
    if sides < 3:
        raise ValueError("Polygon must have at least 3 sides.")

    points = []

    for i in range(sides):
        angle = 2 * math.pi * i / sides
        x = int(center_x + radius_mm * math.cos(angle))
        y = int(center_y + radius_mm * math.sin(angle))
        points.append((x, y))

    # Move to first point
    x0, y0 = points[0]
    plotter.goto_xy(x0, y0)

    for (x, y) in points[1:]:
        plotter.goto_xy(x, y, wait = False)

    plotter.goto_xy(x0, y0, wait = False)


def main():
    # Open serial
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    with ser:
        print(f"Connected to {SERIAL_PORT} at {BAUDRATE} baud.")

        mapping = CalibrationMapping.from_file(
            CALIBRATION_FILE,
            width_mm=WIDTH_MM,
            height_mm=HEIGHT_MM,
        )

        manual_home(ser)

        expected_L, expected_R = mapping.xy_to_steps(0.0, 0.0)

        # Actual linear position reported by firmware:
        try:
            actual_L = read_linear_position(ser)
            print("Firmware linear position (p): %d" % actual_L)
        except Exception as e:
            print("Warning: could not read linear position, assuming no offset: %s" % e)
            actual_L = expected_L

        L_offset = actual_L - expected_L
        print("Applying linear offset of %d steps to mapping." % L_offset)

        # Wrap xy_to_steps to add this offset on L
        original_xy_to_steps = mapping.xy_to_steps

        def xy_to_steps_with_offset(x_mm, y_mm):
            L, R = original_xy_to_steps(x_mm, y_mm)
            return L + L_offset, R

        mapping.xy_to_steps = xy_to_steps_with_offset

        # Assume we start at top-left (0,0) in XY, with mapping now aligned
        plotter = Plotter(ser, mapping, start_x_mm=0.0, start_y_mm=0.0)

        # Example: draw a rectangle border of the full page
        w = WIDTH_MM
        h = HEIGHT_MM

        # print("Drawing page border...")
        # plotter.goto_xy(0, 0)
        # plotter.goto_xy(w, 0)
        # plotter.goto_xy(w, h)
        # plotter.goto_xy(0, h)
        # plotter.goto_xy(0, 0)

        # # Example: draw a diagonal
        # print("Drawing diagonal...")
        # plotter.goto_xy(0, 0)
        # plotter.goto_xy(w, h)


        while True:
            cmd = input("""
            - 'x y' to move to that point
            - a single number to draw a centered rectangle with that side length
            - p <sides> <radius> for a polygon with <sides> and <radius>
            """).strip()

            if not cmd:
                continue

            m = POINT_RE.match(cmd)
            if m:
                x = int(m.group(1))
                y = int(m.group(2))
                plotter.goto_xy(x, y)
                continue

            m = INT_RE.match(cmd)
            if m:
                # Centered square
                side = int(m.group(0))
                padding_x = (WIDTH_MM - side) / 2
                padding_y = (HEIGHT_MM - side) / 2
                plotter.goto_xy(padding_x, padding_y)
                plotter.goto_xy(w - padding_x, padding_y, wait = False)
                plotter.goto_xy(w - padding_x, h + padding_y, wait = False)
                plotter.goto_xy(padding_x, h - padding_y, wait = False)
                plotter.goto_xy(padding_x, padding_y, wait = False)
                continue

            m = POLY_RE.match(cmd)
            if m:
                sides = int(m.group(1))
                radius = int(m.group(2))
                draw_polygon(plotter, sides, int(WIDTH_MM / 2), int(HEIGHT_MM / 2), radius)
                continue

if __name__ == "__main__":
    main()
