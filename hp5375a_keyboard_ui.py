#!/usr/bin/env python3
"""Clickable HP 5375A keyboard panel with optional Arduino serial output.

Install Pillow and, if using a serial connection, pyserial:
    python -m pip install pillow pyserial

Run without serial (clicks are shown in the event log):
    python hp5375a_keyboard_ui.py

Run with a serial port:
    python hp5375a_keyboard_ui.py --port COM5 --baud 115200
    python hp5375a_keyboard_ui.py --port /dev/ttyACM0 --baud 115200

Serial protocol: one JSON value per key press, followed by a newline. The
current serial_packet() sends the key ID as a JSON string; the firmware also
accepts the object format containing a "key" field.
"""

from __future__ import annotations

import argparse
import json
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk
from typing import Any

from PIL import Image, ImageTk

try:
    import serial  # type: ignore[import-not-found]
except ImportError:
    serial = None


BASE_DIR = Path(__file__).resolve().parent
PANEL_IMAGE = BASE_DIR / "hp5375a_front_panel.png"
IMAGE_SIZE = (1672, 941)


@dataclass(frozen=True)
class Key:
    key_id: str
    label: str
    box: tuple[int, int, int, int]
    data: dict[str, Any]


def operation(key_id: str, label: str, box: tuple[int, int, int, int],
              octal: str, edcba: str) -> Key:
    return Key(key_id, label, box, {
        "kind": "operation",
        "keyboard_key_code_octal": octal,
        "keyboard_key_code_edcba": edcba,
    })


def special(key_id: str, label: str, box: tuple[int, int, int, int],
            **data: Any) -> Key:
    return Key(key_id, label, box, data)


# Hitboxes are in original composite-image pixels (1672 x 941). Turn on
# “Show hitboxes” in the UI to inspect or adjust them.
KEYS = [
    # Upper black strip: pushbuttons only. Rotary and slide controls are not
    # mapped to clicks because they are not keys.
    special("start", "START", (1082, 135, 1167, 220), kind="panel_control"),
    special("run", "RUN", (1178, 135, 1264, 220), kind="panel_control"),
    special("step", "STEP", (1276, 135, 1361, 220), kind="panel_control"),
    special("learn", "LEARN", (1372, 135, 1458, 220), kind="panel_control"),
    special("manual", "MANUAL", (1467, 135, 1547, 220), kind="panel_control"),

    # Left-hand conditional and program keys.
    operation("if_x_gt_a", "IF x > a", (127, 357, 226, 558), "14", "HLLHH"),
    operation("if_x_lt_b", "IF x < b", (236, 357, 339, 558), "15", "HLLHL"),
    operation("pause", "PAUSE", (126, 563, 222, 666), "01", "HHHHL"),
    operation("skip_step", "SKIP STEP", (237, 563, 339, 666), "00", "HHHHH"),
    operation("repeat", "REPEAT", (126, 670, 222, 777), "24", "LHLHH"),
    operation("xfer_prog", "XFER PROG", (237, 670, 339, 777), "04", "HHLHH"),
    operation("display_x", "DISPLAY x", (119, 780, 352, 883), "10", "HLHHH"),

    # A/B/C/Z and X/Y register-transfer column.
    operation("a_swap_x", "a ↔ x", (457, 360, 565, 460), "21", "LHHHL"),
    operation("a_copy_xy", "a ↓ x,y ", (566, 360, 675, 460), "23", "LHHLL"),
    operation("b_swap_x", "b ↔ x", (457, 462, 565, 562), "31", "LLHHL"),
    operation("b_copy_xy", "b ↓ xy", (566, 462, 675, 562), "33", "LLHLL"),
    operation("c_swap_x", "c ↔ x", (457, 565, 565, 668), "30", "LLHHH"),
    operation("c_copy_xy", "c ↓ xy", (566, 565, 675, 668), "32", "LLHLH"),
    operation("z_swap_x", "z ↔ x", (457, 671, 565, 775), "35", "LLLHL"),
    operation("z_copy_xy", "z ↓ xy", (566, 671, 675, 775), "37", "LLLLL"),
    operation("y_swap_x", "y ↔ x", (457, 779, 565, 884), "25", "LHLHL"),
    operation("x_copy_y", "x ↓ y ", (566, 779, 675, 884), "27", "LHLLL"),

    # Arithmetic, register, module and plug-in keys.
    operation("clear_x", "CLEAR x", (781, 360, 890, 461), "26", "LHLLH"),
    operation("clear_xyz", "CLEAR x y z", (892, 360, 1002, 461), "36", "LLLLH"),
    operation("sqrt_x", "√x", (1005, 360, 1116, 461), "05", "HHLHL"),
    operation("one_shift_xy", "1 ↓ xy ", (781, 462, 890, 563), "20", "LHHHH"),
    operation("n_shift_xy", "N ↓ xy ", (892, 462, 1002, 563), "22", "LHHLH"),
    operation("divide", "÷", (1005, 462, 1116, 563), "16", "HLLLH"),
    operation("ten_x", "10x", (781, 565, 890, 667), "11", "HLHHL"),
    operation("x_over_ten", "x/10", (892, 565, 1002, 667), "34", "LLLHH"),
    operation("multiply", "x", (1005, 565, 1116, 667), "17", "HLLLL"),
    operation("module_a", "MODULE A", (781, 670, 890, 775), "02", "HHHLH"),
    operation("module_b", "MODULE B", (781, 779, 890, 884), "06", "HHLLH"),
    operation("plugin", "PLUG-IN", (892, 670, 1002, 884), "03", "HHHLL"),
    operation("subtract", "-", (1005, 670, 1116, 775), "13", "HLHLL"),
    operation("add", "+", (1005, 779, 1116, 884), "12", "HLHLH"),

    # Numeric keypad. KD is the manual's active-level pattern in D,C,B,A order.
    special("enter_minus", "ENTER MINUS", (1214, 360, 1333, 459),
            kind="load_control", load_program_octal="15",
            load_program_edcba="HLLHL", kd_dcba="LLLL", negative=True),
    special("enter_prefix", "ENTER PREFIX", (1333, 360, 1443, 459),
            kind="exponent_prefix", exponent_dcba="LLLH"),
    special("clear_entry", "CLEAR ENTRY", (1443, 360, 1546, 459),
            kind="mainframe_operation", mainframe_code_octal="25",
            mainframe_code_edcba="LHLHL"),
]

NUMBERS = [
    # (label, id, box, digit KD code, exponent code, key legend exponent)
    ("7 / K", "digit_7", (1214, 461, 1333, 563), "LHHH", "LLHH", "K"),
    ("8 / M", "digit_8", (1333, 461, 1443, 563), "HLLL", "LHHL", "M"),
    ("9 / G", "digit_9", (1443, 461, 1546, 563), "HLLH", "HLLH", "G"),
    ("4 / m", "digit_4", (1214, 565, 1333, 669), "LHLL", "LLHH", "m"),
    ("5 / µ", "digit_5", (1333, 565, 1443, 669), "LHLH", "LHHL", "µ"),
    ("6 / n", "digit_6", (1443, 565, 1546, 669), "LHHL", "HLLH", "n"),
    ("1 / p", "digit_1", (1214, 671, 1333, 773), "LLLH", "HHLL", "p"),
    ("2 / f", "digit_2", (1333, 671, 1443, 773), "LLHL", "HHHH", "f"),
    ("3 / T", "digit_3", (1443, 671, 1550, 773), "LLHH", "HHLL", "T"),
    ("0", "digit_0", (1214, 777, 1452, 883), "LLLL", "LLLL", "0"),
    ("decimal point", "decimal_point", (1457, 777, 1557, 883), "LLLL", "HHHH", "."),
]
for label, key_id, box, kd, exponent, exponent_label in NUMBERS:
    KEYS.append(special(
        key_id, label, box,
        kind="digit",
        load_program_octal="15",  # LOAD
        load_program_edcba="HLLHL",
        kd_dcba=kd,
        exponent_dcba=exponent,
        exponent_label=exponent_label,
    ))


def serial_packet(key: Key) -> str:
    """Return the key ID that on_click() encodes as a JSON string."""
    # return {"key": key.key_id, "label": key.label, **key.data}
    return f"{key.key_id}"


class KeyboardUI:
    def __init__(self, root: tk.Tk, port: str | None, baud: int) -> None:
        self.root = root
        self.root.title("HP 5375A Keyboard Emulator")
        self.root.minsize(900, 540)
        self.serial_port = None
        self.port_arg = port
        self.baud = baud
        self.show_boxes = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value="Disconnected — clicks are logged locally")
        self.current_scale = 1.0
        self.photo: ImageTk.PhotoImage | None = None
        self.panel: Image.Image | None = None
        self.image_item: int | None = None
        self.visible_hitboxes: dict[str, int] = {}
        self.hover_id: str | None = None

        if not PANEL_IMAGE.is_file():
            raise FileNotFoundError(f"Panel image not found: {PANEL_IMAGE}")
        self.panel = Image.open(PANEL_IMAGE).convert("RGB")

        toolbar = ttk.Frame(root, padding=(8, 8, 8, 4))
        toolbar.pack(fill="x")
        ttk.Checkbutton(toolbar, text="Show hitboxes", variable=self.show_boxes,
                        command=self.draw_hitboxes).pack(side="left")
        ttk.Label(toolbar, text="Arduino port:").pack(side="left", padx=(18, 4))
        self.port_entry = ttk.Entry(toolbar, width=18)
        self.port_entry.insert(0, port or "")
        self.port_entry.pack(side="left")
        ttk.Button(toolbar, text="Connect", command=self.connect_serial).pack(side="left", padx=5)
        ttk.Button(toolbar, text="Disconnect", command=self.disconnect_serial).pack(side="left")
        ttk.Label(toolbar, textvariable=self.status).pack(side="left", padx=(14, 0))

        self.canvas = tk.Canvas(root, background="#242424", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(4, 6))
        self.canvas.bind("<Configure>", self.redraw)
        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<Leave>", self.on_leave)
        self.canvas.bind("<Button-1>", self.on_click)

        footer = ttk.Frame(root, padding=(8, 0, 8, 8))
        footer.pack(fill="x")
        self.last_event = tk.StringVar(value="Click a key to see its event here.")
        ttk.Label(footer, textvariable=self.last_event).pack(side="left")

        if self.port_arg:
            self.connect_serial()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def redraw(self, _event: tk.Event | None = None) -> None:
        if self.panel is None:
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        scale = min(width / self.panel.width, height / self.panel.height)
        draw_w = max(1, round(self.panel.width * scale))
        draw_h = max(1, round(self.panel.height * scale))
        resized = self.panel.resize((draw_w, draw_h), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(resized)
        self.current_scale = scale
        self.canvas.delete("all")
        x = (width - draw_w) // 2
        y = (height - draw_h) // 2
        self.image_item = self.canvas.create_image(x, y, anchor="nw", image=self.photo)
        self.image_origin = (x, y)
        self.draw_hitboxes()

    def draw_hitboxes(self) -> None:
        if not hasattr(self, "image_origin"):
            return
        self.canvas.delete("hotspot")
        self.visible_hitboxes.clear()
        ox, oy = self.image_origin
        for key in KEYS:
            x1, y1, x2, y2 = key.box
            coords = (ox + x1 * self.current_scale, oy + y1 * self.current_scale,
                      ox + x2 * self.current_scale, oy + y2 * self.current_scale)
            visible = self.show_boxes.get() or key.key_id == self.hover_id
            if visible:
                rect = self.canvas.create_rectangle(
                    *coords,
                    fill="#1687ff" if key.key_id != self.hover_id else "#ffae00",
                    stipple="gray50",
                    outline="#f8f8f8",
                    width=2,
                    tags=("hotspot", key.key_id),
                )
                self.visible_hitboxes[key.key_id] = rect

    def key_at(self, x: float, y: float) -> Key | None:
        ox, oy = self.image_origin
        px = (x - ox) / self.current_scale
        py = (y - oy) / self.current_scale
        for key in reversed(KEYS):
            x1, y1, x2, y2 = key.box
            if x1 <= px <= x2 and y1 <= py <= y2:
                return key
        return None

    def on_motion(self, event: tk.Event) -> None:
        key = self.key_at(event.x, event.y)
        new_id = key.key_id if key else None
        if new_id != self.hover_id:
            self.hover_id = new_id
            self.draw_hitboxes()
            if key:
                self.root.configure(cursor="hand2")
            else:
                self.root.configure(cursor="")

    def on_leave(self, _event: tk.Event) -> None:
        self.hover_id = None
        self.root.configure(cursor="")
        self.draw_hitboxes()

    def on_click(self, event: tk.Event) -> None:
        key = self.key_at(event.x, event.y)
        if key is None:
            return
        packet = serial_packet(key)
        payload = json.dumps(packet, ensure_ascii=False, separators=(",", ":"))
        self.last_event.set(payload)
        if self.serial_port is not None:
            try:
                self.serial_port.write((key.key_id + "\n").encode("utf-8"))
                print((key.key_id + "\n"))
                self.status.set(f"Sent {key.label} to {self.serial_port.port}")
            except Exception as exc:
                self.status.set(f"Serial write failed: {exc}")

    def connect_serial(self) -> None:
        port = self.port_entry.get().strip()
        if not port:
            self.status.set("Enter a serial port, or leave disconnected for local clicks")
            return
        if serial is None:
            self.status.set("pyserial is missing; install it with: python -m pip install pyserial")
            return
        self.disconnect_serial()
        try:
            self.serial_port = serial.Serial(port, self.baud, timeout=0.1)
            self.status.set(f"Connected to {port} at {self.baud} baud")
        except Exception as exc:
            self.serial_port = None
            self.status.set(f"Could not open {port}: {exc}")

    def disconnect_serial(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None
        self.status.set("Disconnected — clicks are logged locally")

    def close(self) -> None:
        self.disconnect_serial()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Arduino serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (default: 115200)")
    args = parser.parse_args()
    root = tk.Tk()
    KeyboardUI(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
