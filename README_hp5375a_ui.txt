HP 5375A clickable keyboard UI
==============================

Files
-----
hp5375a_keyboard_ui.py     Tkinter interface and key hitboxes
hp5375a_front_panel.png    stitched full-panel background image

Requirements
------------
Python 3, Tkinter, Pillow. To use the serial connection, also install pyserial.

    python -m pip install pillow pyserial

Run without an Arduino connection:

    python hp5375a_keyboard_ui.py

Run with an Arduino serial port:

    python hp5375a_keyboard_ui.py --port COM5 --baud 115200
    python hp5375a_keyboard_ui.py --port /dev/ttyACM0 --baud 115200

The window has a “Show hitboxes” checkbox for visualizing and tuning clickable
areas. Hitbox coordinates are stored in the KEYS list in source-image pixels.

Serial output
-------------
Each click sends one UTF-8 JSON object followed by a newline. For example:

    {"key":"digit_7","label":"7 / K","kind":"digit","load_program_octal":"15","load_program_edcba":"HLLHL","kd_dcba":"LHHH","exponent_dcba":"LLHH","exponent_label":"K"}

The packet includes semantic key IDs and the corresponding table codes. The
Arduino firmware must parse this JSON-line protocol, or serial_packet() and
the write in on_click() must be changed to match an existing firmware parser.

The operation and digit code references come from Table 5-4 in the HP 5375A
Operating and Service Manual. The image is a reconstructed composite from
the attached instrument photos, so check the legends before finalizing a
hardware key mapping.
