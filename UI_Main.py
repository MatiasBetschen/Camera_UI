import serial
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import io

# -------------------------
# CONFIG
# -------------------------
SERIAL_PORT = "COM11"     # CHANGE THIS
BAUD_RATE = 115200       # MUST MATCH ARDUINO
PREVIEW_SIZE = (320, 240)

# -------------------------
# RESOLUTION COMMANDS
# -------------------------
RESOLUTIONS = {
    "160 x 120": 0x00,
    "176 x 144": 0x01,
    "320 x 240": 0x02,
    "352 x 288": 0x03,
    "640 x 480": 0x04,
    "800 x 600": 0x05,
    "1024 x 768": 0x06,
    "1280 x 1024": 0x07,
    "1600 x 1200": 0x08
}

# -------------------------
# SERIAL
# -------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # allow SAMD reset

# -------------------------
# FUNCTIONS
# -------------------------
def set_resolution(event=None):
    res = resolution_combo.get()
    cmd = RESOLUTIONS[res]
    ser.write(bytes([cmd]))
    status_label.config(text=f"Resolution set to {res}")

def capture_image():
    status_label.config(text="Capturing...")
    root.update()

    ser.write(bytes([0x10]))  # capture command

    jpg = bytearray()
    prev = None
    in_image = False

    while True:
        b = ser.read(1)
        if not b:
            continue

        byte = b[0]

        if prev == 0xFF and byte == 0xD8:
            in_image = True
            jpg.extend([0xFF, 0xD8])

        elif in_image:
            jpg.append(byte)
            if prev == 0xFF and byte == 0xD9:
                break

        prev = byte

    if not jpg:
        status_label.config(text="Capture failed.")
        return

    img = Image.open(io.BytesIO(jpg))
    img.thumbnail(PREVIEW_SIZE)
    img_tk = ImageTk.PhotoImage(img)

    image_label.config(image=img_tk)
    image_label.image = img_tk

    status_label.config(text="Image captured.")

# -------------------------
# GUI
# -------------------------
root = tk.Tk()
root.title("ArduCAM Mini 2MP Viewer")

image_label = tk.Label(root)
image_label.pack(padx=10, pady=10)

control_frame = tk.Frame(root)
control_frame.pack(pady=5)

tk.Label(control_frame, text="Resolution:").grid(row=0, column=0, padx=5)

resolution_combo = ttk.Combobox(
    control_frame,
    values=list(RESOLUTIONS.keys()),
    state="readonly",
    width=15
)
resolution_combo.set("320 x 240")
resolution_combo.grid(row=0, column=1)
resolution_combo.bind("<<ComboboxSelected>>", set_resolution)

capture_btn = tk.Button(control_frame, text="Capture", command=capture_image)
capture_btn.grid(row=0, column=2, padx=10)

status_label = tk.Label(root, text="Ready.")
status_label.pack(pady=5)

root.mainloop()
