import serial
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import io

# -------------------------
# CONFIG
# -------------------------
SERIAL_PORT = "COM16"   # CHANGE
BAUD_RATE = 115200

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

EXPOSURE_VALUES_MS = [5, 10, 20, 50, 100, 200, 500, 1000]
  # Example values
GAIN_VALUES = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80]      # Example values

# -------------------------
# SERIAL
# -------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# -------------------------
# GUI SETUP
# -------------------------
root = tk.Tk()
root.title("ArduCAM Mini 2MP Viewer")

# Start maximized (normal window, not kiosk)
try:
    root.state("zoomed")     # Windows
except:
    root.attributes("-zoomed", True)  # Linux fallback

# -------------------------
# MAIN LAYOUT
# -------------------------
root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=0)  # left panel
root.grid_columnconfigure(1, weight=1)  # image area

# -------------------------
# LEFT DATA PANEL
# -------------------------
data_panel = tk.Frame(root, bg="#1e1e1e", width=350)
data_panel.grid(row=0, column=0, sticky="ns")
data_panel.grid_propagate(False)

# Title
tk.Label(
    data_panel,
    text="Live Data",
    fg="white",
    bg="#1e1e1e",
    font=("Segoe UI", 14, "bold")
).pack(pady=(15, 20))

# Object temperature
tk.Label(
    data_panel,
    text="Object Temperature:",
    fg="white",
    bg="#1e1e1e",
    font=("Segoe UI", 11)
).pack(anchor="w", padx=15)

object_temp_var = tk.StringVar(value="--.- °C")

object_temp_label = tk.Label(
    data_panel,
    textvariable=object_temp_var,
    fg="#00ffcc",
    bg="#1e1e1e",
    font=("Segoe UI", 18, "bold")
)
object_temp_label.pack(anchor="w", padx=15, pady=(5, 20))
# -------------------------
# IMU DATA
# -------------------------
tk.Label(
    data_panel,
    text="IMU (Accel | Gyro):",
    fg="white",
    bg="#1e1e1e",
    font=("Segoe UI", 11)
).pack(anchor="w", padx=15, pady=(10, 0))

imu_var = tk.StringVar(value="--,--,-- | --,--,--")

imu_label = tk.Label(
    data_panel,
    textvariable=imu_var,
    fg="#ffaa00",
    bg="#1e1e1e",
    font=("Segoe UI", 11)
)
imu_label.pack(anchor="w", padx=15, pady=(5, 20))
# -------------------------
# VoltageDATA
# -------------------------
tk.Label(
    data_panel,
    text="Battery Voltage:",
    fg="white",
    bg="#1e1e1e",
    font=("Segoe UI", 11)
).pack(anchor="w", padx=15)
object_voltage_var = tk.StringVar(value="--.- V")
latest_voltage = tk.Label(
    data_panel,
    textvariable=object_voltage_var,
    fg="#ff0000",
    bg="#1e1e1e",
    font=("Segoe UI", 18, "bold")
)
latest_voltage.pack(anchor="w", padx=15, pady=(5, 20))

# -------------------------
# IMAGE AREA
# -------------------------

image_label = tk.Label(root, bg="black")
image_label.grid(row=0, column=1, sticky="nsew")

control_bar = tk.Frame(root, bg="#2b2b2b", height=50)
control_bar.grid(row=1, column=0, columnspan=2, sticky="ew")
control_bar.grid_propagate(False)

# -------------------------
# CONTROLS
# -------------------------
tk.Label(control_bar, text="Resolution:", fg="white", bg="#2b2b2b").pack(side="left", padx=10)

resolution_combo = ttk.Combobox(
    control_bar,
    values=list(RESOLUTIONS.keys()),
    state="readonly",
    width=15
)
resolution_combo.set("1600 x 1200")
resolution_combo.pack(side="left")
resolution_combo.bind(
    "<<ComboboxSelected>>",
    lambda e: ser.write(bytes([RESOLUTIONS[resolution_combo.get()]]))
)

capture_btn = tk.Button(
    control_bar,
    text="Capture",
    font=("Segoe UI", 11),
    command=lambda: capture_image()
)
capture_btn.pack(side="left", padx=20)

tk.Label(control_bar, text="Exposure:", fg="white", bg="#2b2b2b").pack(side="left", padx=10)

exposure_combo = ttk.Combobox(
    control_bar,
    values=[f"{v} ms" for v in EXPOSURE_VALUES_MS],
    state="readonly",
    width=10
)
exposure_combo.set("50 ms")
exposure_combo.pack(side="left")
exposure_combo.bind(
    "<<ComboboxSelected>>",
    lambda e: set_exposure_ms(
        exposure_combo.get().replace(" ms", "")
    )
)


tk.Label(control_bar, text="Gain:", fg="white", bg="#2b2b2b").pack(side="left", padx=10)

gain_combo = ttk.Combobox(
    control_bar,
    values=[f"0x{value:02X}" for value in GAIN_VALUES],
    state="readonly",
    width=10
)
gain_combo.set(f"0x{GAIN_VALUES[0]:02X}") 
gain_combo.pack(side="left")
gain_combo.bind(
    "<<ComboboxSelected>>",
    lambda e: set_gain(int(gain_combo.get(), 16))
)

status_label = tk.Label(control_bar, text="Ready", fg="white", bg="#2b2b2b")
status_label.pack(side="right", padx=10)

# -------------------------
# CAPTURE FUNCTION
# -------------------------
def capture_image():
    status_label.config(text="Capturing...")
    root.update_idletasks()

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)

    # Send selected resolution command
    ser.write(bytes([RESOLUTIONS[resolution_combo.get()]]))

    ser.write(bytes([0x10]))

    jpg = bytearray()
    prev = None
    in_image = False

    while True:
        b = ser.read(1)
        if not b:
            continue

        cur = b[0]

        if not in_image:
            if prev == 0xFF and cur == 0xD8:
                in_image = True
                jpg.extend([0xFF, 0xD8])
        else:
            jpg.append(cur)
            if prev == 0xFF and cur == 0xD9:
                break

        prev = cur

    if not jpg:
        status_label.config(text="Capture failed")
        return

    img = Image.open(io.BytesIO(jpg))

    # Resize to available area
    area_w = image_label.winfo_width()
    area_h = image_label.winfo_height()

    img_ratio = img.width / img.height
    area_ratio = area_w / area_h

    if img_ratio > area_ratio:
        new_w = area_w
        new_h = int(area_w / img_ratio)
    else:
        new_h = area_h
        new_w = int(area_h * img_ratio)

    img = img.resize((new_w, new_h), Image.BILINEAR)

    img_tk = ImageTk.PhotoImage(img)
    image_label.config(image=img_tk)
    image_label.image = img_tk

    status_label.config(text=f"Captured ({img.width}x{img.height})")

# -------------------------
# LOW LIGHT FUNCTIONS
# -------------------------
def set_exposure_ms(ms):
    ms = int(ms)

    msb = (ms >> 8) & 0xFF
    lsb = ms & 0xFF

    ser.write(bytes([0x21, msb, lsb]))
    status_label.config(text=f"Exposure set to {ms} ms")


def set_gain(value):
    ser.write(bytes([0x22, value]))
    status_label.config(text=f"Gain set to 0x{value:02X}")

def update_temperature():
    latest_temp = None
    latest_imu = None

    try:
        while ser.in_waiting:
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if not line:
                continue

            # ---- Temperature ----
            if line.startswith("TMP:"):
                latest_temp = float(line.split(":")[1])

            # ---- IMU ----
            elif line.startswith("IMU:"):
                values = line.split(":")[1].split(",")
                if len(values) == 6:
                    latest_imu = [float(v) for v in values]
            elif line.startswith("V:"):
                vbat = float(line.split(":")[1])
                latest_voltage.config(text=f"VBAT: {vbat:.2f} V")

        if latest_temp is not None:
            object_temp_var.set(f"{latest_temp:.2f} °C")

        if latest_imu is not None:
            ax, ay, az, gx, gy, gz = latest_imu
            imu_var.set(
                f"{ax:.2f}, {ay:.2f}, {az:.2f} | "
                f"{gx:.1f}, {gy:.1f}, {gz:.1f}"
            )
        if latest_voltage is not None:
            object_voltage_var.set(f"{vbat:.2f} %")

    except Exception:
        pass

    root.after(250, update_temperature)

# -------------------------
# CLEAN EXIT
# -------------------------
def on_close():
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# -------------------------
# START
# -------------------------
update_temperature()
root.mainloop()
