import serial
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import io
from PIL import ImageDraw, ImageFont
import math
import threading

# -------------------------
# CONFIG
# -------------------------
SERIAL_PORT = "COM16"   # CHANGE
BAUD_RATE = 115200
FONT_SIZE = 18
FONT_COLOR = (0, 255, 255)  # cyan-ish

try:
    FONT = ImageFont.truetype("arial.ttf", FONT_SIZE)
except:
    FONT = ImageFont.load_default()
# -------------------------
# IMPACT DETECTION CONFIG
# -------------------------
IMPACT_THRESHOLD_G = 1.2      # g's above baseline
IMPACT_COOLDOWN_MS = 1500     # minimum time between captures
IMPACT_MIN_DELTA = 0.4        # sudden change filter

last_impact_time = 0
last_accel_mag = None
# -------------------------
# STATE
# -------------------------
deployed = False
deploy_start_time = None
impact_active = False



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
root.grid_columnconfigure(0, minsize=500, weight=0)  # left panel
root.grid_columnconfigure(1, weight=1)  # image area

# -------------------------
# LEFT DATA PANEL
# -------------------------
data_panel = tk.Frame(root, bg="#1e1e1e", width=500)
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
# -------------------------
# STATUS LABEL (BIG)
# -------------------------
status_var = tk.StringVar(value="READY")

status_big = tk.Label(
    data_panel,
    textvariable=status_var,
    fg="#aaaaaa",
    bg="#1e1e1e",
    font=("Segoe UI", 22, "bold"),
    width=16,
    anchor="center"
)
status_big.pack(pady=(0, 15))

# -------------------------
# MISSION TIMER (BIG)
# -------------------------
timer_var = tk.StringVar(value="00:00")

timer_label = tk.Label(
    data_panel,
    textvariable=timer_var,
    fg="#00ffcc",
    bg="#1e1e1e",
    font=("Consolas", 36, "bold"),
    width=8,
    anchor="center"
)
timer_label.pack(pady=(0, 25))

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
    font=("Consolas", 18, "bold"),
    width=12,                      
    anchor="w"
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
    font=("Consolas", 11),
    width=32,     # enough for full IMU line
    anchor="w"
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
    fg="#ff4444",
    bg="#1e1e1e",
    font=("Consolas", 18, "bold"),
    width=10,
    anchor="w"
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
    text="Deploy",
    font=("Segoe UI", 11),
    command=lambda: deploy()
)
reset_btn = tk.Button(
    control_bar,
    text="Reset",
    font=("Segoe UI", 11),
    command=lambda: reset_deploy()
)
capture_btn.pack(side="left", padx=20)
reset_btn.pack(side="left")
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
def deploy():
    global deployed, deploy_start_time
    status_label.config(text="Deploying...")
    root.update_idletasks()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)
    ser.write(bytes([0x99]))
    set_status("DEPLOYED")
    deployed = True
    deploy_start_time = time.time()


def reset_deploy():
    global deployed, deploy_start_time
    status_label.config(text="Resetting...")
    root.update_idletasks()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)
    ser.write(bytes([0x98]))
    set_status("READY")
    deployed = False
    deploy_start_time = None
    timer_var.set("00:00")

def set_status(state):
    status_var.set(state)

    if state == "READY":
        status_big.config(fg="#aaaaaa")
    elif state == "DEPLOYED":
        status_big.config(fg="#00ff55")
    elif state == "IMPACT":
        status_big.config(fg="#ff3333")

def update_timer():
    if deployed and deploy_start_time is not None:
        elapsed = int(time.time() - deploy_start_time)
        mins = elapsed // 60
        secs = elapsed % 60
        timer_var.set(f"{mins:02d}:{secs:02d}")

    root.after(500, update_timer)

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
    draw = ImageDraw.Draw(img)

    # ---- Gather latest values ----
    temp_text = object_temp_var.get()
    imu_text = imu_var.get()
    voltage_text = object_voltage_var.get()

    overlay_text = (
        f"TEMP: {temp_text}\n"
        f"IMU:  {imu_text}\n"
        f"BAT:  {voltage_text}"
    )

    # ---- Text positioning (top-right) ----
    padding = 10
    text_bbox = draw.multiline_textbbox((0, 0), overlay_text, font=FONT)
    text_w = text_bbox[2] - text_bbox[0]
    text_h = text_bbox[3] - text_bbox[1]

    x = text_w - padding
    y = padding

    # ---- Optional background box for readability ----
    bg_padding = 6
    draw.rectangle(
        [
            x - bg_padding,
            y - bg_padding,
            x + text_w + bg_padding,
            y + text_h + bg_padding,
        ],
        fill=(0, 0, 0, 160)
    )

    # ---- Draw text ----
    draw.multiline_text(
        (x, y),
        overlay_text,
        fill=FONT_COLOR,
        font=FONT,
        spacing=4,
    )


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
def image_receiver():
    jpg = bytearray()
    prev = None
    in_image = False

    while True:
        try:
            b = ser.read(1)
            if not b:
                continue

            cur = b[0]

            if not in_image:
                if prev == 0xFF and cur == 0xD8:
                    in_image = True
                    jpg = bytearray([0xFF, 0xD8])
            else:
                jpg.append(cur)
                if prev == 0xFF and cur == 0xD9:
                    # Full JPEG received
                    process_image(bytes(jpg))
                    in_image = False
                    jpg = bytearray()

            prev = cur

        except Exception:
            pass
def process_image(jpg_bytes):
    try:
        img = Image.open(io.BytesIO(jpg_bytes))

        # Resize to image area
        area_w = image_label.winfo_width()
        area_h = image_label.winfo_height()

        if area_w <= 1 or area_h <= 1:
            return

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

        def update_ui():
            image_label.config(image=img_tk)
            image_label.image = img_tk

        root.after(0, update_ui)

    except Exception as e:
        print("Image error:", e)

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
                object_voltage_var.set(f"{vbat:.0f} V")

        if latest_temp is not None:
            object_temp_var.set(f"{latest_temp:.2f} °C")

        if latest_imu is not None:
            ax, ay, az, gx, gy, gz = latest_imu
            accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
            imu_var.set(
                f"{ax:.2f}, {ay:.2f}, {az:.2f} | "
                f"{gx:.1f}, {gy:.1f}, {gz:.1f}"
            )
            check_for_impact(accel_mag)
        if latest_voltage is not None:
            object_voltage_var.set(f"{vbat:.2f} %")

    except Exception:
        pass

    root.after(250, update_temperature)
def check_for_impact(accel_mag):
    global last_impact_time, last_accel_mag

    now = int(time.time() * 1000)

    # First sample
    if last_accel_mag is None:
        last_accel_mag = accel_mag
        return

    delta = abs(accel_mag - last_accel_mag)
    last_accel_mag = accel_mag

    # Impact conditions
    impact_detected = (
        accel_mag > IMPACT_THRESHOLD_G and
        delta > IMPACT_MIN_DELTA and
        (now - last_impact_time) > IMPACT_COOLDOWN_MS
    )
    status_label.config(text=f"Accel Mag: {accel_mag:.2f}")
    if impact_detected:
        last_impact_time = now
        status_label.config(text="IMPACT DETECTED – CAPTURING")
        set_status("IMPACT")
        root.after(0, capture_image)


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
update_timer()
threading.Thread(target=image_receiver, daemon=True).start()

root.mainloop()
