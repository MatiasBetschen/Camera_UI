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
IMPACT_THRESHOLD_G = 1.15     # g's above baseline
IMPACT_COOLDOWN_MS = 100     # minimum time between captures
IMPACT_MIN_DELTA = 0.05       # sudden change filter

last_impact_time = 0
last_accel_mag = None
# -------------------------
# STATE
# -------------------------
deployed = False
deploy_start_time = None
impact_active = False
image_buffer = []          # list of PIL Images
MAX_IMAGES = 50            # safety limit
playback_active = False

# -------------------------
# DEBUG MODE
# -------------------------
debug_mode = False
debug_last_frame = None


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
root.grid_columnconfigure(0, minsize=200, weight=0)  # left panel
root.grid_columnconfigure(1, weight=1)  # image area

# -------------------------
# LEFT DATA PANEL
# -------------------------
data_panel = tk.Frame(root, bg="#1e1e1e", width=200)
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
    fg="white",
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

imu_accel_var = tk.StringVar(value="--,--,--")

imu_accel_label = tk.Label(
    data_panel,
    textvariable=imu_accel_var,
    fg="white",
    bg="#1e1e1e",
    font=("Consolas", 11),
    width=32,     # enough for full IMU line
    anchor="w"
)
imu_accel_label.pack(anchor="w", padx=15, pady=(10, 0))
imu_gyro_var = tk.StringVar(value="--,--,--")

imu_gyro_label = tk.Label(
    data_panel,
    textvariable=imu_gyro_var,
    fg="white",
    bg="#1e1e1e",
    font=("Consolas", 11),
    width=32,     # enough for full IMU line
    anchor="w"
)
imu_gyro_label.pack(anchor="w", padx=15, pady=(5, 20))
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
    fg="white",
    bg="#1e1e1e",
    font=("Consolas", 18, "bold"),
    width=10,
    anchor="w"
)
latest_voltage.pack(anchor="w", padx=15, pady=(5, 20))
impact_debug_var = tk.StringVar(value="Impact Debug: --")

impact_debug_label = tk.Label(
    data_panel,
    textvariable=impact_debug_var,
    fg="#ffaa00",
    bg="#1e1e1e",
    font=("Consolas", 9),
    wraplength=180,
    justify="left"
)
impact_debug_label.pack(anchor="w", padx=15, pady=(0, 10))

button_frame = tk.Frame(data_panel, bg="#1e1e1e")
button_frame.pack(side="bottom", pady=15)

capture_btn = tk.Button(
    button_frame,
    text="Deploy",
    font=("Segoe UI", 11),
    width=12,
    command=lambda: deploy()
)
reset_btn = tk.Button(
    button_frame,
    text="Reset",
    font=("Segoe UI", 11),
    width=12,
    command=lambda: reset_deploy()
)
debug_start_btn = tk.Button(
    button_frame,
    text="Start Debug",
    font=("Segoe UI", 11),
    width=12,
    command=lambda: start_debug()
)

debug_stop_btn = tk.Button(
    button_frame,
    text="Stop Debug",
    font=("Segoe UI", 11),
    width=12,
    command=lambda: stop_debug()
)

debug_start_btn.pack(side="left", padx=10)
debug_stop_btn.pack(side="left", padx=10)

capture_btn.pack(side="left", padx=10)
reset_btn.pack(side="left", padx=10)
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
resolution_combo.set("800 x 600")
resolution_combo.pack(side="left")
resolution_combo.bind(
    "<<ComboboxSelected>>",
    lambda e: ser.write(bytes([RESOLUTIONS[resolution_combo.get()]]))
)

status_label = tk.Label(control_bar, text="Ready", fg="white", bg="#2b2b2b")
status_label.pack(side="right", padx=10)

# -------------------------
# CAPTURE FUNCTION
# -------------------------
def start_debug():
    global debug_mode, playback_active
    debug_mode = True
    playback_active = False
    set_status("DEBUG")
    status_label.config(text="DEBUG MODE – LIVE VIEW")

    # Tell MCU to start continuous capture
    ser.write(bytes([0x99]))   # reuse deploy / start camera command


def stop_debug():
    global debug_mode
    debug_mode = False
    set_status("READY")
    status_label.config(text="Debug stopped")

    # Stop camera on MCU
    ser.write(bytes([0x98]))

def deploy():
    global deployed, deploy_start_time
    status_label.config(text="Deploying...")
    root.update_idletasks()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)
    ser.write(bytes([RESOLUTIONS[resolution_combo.get()]]))
    time.sleep(0.05)
    ser.write(bytes([0x99]))
    set_status("DEPLOYED")
    deployed = True
    deploy_start_time = time.time()
def reset_deploy():
    global deployed, deploy_start_time, playback_active,last_accel_mag, last_impact_time
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
    object_temp_var.set("--.- °C")
    imu_accel_var.set("--,--,--")
    imu_gyro_var.set("--,--,--")
    object_voltage_var.set("--.- %")
    image_buffer.clear()
    playback_active = False
    image_label.config(image="")
    image_label.image = None
    last_accel_mag = None
    last_impact_time = 0

def start_playback():
    global playback_active
    playback_active = True

    if not image_buffer:
        status_label.config(text="No images captured")
        return

    status_label.config(text="PLAYBACK STARTED")

    show_image_index(0)


def show_image_index(index):
    if index >= len(image_buffer):
        status_label.config(text="PLAYBACK COMPLETE")
        return

    img = image_buffer[index]

    # Resize to fit display
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

    img_disp = img.resize((new_w, new_h), Image.BILINEAR)

    img_tk = ImageTk.PhotoImage(img_disp)
    image_label.config(image=img_tk)
    image_label.image = img_tk

    status_label.config(text=f"Playback {index+1}/{len(image_buffer)}")

    # Show next image after 5 seconds
    root.after(2500, lambda: show_image_index(index + 1))

def set_status(state):
    global deployed
    status_var.set(state)
    if state == "READY":
        status_big.config(fg="#aaaaaa")
    elif state == "DEPLOYED":
        status_big.config(fg="#00ff55")
    elif state == "IMPACT":
        deployed=False
        status_big.config(fg="#ff3333")
def update_timer():
    if deployed and deploy_start_time is not None:
        elapsed = int(time.time() - deploy_start_time)
        mins = elapsed // 60
        secs = elapsed % 60
        timer_var.set(f"{mins:02d}:{secs:02d}")

    root.after(500, update_timer)
def serial_receiver():
    jpg = bytearray()
    line_buf = bytearray()
    prev = None
    in_image = False

    while True:
        b = ser.read(1)
        if not b:
            continue

        cur = b[0]

        if in_image:
            jpg.append(cur)
            if prev == 0xFF and cur == 0xD9:
                process_image(bytes(jpg))
                jpg.clear()
                in_image = False
        else:
            if prev == 0xFF and cur == 0xD8:
                jpg = bytearray([0xFF, 0xD8])
                in_image = True
            else:
                line_buf.append(cur)
                if cur == ord('\n'):
                    handle_text_line(line_buf.decode(errors="ignore").strip())
                    line_buf.clear()

        prev = cur
def handle_text_line(line):
    global playback_active
    try:
        if line.startswith("TMP:"):
            t = float(line.split(":")[1])
            root.after(0, lambda: object_temp_var.set(f"{t:.2f} °C"))

        elif line.startswith("IMU:"):
            vals = [float(v) for v in line.split(":")[1].split(",")]
            ax, ay, az, gx, gy, gz = vals
            mag = math.sqrt(ax*ax + ay*ay + az*az)

            def update_imu_labels():
                imu_accel_var.set(f"{ax:.2f}, {ay:.2f}, {az:.2f} [g]")
                imu_gyro_var.set(f"{gx:.1f}, {gy:.1f}, {gz:.1f} [°/s]")

            root.after(0, update_imu_labels)

        elif line.startswith("VEL:"):
            vals = [float(v) for v in line.split(":")[1].split(",")]
            velx, vely, velz= vals

            def update_vel_labels():
                impact_debug_var.set(f"VEL={velx:.3f}, {vely:.3f}, {velz:.3f} [m/s]")

            root.after(0, update_vel_labels)
            #check_for_impact(mag)

        elif line.startswith("V:"):
            v = float(line.split(":")[1])
            root.after(0, lambda: object_voltage_var.set(f"{v:.2f} V"))
        elif line == "IMPACT":
            # Stop further image buffering
            playback_active = True  # Prevents new images from being buffered
            
            # Update status
            root.after(0, lambda: set_status("IMPACT"))
            root.after(0, lambda: status_label.config(text="IMPACT DETECTED – STOPPING CAMERA"))
            
            # Stop camera
            ser.write(bytes([0x98]))
            
            # Start playback after short delay
            root.after(1500, start_playback)

    except Exception:
        pass


def process_image(jpg_bytes):
    global image_buffer, playback_active, debug_mode

    try:
        img = Image.open(io.BytesIO(jpg_bytes)).convert("RGBA")

        # ---- Resize to UI size ----
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
        if debug_mode:
            # LIVE VIEW – no buffering, no overlay needed (optional)
            img_tk = ImageTk.PhotoImage(img)
            root.after(
                0,
                lambda im=img_tk: (
                    image_label.config(image=im),
                    setattr(image_label, "image", im)
                )
            )
            return


        # ---- DRAW OVERLAY (unchanged) ----
        draw = ImageDraw.Draw(img)

        overlay_text = (
            f"TEMP: {object_temp_var.get()}\n"
            f"ACCEL:  {imu_accel_var.get()}\n"
            f"GYRO: {imu_gyro_var.get()}\n"
            f"BAT:  {object_voltage_var.get()}\n"
            f"TIME: {timer_var.get()}\n"
            f"IMG:  {len(image_buffer)+1}"  # +1 for current image
        )

        padding = 10
        bg_padding = 6

        text_bbox = draw.multiline_textbbox(
            (0, 0), overlay_text, font=FONT, spacing=4
        )

        text_w = text_bbox[2] - text_bbox[0]
        text_h = text_bbox[3] - text_bbox[1]

        x = img.width - text_w - padding
        y = padding

        draw.rectangle(
            (
                x - bg_padding,
                y - bg_padding,
                x + text_w + bg_padding,
                y + text_h + bg_padding,
            ),
            fill=(0, 0, 0, 160),
        )

        draw.multiline_text(
            (x, y),
            overlay_text,
            fill=FONT_COLOR,
            font=FONT,
            spacing=4,
        )

        # ---- STORE INSTEAD OF DISPLAY ----
        if len(image_buffer) < MAX_IMAGES:
            image_buffer.append(img.copy())

        status_label.config(text=f"Buffered images: {len(image_buffer)}")

    except Exception as e:
        print("Image processing failed:", e)



def on_close():
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
update_timer()
threading.Thread(target=serial_receiver, daemon=True).start()

root.mainloop()
