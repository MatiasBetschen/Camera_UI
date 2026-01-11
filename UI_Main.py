import serial
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import io
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import queue

# -------------------------
# CONFIG
# -------------------------
SERIAL_PORT = "COM11"   # CHANGE
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
root.grid_columnconfigure(0, weight=1)

image_label = tk.Label(root, bg="black")
image_label.grid(row=0, column=0, sticky="nsew")

control_bar = tk.Frame(root, bg="#2b2b2b", height=50)
control_bar.grid(row=1, column=0, sticky="ew")
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
resolution_combo.set("320 x 240")
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

status_label = tk.Label(control_bar, text="Ready", fg="white", bg="#2b2b2b")
status_label.pack(side="right", padx=10)

# -------------------------
# DATA QUEUE FOR GRAPH
# -------------------------
data_queue = queue.Queue()

def read_serial():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("Gyro"):
                try:
                    data = [float(x) for x in line.split(": ")[1].split(", ")]
                    data_queue.put(data)
                except ValueError:
                    continue

serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# -------------------------
# GRAPH SETUP
# -------------------------
fig, ax = plt.subplots()
ax.set_title("Live Gyro Data")
ax.set_xlabel("Time")
ax.set_ylabel("Gyro [deg/s]")

x_data, gx_data, gy_data, gz_data = [], [], [], []

def update(frame):
    while not data_queue.empty():
        data = data_queue.get()
        x_data.append(len(x_data))
        gx_data.append(data[0])
        gy_data.append(data[1])
        gz_data.append(data[2])

        if len(x_data) > 100:
            x_data.pop(0)
            gx_data.pop(0)
            gy_data.pop(0)
            gz_data.pop(0)

    ax.clear()
    ax.plot(x_data, gx_data, label="Gyro X")
    ax.plot(x_data, gy_data, label="Gyro Y")
    ax.plot(x_data, gz_data, label="Gyro Z")
    ax.legend()
    ax.set_title("Live Gyro Data")
    ax.set_xlabel("Time")
    ax.set_ylabel("Gyro [deg/s]")

ani = FuncAnimation(fig, update, interval=100)

# -------------------------
# CAPTURE FUNCTION
# -------------------------
def capture_image():
    status_label.config(text="Capturing...")
    root.update_idletasks()

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)
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
# DASHBOARD LAYOUT
# -------------------------
def show_dashboard():
    dashboard = tk.Toplevel(root)
    dashboard.title("Dashboard")

    # Image Viewer
    image_frame = tk.Frame(dashboard)
    image_frame.pack(side="left", fill="both", expand=True)

    image_label = tk.Label(image_frame, bg="black")
    image_label.pack(fill="both", expand=True)

    # Graph Viewer
    graph_frame = tk.Frame(dashboard)
    graph_frame.pack(side="right", fill="both", expand=True)

    canvas = plt.backends.backend_tkagg.FigureCanvasTkAgg(fig, master=graph_frame)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    dashboard.protocol("WM_DELETE_WINDOW", on_close)

# Add Dashboard Button
btn_dashboard = tk.Button(control_bar, text="Show Dashboard", command=show_dashboard)
btn_dashboard.pack(side="left", padx=20)

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
root.mainloop()
