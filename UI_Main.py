import serial
import time
import tkinter as tk
from tkinter import ttk, filedialog
from PIL import Image, ImageTk
import io

# -------------------------
# Configuration
# -------------------------
SERIAL_PORT = "COM11"  # Change to your port
BAUD_RATE = 112500
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# -------------------------
# ArduCAM commands
# -------------------------
CMD_CAPTURE_SINGLE = 0x10
CMD_SET_BRIGHTNESS = { "B-2":0x64, "B-1":0x63, "B0":0x62, "B+1":0x61, "B+2":0x60 }
CMD_SET_CONTRAST   = { "C-2":0x74, "C-1":0x73, "C0":0x72, "C+1":0x71, "C+2":0x70 }
CMD_SET_SATURATION = { "S-2":0x54, "S-1":0x53, "S0":0x52, "S+1":0x51, "S+2":0x50 }
CMD_SPECIAL_EFFECTS= { "Normal":0x87, "Negative":0x85, "BW":0x84, "Antique":0x80 }

# -------------------------
# Serial setup
# -------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
time.sleep(2)  # wait for Arduino to reset

# -------------------------
# Add a global variable to store the displayed image data
# -------------------------
displayed_image_data = None

# -------------------------
# Capture single image
# -------------------------
def capture_image():
    global displayed_image_data
    ser.write(bytes([CMD_CAPTURE_SINGLE]))
    image_data = bytearray()
    started = False

    while True:
        byte = ser.read(1)
        if not byte:
            continue
        b = byte[0]
        if not started:
            image_data.append(b)
            if len(image_data) >= 2 and image_data[-2:] == b'\xFF\xD8':
                started = True
                image_data = bytearray(b'\xFF\xD8')
        else:
            image_data.append(b)
            if len(image_data) >= 2 and image_data[-2:] == b'\xFF\xD9':
                break

    # Store the image data globally
    displayed_image_data = image_data

    # Display image in Tkinter
    img = Image.open(io.BytesIO(image_data))
    img = img.resize((IMAGE_WIDTH, IMAGE_HEIGHT))
    img_tk = ImageTk.PhotoImage(img)
    panel.config(image=img_tk)
    panel.image = img_tk

# -------------------------
# Add a function to save the displayed image
# -------------------------
def save_image():
    global displayed_image_data
    if displayed_image_data is None:
        print("No image to save.")
        return

    filename = filedialog.asksaveasfilename(defaultextension=".jpg", filetypes=[("JPEG files", "*.jpg")])
    if filename:
        with open(filename, "wb") as f:
            f.write(displayed_image_data)
        print(f"Saved image to {filename}")

# -------------------------
# Move the console creation to the correct place after root is defined
# -------------------------
# -------------------------
# GUI setup
# -------------------------
root = tk.Tk()
root.title("ArduCAM GUI")

# Add a console to display commands and replies
console = tk.Text(root, height=10, state="disabled")
console.pack()

def log_to_console(message):
    console.config(state="normal")
    console.insert("end", message + "\n")
    console.see("end")
    console.config(state="disabled")

# -------------------------
# Send camera commands
# -------------------------
def send_command(mapping, value):
    cmd = mapping[value]
    ser.write(bytes([cmd]))
    log_to_console(f"Sent command: {value} (0x{cmd:02X})")

# -------------------------
# Update the resolution options and commands
# -------------------------
RESOLUTION_OPTIONS = {
    "160x120": 0x02,
    "320x240": 0x03,
    "640x480": 0x04,
    "800x600": 0x05,
    "1024x768": 0x06,
    "1280x1024": 0x07,
    "1600x1200": 0x08
}

def change_resolution(event):
    resolution = resolution_combo.get()
    if resolution in RESOLUTION_OPTIONS:
        cmd = RESOLUTION_OPTIONS[resolution]
        ser.write(bytes([cmd]))
        log_to_console(f"Sent resolution change command: {resolution} (0x{cmd:02X})")
        # Example of reading a reply from the camera
        reply = ser.read(10).decode(errors="ignore")  # Adjust the number of bytes as needed
        log_to_console(f"Camera reply: {reply}")
    else:
        log_to_console("Invalid resolution selected.")

# -------------------------
# GUI setup
# -------------------------
root = tk.Tk()
root.title("ArduCAM GUI")

panel = tk.Label(root)
panel.pack()

# Capture button
btn_capture = tk.Button(root, text="Capture Image", command=capture_image)
btn_capture.pack()

# Save Image button
btn_save = tk.Button(root, text="Save Image", command=save_image)
btn_save.pack()

# Brightness
tk.Label(root, text="Brightness").pack()
brightness_combo = ttk.Combobox(root, values=list(CMD_SET_BRIGHTNESS.keys()))
brightness_combo.set("B0")
brightness_combo.pack()
def brightness_changed(event):
    send_command(CMD_SET_BRIGHTNESS, brightness_combo.get())
brightness_combo.bind("<<ComboboxSelected>>", brightness_changed)

# Contrast
tk.Label(root, text="Contrast").pack()
contrast_combo = ttk.Combobox(root, values=list(CMD_SET_CONTRAST.keys()))
contrast_combo.set("C0")
contrast_combo.pack()
def contrast_changed(event):
    send_command(CMD_SET_CONTRAST, contrast_combo.get())
contrast_combo.bind("<<ComboboxSelected>>", contrast_changed)

# Saturation
tk.Label(root, text="Saturation").pack()
saturation_combo = ttk.Combobox(root, values=list(CMD_SET_SATURATION.keys()))
saturation_combo.set("S0")
saturation_combo.pack()
def saturation_changed(event):
    send_command(CMD_SET_SATURATION, saturation_combo.get())
saturation_combo.bind("<<ComboboxSelected>>", saturation_changed)

# Special effects
tk.Label(root, text="Special Effects").pack()
effects_combo = ttk.Combobox(root, values=list(CMD_SPECIAL_EFFECTS.keys()))
effects_combo.set("Normal")
effects_combo.pack()
def effects_changed(event):
    send_command(CMD_SPECIAL_EFFECTS, effects_combo.get())
effects_combo.bind("<<ComboboxSelected>>", effects_changed)

# Resolution
tk.Label(root, text="Resolution").pack()
resolution_combo = ttk.Combobox(root, values=list(RESOLUTION_OPTIONS.keys()))
resolution_combo.set("320x240")
resolution_combo.pack()
resolution_combo.bind("<<ComboboxSelected>>", change_resolution)

root.mainloop()
