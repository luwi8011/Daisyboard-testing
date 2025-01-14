import tkinter as tk
from tkinter import ttk
import serial
import time
import threading

# Configure the serial connection
ser = serial.Serial('COM6', 115200)  # Replace 'COM6' with your Arduino's serial port
time.sleep(2)  # Wait for the serial connection to initialize

# Global variables to store angles
angles = [0, 0, 0]
# Joint addresses
joint_addresses = [0x127, 0x125, 0x123]

# Function to send target angle to the master
def send_target_angle(slave_id, value):
    angles[slave_id] = int(float(value))
    ser.write(f"SET_ANGLE:0x{joint_addresses[slave_id]:X}:{angles[slave_id]}\n".encode())
    print(f"SET_ANGLE:0x{joint_addresses[slave_id]:X}:{angles[slave_id]}")

# Function to send ZERO command to the master
def send_zero_command(slave_id):
    ser.write(f"ZERO:0x{joint_addresses[slave_id]:X}:\n".encode())
    print(f"ZERO:0x{joint_addresses[slave_id]:X}:")

# Function to send Motor off command to the master
def send_motor_off_command(slave_id):
    ser.write(f"SET_ANGLE:0x{joint_addresses[slave_id]:X}:0\n".encode())
    print(f"SET_ANGLE:0x{joint_addresses[slave_id]:X}:0")

# Function to process and update motor data
def process_data(line):
    parts = line.split(":")
    if len(parts) != 2:
        return  # Ignore malformed data

    joint_name = parts[0]
    data_parts = parts[1].split(",")
    if len(data_parts) != 2:
        return  # Ignore malformed data

    angle = data_parts[0].split("=")[1]
    current = data_parts[1].split("=")[1]

    # Update the appropriate joint labels
    if joint_name == "HIP":
        angle_labels[0].config(text=f"Current Angle: {angle}")
        current_labels[0].config(text=f"Motor Current: {current}")
    elif joint_name == "KNEE":
        angle_labels[1].config(text=f"Current Angle: {angle}")
        current_labels[1].config(text=f"Motor Current: {current}")
    elif joint_name == "ANKLE":
        angle_labels[2].config(text=f"Current Angle: {angle}")
        current_labels[2].config(text=f"Motor Current: {current}")

# Function to read and process serial data
def update_motor_data():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print(f"Received: {line}")  # Debugging print
            root.after(0, process_data, line)  # Ensure GUI updates occur in the main thread

# Create the main window
root = tk.Tk()
root.title("RoboDog Control Panel")

# Create sliders, labels, and buttons for each slave
sliders = []
angle_labels = []
current_labels = []
for i, (joint_name, max_value) in enumerate([("HIP", 148), ("KNEE", 80), ("ANKLE", 70)]):
    frame = tk.Frame(root)
    frame.pack(pady=10)

    label = tk.Label(frame, text=f"{joint_name} Target Angle", font=("Helvetica", 14))
    label.pack()

    slider = ttk.Scale(frame, from_=0, to=max_value, orient='horizontal', command=lambda value, i=i: send_target_angle(i, value), length=400)
    slider.pack()

    angle_label = tk.Label(frame, text="Current Angle: 0", font=("Helvetica", 12))
    angle_label.pack()
    angle_labels.append(angle_label)

    current_label = tk.Label(frame, text="Motor Current: 0", font=("Helvetica", 12))
    current_label.pack()
    current_labels.append(current_label)

    button_frame = tk.Frame(frame)
    button_frame.pack(pady=5)

    zero_button = tk.Button(button_frame, text="ZERO", command=lambda i=i: send_zero_command(i))
    zero_button.pack(side=tk.LEFT, padx=5)

    motor_off_button = tk.Button(button_frame, text="Motor off", command=lambda i=i: send_motor_off_command(i))
    motor_off_button.pack(side=tk.LEFT, padx=5)

# Start the thread to update motor data
thread = threading.Thread(target=update_motor_data, daemon=True)
thread.start()

# Run the Tkinter event loop
root.mainloop()

# Close the serial connection when the GUI is closed
ser.close()
