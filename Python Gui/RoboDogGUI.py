import tkinter as tk
from tkinter import ttk
import serial
import time
import threading
import json
import numpy as np
from scipy.signal import savgol_filter

# Configure the serial connection
ser = serial.Serial('COM4', 115200)  # Replace 'COM6' with your Arduino's serial port
time.sleep(2)  # Wait for the serial connection to initialize

# Global variables to store angles, initialization state, and recording data
angles = [0, 0, 0]
initialized = [False, False, False]  # Track if each slider has been initialized
recording = False
recorded_data = []
joint_addresses = [0x127, 0x125, 0x123]
recording_thread = None
recording_lock = threading.Lock()  # Lock for synchronizing access to recorded_data

# Global variable to store playback speed
playback_speed = 1.0

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

# Function to start recording angles
def start_recording():
    global recording, recorded_data, recording_thread
    recording = True
    with recording_lock:
        recorded_data = []  # Clear any previous recordings
    record_button.config(bg="red")  # Indicate recording in progress

    # Turn off all motors
    for i in range(len(joint_addresses)):
        send_motor_off_command(i)

    def record_loop():
        while recording:
            with recording_lock:
                recorded_data.append(list(angles))
            time.sleep(1 / 20)  # Record at 20 Hz

    recording_thread = threading.Thread(target=record_loop, daemon=True)
    recording_thread.start()
    print("Recording started")

# Function to stop recording angles
def stop_recording():
    global recording, recorded_data  # Ensure recorded_data is referenced as a global variable
    recording = False
    if recording_thread is not None:
        recording_thread.join()  # Ensure the recording thread has stopped
    record_button.config(bg="SystemButtonFace")  # Reset button color
    print("Recording stopped")

    # Apply Savitzky-Golay filter to recorded data
    with recording_lock:
        if recorded_data:
            recorded_data = np.array(recorded_data)
            filtered_data = savgol_filter(recorded_data, window_length=11, polyorder=3, axis=0)
            recorded_data = filtered_data.tolist()

    # Save recorded data to a text file
    with recording_lock:
        with open("recorded_data.txt", "w") as file:
            json.dump(recorded_data, file)
    print("Recorded data saved to recorded_data.txt")

# Function to update playback speed
def update_playback_speed(value):
    global playback_speed
    playback_speed = float(value)
    print(f"Playback speed set to {playback_speed}x")

# Function to play back recorded angles from a text file
def playback_recording():
    try:
        with open("recorded_data.txt", "r") as file:
            recorded_data = json.load(file)
    except FileNotFoundError:
        print("No recorded data file found")
        return

    print("Playback started")
    for frame in recorded_data:
        for i, angle in enumerate(frame):
            ser.write(f"SET_ANGLE:0x{joint_addresses[i]:X}:{angle}\n".encode())
            print(f"SET_ANGLE:0x{joint_addresses[i]:X}:{angle}")
        time.sleep((1 / 20) / playback_speed)  # Adjust playback speed
    print("Playback finished")

# Function to process and update motor data
def process_data(line):
    parts = line.split(":")
    if len(parts) != 2:
        return  # Ignore malformed data

    joint_name = parts[0]
    data_parts = parts[1].split(",")
    if len(data_parts) != 2:
        return  # Ignore malformed data

    angle = float(data_parts[0].split("=")[1])
    current = data_parts[1].split("=")[1]

    # Update the appropriate joint labels and sliders
    if joint_name == "HIP":
        angle_labels[0].config(text=f"Current Angle: {angle}")
        current_labels[0].config(text=f"Motor Current: {current}")
        angles[0] = angle  # Update the angles list
        if not initialized[0]:
            sliders[0].set(angle)
            initialized[0] = True
    elif joint_name == "KNEE":
        angle_labels[1].config(text=f"Current Angle: {angle}")
        current_labels[1].config(text=f"Motor Current: {current}")
        angles[1] = angle  # Update the angles list
        if not initialized[1]:
            sliders[1].set(angle)
            initialized[1] = True
    elif joint_name == "ANKLE":
        angle_labels[2].config(text=f"Current Angle: {angle}")
        current_labels[2].config(text=f"Motor Current: {current}")
        angles[2] = angle  # Update the angles list
        if not initialized[2]:
            sliders[2].set(angle)
            initialized[2] = True

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
    sliders.append(slider)

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

# Add Record and Playback buttons
record_button = tk.Button(root, text="Record", command=lambda: start_recording() if not recording else stop_recording())
record_button.pack(pady=10)

playback_button = tk.Button(root, text="Playback", command=playback_recording)
playback_button.pack(pady=10)

# Add Playback Speed slider
playback_speed_label = tk.Label(root, text="Playback Speed", font=("Helvetica", 14))
playback_speed_label.pack(pady=5)

playback_speed_slider = ttk.Scale(root, from_=1.0, to=10.0, orient='horizontal', command=update_playback_speed, length=400)
playback_speed_slider.set(1.0)  # Set initial value to 1x
playback_speed_slider.pack(pady=5)

# Add tick marks to the slider
tick_frame = tk.Frame(root)
tick_frame.pack(pady=5)

for i in range(1, 11):
    tick_label = tk.Label(tick_frame, text=f"{i}x", font=("Helvetica", 10))
    tick_label.pack(side=tk.LEFT, padx=15)

# Start the thread to update motor data
thread = threading.Thread(target=update_motor_data, daemon=True)
thread.start()

# Run the Tkinter event loop
root.mainloop()

# Close the serial connection when the GUI is closed
ser.close()