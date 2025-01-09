import tkinter as tk
from tkinter import ttk
import serial
import time
import threading

# Configure the serial connection
ser = serial.Serial('COM6', 115200)  # Replace 'COM3' with your Arduino's serial port
time.sleep(2)  # Wait for the serial connection to initialize
angle = 0
angle2 = 0  # Declare the second angle variable

def send_target_angle(value):
    global angle
    angle = int(float(value))
    print(f"New target angle set via GUI: {angle}")

def send_target_angle2(value):
    global angle2
    angle2 = int(float(value))
    print(f"New target angle2 set via GUI: {angle2}")

def custom_main_loop():
    last_time = time.time()
    while True:
        # Your custom code here

        current_time = time.time()
        if (current_time - last_time) >= 0.2:  # 20 milliseconds
            # Run your command every 20 milliseconds
            ser.write(f"{angle}a\n".encode())
            ser.write(f"{angle2}b\n".encode())
            last_time = current_time
        
        time.sleep(0.001)  # Sleep for 1ms to prevent high CPU usage

# Create the main window
root = tk.Tk()
root.title("Target Angle Adjuster")

# Create the first slider
slider = ttk.Scale(root, from_=5, to=150, orient='horizontal', command=send_target_angle, length=800)
slider.pack(pady=20)

# Create the second slider
slider2 = ttk.Scale(root, from_=5, to=150, orient='horizontal', command=send_target_angle2, length=800)
slider2.pack(pady=20)

# Create a label to display the current value of the sliders
label = tk.Label(root, text="Adjust the target angles using the sliders", font=("Helvetica", 16))
label.pack(pady=10)

# Start the custom main loop in a separate thread
thread = threading.Thread(target=custom_main_loop, daemon=True)
thread.start()

# Run the Tkinter event loop
root.mainloop()

# Close the serial connection when the GUI is closed
ser.close()