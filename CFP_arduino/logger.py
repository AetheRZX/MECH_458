# pip install pyserial

import serial
import time
import csv
import threading

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'  # Change this to your Arduino Port (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Mac/Linux)
BAUD_RATE = 115200
FILENAME = "motor_data.csv"

# Connect to Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error connecting to serial: {e}")
    exit()

# Function to handle reading data and saving to CSV
def read_from_serial():
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write Header
        writer.writerow(["Time_ms", "Target", "M1_Pos", "M2_Pos", "PWM1", "PWM2"])
        
        print("Logging started... (Press Ctrl+C to stop)")
        
        while True:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    
                    # Check if the line looks like CSV data (numbers and commas)
                    if "," in line:
                        data = line.split(',')
                        if len(data) == 6: # We expect 6 columns
                            writer.writerow(data)
                            # Optional: Print to console so you see it's working
                            print(f"Log: {data}")
                    else:
                        # Print other messages (like "Jog done")
                        print(f"Arduino: {line}")
                        
            except Exception as e:
                print(f"Error reading: {e}")
                break

# Function to handle User Input (Sending commands to Arduino)
def write_to_serial():
    while True:
        user_input = input() # Wait for user to type
        ser.write((user_input + '\n').encode('utf-8'))

# Run threads
if __name__ == "__main__":
    # Start the reading thread (Daemon means it dies when main program dies)
    t1 = threading.Thread(target=read_from_serial, daemon=True)
    t1.start()

    # Run writing in the main thread
    try:
        write_to_serial()
    except KeyboardInterrupt:
        print("\nStopping...")
        ser.close()