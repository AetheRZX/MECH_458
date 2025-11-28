import serial
import time
import csv
import threading
import sys
import os

# //// CONFIGURATION
SERIAL_PORT = 'COM6'  # Check your port!
BAUD_RATE = 115200
LOG_FOLDER = "motor_logs" 

# //// SETUP
print("--- Motor Logger Setup ---")

# 1. Get current working directory (Where this script is)
current_dir = os.getcwd()
folder_path = os.path.join(current_dir, LOG_FOLDER)

# 2. Create Folder
if not os.path.exists(folder_path):
    os.makedirs(folder_path)

# 3. Get Filename
filename_input = input("Enter filename for log (e.g. 'test1'): ")
if not filename_input.endswith(".csv"):
    filename_input += ".csv"

# 4. Create full path
full_filepath = os.path.join(folder_path, filename_input)

print("\n" + "="*50)
print(f"FILE WILL BE SAVED HERE:\n{full_filepath}")
print("="*50 + "\n")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error connecting to serial: {e}")
    sys.exit()

stop_event = threading.Event()

# //// READING THREAD
def read_from_serial():
    with open(full_filepath, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time_ms", "Target", "M1_Pos", "M2_Pos", "M1_RPM", "M2_RPM", "PWM1", "PWM2"])
        
        print("Logging started. Type 'stop' to save and quit.")
        
        while not stop_event.is_set():
            try:
                if ser.in_waiting > 0:
                    raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "," in raw_line:
                        data = raw_line.split(',')
                        if len(data) == 8: 
                            writer.writerow(data)
                    else:
                        if raw_line:
                            print(f"Arduino: {raw_line}")
            except Exception as e:
                print(f"Error: {e}")
                break
    print(f"Log file saved.")

# //// WRITING THREAD
def write_to_serial():
    print("\nControls: q/a (M1), w/s (M2), z (Zero), [Number] (Move)")
    
    while not stop_event.is_set():
        user_input = input() 
        if user_input.lower() in ["stop", "exit"]:
            stop_event.set()
            print("Stopping...")
            break
        if user_input:
            ser.write((user_input + '\n').encode('utf-8'))

# //// MAIN
if __name__ == "__main__":
    t1 = threading.Thread(target=read_from_serial, daemon=True)
    t1.start()
    try:
        write_to_serial()
    except KeyboardInterrupt:
        stop_event.set()
    ser.close()
    sys.exit()