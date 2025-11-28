import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# //// CONFIGURATION
LOG_FOLDER = "motor_logs"

# 1. Ask user which file to open
print(f"Looking in folder: {os.path.join(os.getcwd(), LOG_FOLDER)}")
filename = input("Enter filename to plot (e.g. test1): ")
if not filename.endswith(".csv"):
    filename += ".csv"

filepath = os.path.join(LOG_FOLDER, filename)

if not os.path.exists(filepath):
    print("File not found!")
    sys.exit()

# 2. Load Data
try:
    df = pd.read_csv(filepath)
except Exception as e:
    print(f"Error reading file: {e}")
    sys.exit()

# 3. Plotting
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Plot 1: Positions (Are they synced?)
ax1.set_title(f"Synchronization Analysis: {filename}")
ax1.plot(df['Time_ms'], df['M1_Pos'], label='M1 (Master)', color='blue')
ax1.plot(df['Time_ms'], df['M2_Pos'], label='M2 (Slave)', color='red', linestyle='--')
ax1.set_ylabel("Position (Pulses)")
ax1.legend()
ax1.grid(True)

# Plot 2: Speed (Is it stable?)
ax2.plot(df['Time_ms'], df['M1_RPM'], label='M1 RPM', color='blue')
ax2.plot(df['Time_ms'], df['M2_RPM'], label='M2 RPM', color='red', alpha=0.7)
ax2.set_ylabel("Speed (RPM)")
ax2.legend()
ax2.grid(True)

# Plot 3: PWM Output (Is the PID fighting?)
ax3.plot(df['Time_ms'], df['PWM1'], label='M1 PWM', color='blue')
ax3.plot(df['Time_ms'], df['PWM2'], label='M2 PWM', color='red')
ax3.set_ylabel("PWM (0-255)")
ax3.set_xlabel("Time (ms)")
ax3.legend()
ax3.grid(True)

plt.show()