#!/usr/bin/env python3
import sys
import os
import matplotlib
# Force Agg backend to run without X11 display
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import rosbag

# 1. Path Handling
bag_file = "control_result.bag"
if len(sys.argv) > 1:
    bag_file = sys.argv[1]

if not os.path.exists(bag_file):
    print(f"Error: {bag_file} not found.")
    sys.exit(1)

print(f"Processing {bag_file}...")

# 2. Data Extraction
t_traj = []
x_ref, y_ref = [], []
x_P, y_P = [], []

t_cmd = []
v_cmd, w_cmd = [], []

try:
    bag = rosbag.Bag(bag_file)
    
    # Read Trajectory
    for topic, msg, t in bag.read_messages(topics=['/trajectory']):
        if len(msg.data) >= 5:
            t_traj.append(msg.data[0])
            x_ref.append(msg.data[1])
            y_ref.append(msg.data[2])
            x_P.append(msg.data[3])
            y_P.append(msg.data[4])
        
    # Read Commands
    for topic, msg, t in bag.read_messages(topics=['/cmd']):
        if len(msg.data) >= 3:
            t_cmd.append(msg.data[0])
            v_cmd.append(msg.data[1])
            w_cmd.append(msg.data[2])
        
    bag.close()
except Exception as e:
    print(f"Error reading bag: {e}")
    sys.exit(1)

if len(t_traj) == 0:
    print("Error: no /trajectory samples found in the bag.")
    sys.exit(1)

# --- 3. Time Normalization ---
if len(t_traj) > 0:
    start_time = t_traj[0]
    t_traj = [t - start_time for t in t_traj]
    
    if len(t_cmd) > 0:
        t_cmd = [t - start_time for t in t_cmd]

# 4. Calculate Errors and Maximums
errors_x = [r - p for r, p in zip(x_ref, x_P)]
errors_y = [r - p for r, p in zip(y_ref, y_P)]

# Compute absolute maximums
max_err_x = max([abs(e) for e in errors_x])
max_err_y = max([abs(e) for e in errors_y])

# Print to console for easy copying into text report
print(f"=== RESULTS ===")
print(f"Max Error X: {max_err_x:.5f} m")
print(f"Max Error Y: {max_err_y:.5f} m")

# --- Plot (a): XY Trajectory ---
print("Generating Figure (a)...")
plt.figure(figsize=(8, 8))
plt.plot(x_ref, y_ref, 'r--', label='Reference', linewidth=2)
plt.plot(x_P, y_P, 'b', label='Robot Path (P)', linewidth=1.5)
plt.title('Figure (a): Reference vs Actual Trajectory')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('figure_a_trajectory.png')

# --- Plot (b): Tracking Errors ---
print("Generating Figure (b)...")
plt.figure(figsize=(10, 6))

# Plot lines
plt.plot(t_traj, errors_x, label=f'Error X')
plt.plot(t_traj, errors_y, label=f'Error Y')

# Add Max Error text clearly in the title or as a text box
title_text = f"Figure (b): Tracking Errors\nMax |ex| = {max_err_x:.4f} m, Max |ey| = {max_err_y:.4f} m"
plt.title(title_text)

plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.legend()
plt.grid(True)
plt.savefig('figure_b_errors.png')

# --- Plot (c): Control Signals ---
print("Generating Figure (c)...")
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(t_cmd, v_cmd, 'g')
plt.title('Figure (c): Control Signals')
plt.ylabel('Linear Velocity v [m/s]')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t_cmd, w_cmd, 'm')
plt.ylabel('Angular Velocity w [rad/s]')
plt.xlabel('Time [s]')
plt.grid(True)

plt.savefig('figure_c_controls.png')

print("All figures saved to current directory.")
