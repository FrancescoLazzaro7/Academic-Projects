#!/usr/bin/env python3
import sys
import os
import matplotlib
# Force non-interactive backend (Must be before importing pyplot)
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import rosbag

# 1. Robust Path handling
bag_file = "test_result.bag"
if len(sys.argv) > 1:
    bag_file = sys.argv[1]

print(f"Opening bag file: {bag_file}")

if not os.path.exists(bag_file):
    print(f"Error: File '{bag_file}' not found.")
    sys.exit(1)

# 2. Read Data
time_vec = []
v_actual_vec = []

try:
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=['/state']):
        if len(msg.data) >= 5:
            time_vec.append(msg.data[0])
            v_actual_vec.append(msg.data[4]) # Assuming v_actual is at index 4
    bag.close()
except Exception as e:
    print(f"Error reading bag: {e}")
    sys.exit(1)

if len(time_vec) == 0:
    print("Error: no /state samples with v_actual found in the bag.")
    sys.exit(1)

# 3. Plot and Save
print("Generating plot...")
plt.figure(figsize=(10, 6))
plt.plot(time_vec, v_actual_vec, label='v_actual', linewidth=2)
plt.axhline(y=0.632, color='r', linestyle='--', label='Target (63.2%)') 
plt.title('Step Response Verification')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.grid(True)
plt.legend()

# Save instead of show
output_filename = "step_response.png"
plt.savefig(output_filename)
print(f"Plot saved to: {os.path.abspath(output_filename)}")
