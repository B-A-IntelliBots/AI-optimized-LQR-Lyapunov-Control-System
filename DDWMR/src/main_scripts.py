'''
Trajectory Controller Node

This node defines a sequence of target waypoints for the robot to follow.  
For each waypoint, it:
  1. Calls the control service ("next_step") to iteratively move the robot until the target is reached within tolerance.
  2. Stops the robot and calls the capturing service ("capture_images") to record images for mapping.  

Finally, it plots the executed path for visualization.
'''

#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from std_msgs.msg import Int64
from control.srv import next_step, capture

# =======================
# Initialization
# =======================
rospy.init_node("Control")
rospy.loginfo("✅ Control node started")

# Simulation timing
dt = 0.001          # time step for ROS sleep
total_time = 50      # maximum allowed runtime
time_vector = np.linspace(0, total_time, int(total_time / dt) + 1)

# Target positions (reference trajectory)
x_targets = [0.8]
y_targets = [0.0]

# Robot state history (initialized at origin)
x_positions = [0]
y_positions = [0]
orientations = [0]

# Publishers
pwm_publisher = rospy.Publisher("/motor_pwm", Point, queue_size=10)
image_publisher = rospy.Publisher("/image", Int64, queue_size=10)

# =======================
# Main trajectory loop
# =======================
for i in range(len(x_targets)):

    # --- First call: initialize path planning & compute control gains ---
    rospy.wait_for_service("next_step")
    path_service = rospy.ServiceProxy("next_step", next_step)

    response = path_service(
        x_targets[i], y_targets[i],
        x_positions[-1], y_positions[-1], orientations[-1],
        True,  # flag to recalculate control gains
        0
    )

    # Update state history
    x_positions.append(response.nx)
    y_positions.append(response.ny)
    orientations.append(response.nth)
    distance_remaining = response.df

    # --- Iteratively move towards the target until tolerance is satisfied ---
    while distance_remaining > 0.15:  # tolerance radius
        rospy.wait_for_service("next_step")
        response = path_service(
            x_targets[i], y_targets[i],
            x_positions[-1], y_positions[-1], orientations[-1],
            False,  # continue with current gains
            0
        )
        x_positions.append(response.nx)
        y_positions.append(response.ny)
        orientations.append(response.nth)
        distance_remaining = response.df
        rospy.sleep(dt)  # allow system to update

    # --- Stop the robot before capturing images ---
    stop_msg = Point(x=0, y=0)
    for _ in range(3):  # publish multiple times for safety
        pwm_publisher.publish(stop_msg)
        rospy.sleep(0.01)

    # --- Capture mapping images at multiple angles ---
    rospy.wait_for_service("capture_images")
    image_service = rospy.ServiceProxy("capture_images", capture)

    for j in range(19):  # capture at 0°, 10°, ..., 180°
        image_publisher.publish(j * 10)  # publish angle
        rospy.sleep(1)
        image_service(1)  # trigger capture

    image_publisher.publish(90)  # final position at 90°

# =======================
# Shutdown & Visualization
# =======================
rospy.loginfo("🏁 Trajectory completed. Stopping robot.")

# Final stop command
stop_msg = Point(x=0, y=0)
pwm_publisher.publish(stop_msg)

# Plot executed trajectory
plt.plot(x_positions, y_positions, marker='o')
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Robot Executed Path")
plt.grid(True)
plt.axis("equal")
plt.show()

