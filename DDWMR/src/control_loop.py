'''
Path-Following Controller Node for a Differential-Drive Robot

This node implements one step of an LQR-based path-following controller 
with a full-state observer to estimate unmeasured robot states 
(e.g., motor currents) using measured wheel velocities.  

Workflow:
1. Subscribes to:
   - Robot pose (x, y, θ) from Arduino node
   - Wheel velocities from encoders
2. On service call:
   - Computes control gains (Kp, Kth) using a neural network service ("gains") if requested
   - Calculates desired wheel velocities and errors
   - Updates the observer and LQR controller
   - Publishes PWM commands to motors
3. Returns updated robot state to the "next_step" service caller.
'''

#!/usr/bin/env python3
import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from control.srv import next_step, next_stepResponse, gains

# =======================
# Global State Variables
# =======================
current_pos_x = 0.0
current_pos_y = 0.0
current_theta = 0.0
actual_velocity = np.array([[0.0], [0.0]])  # [v_right, v_left]

# Publisher for motor PWM
pwm_pub = rospy.Publisher("/motor_pwm", Point, queue_size=10)

# =======================
# Robot Parameters
# =======================
dt = 0.001
WHEEL_BASE = 0.181     # distance between wheels [m]
WHEEL_RADIUS = 0.0335  # wheel radius [m]

# =======================
# Control & Observer Matrices
# =======================
# Initial NN-based proportional gains (will be updated dynamically)
Kp = 0.2
Kth = 3.3

# LQR gains (2x6 matrix for state feedback)
K = np.array([
    [0.0638, 0.0129, 0.0087, -0.0002, -1.0, 0.0],
    [0.0129, 0.0638, -0.0002, 0.0087, 0.0, -1.0]
])

# Observer gain matrix
Ke = np.array([
    [0.0419, 0.0062],
    [0.0062, 0.0419],
    [-0.0023, -0.0005],
    [-0.0005, -0.0023]
])

# System matrices (linearized model of motor + dynamics)
A = np.array([
    [-0.9099, 0.2041, 210.134, -47.1424],
    [0.2041, -0.9099, -47.1424, 210.134],
    [-100.4333, 0.0, -1537.52, 0.0],
    [0.0, -100.4333, 0.0, -1537.52]
])
B = np.array([
    [0.0, 0.0],
    [0.0, 0.0],
    [331.74, 0.0],
    [0.0, 331.74]
])
C = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])

# =======================
# Observer State Variables
# =======================
observer_state = np.zeros((6, 1))       # extended observer states
observer_derivative = np.zeros((6, 1))  # derivative for integration
integrated_error_right = 0.0
integrated_error_left = 0.0
update_counter = 0

# =======================
# Callbacks
# =======================
def pose_callback(msg):
    """ Update robot pose (x, y, θ). """
    global current_pos_x, current_pos_y, current_theta
    current_pos_x = msg.x
    current_pos_y = msg.y
    current_theta = msg.z


def velocity_callback(msg):
    """ Update actual wheel velocities from encoders. """
    global actual_velocity
    actual_velocity[0, 0] = msg.x  # right wheel velocity
    actual_velocity[1, 0] = msg.y  # left wheel velocity


# =======================
# Service: Path Following Step
# =======================
def follow_path_callback(req):
    """
    Compute one control step:
    - Calculate distance & heading error to goal
    - Update control gains via NN service (if requested)
    - Run LQR + observer to compute PWM commands
    - Return updated robot state
    """
    global update_counter, observer_state, Kp, Kth
    global integrated_error_left, integrated_error_right

    # Skip if imaging flag is set
    if req.imaging != 0:
        return

    # --- Compute distance & heading error ---
    dx = req.x_start - req.xx0
    dy = req.y_start - req.yy0
    distance = math.hypot(dx, dy)
    heading_error = math.atan2(dy, dx) - req.th0
    # Normalize angle to [-π, π]
    heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

    # --- Update gains if requested ---
    if req.flagg:
        x_local = distance * math.cos(heading_error)
        y_local = distance * math.sin(heading_error)
        rospy.wait_for_service("gains")
        gain_service = rospy.ServiceProxy("gains", gains)
        gain_response = gain_service(x_local, y_local)
        Kp = gain_response.Kp
        Kth = gain_response.Kth

    # --- Compute control every 10 updates (downsampling) ---
    if update_counter % 10 == 0:
        linear_velocity = Kp * distance * math.cos(heading_error)
        angular_velocity = Kth * heading_error + Kp * math.cos(heading_error) * math.sin(heading_error)
        update_counter = 0  # reset counter

        # Desired wheel velocities
        v_left = (linear_velocity - (WHEEL_BASE / 2) * angular_velocity) / WHEEL_RADIUS
        v_right = (linear_velocity + (WHEEL_BASE / 2) * angular_velocity) / WHEEL_RADIUS

        # Integration of velocity error
        integrated_error_right += dt * (v_right - actual_velocity[0, 0])
        integrated_error_left += dt * (v_left - actual_velocity[1, 0])

        # --- LQR control input ---
        control_input = (
            -K[:, :4] @ observer_state[0:4]
            -K[:, 4:] @ np.array([[integrated_error_right], [integrated_error_left]])
        )

        # --- Observer update (estimate motor currents) ---
        observer_derivative = (
            (A - Ke @ C) @ observer_state[0:4]
            + B @ control_input
            + Ke @ actual_velocity
        )
        observer_state[0:4] += observer_derivative * dt

        # --- Map control input to PWM ---
        pwm_right = int(control_input[0, 0] / 12 * 255)
        pwm_left = int(control_input[1, 0] / 12 * 255)

        # Publish PWM
        pwm_msg = Point(x=pwm_right, y=pwm_left)
        pwm_pub.publish(pwm_msg)

    # Increment update counter
    update_counter += 1

    # Normalize theta
    current_theta = (current_theta + np.pi) % (2 * np.pi) - np.pi

    return next_stepResponse(current_pos_x, current_pos_y, current_theta, distance)


# =======================
# Main
# =======================
if __name__ == "__main__":
    rospy.init_node("follow_path")
    rospy.loginfo("✅ Follow Path node started")

    # Subscribers
    rospy.Subscriber("/robot_pos", Point, pose_callback)
    rospy.Subscriber("/velocities", Point, velocity_callback)

    # Service
    rospy.Service("next_step", next_step, follow_path_callback)

    rospy.spin()

