# AI-optimized-LQR-Lyapunov-Control-System

This project focuses on 3D reconstruction of indoor environments (e.g., rooms, stores, museums) using a differential-drive mobile robot equipped with a monocular camera.
The robot autonomously navigates along a predefined path, pauses to capture images, and reconstructs a 3D model of the environment.

The system integrates kinematic & dynamic modeling, hybrid control strategies (LQR, Lyapunov-based, and Neural Networks), and simulation frameworks such as MATLAB/Simulink, CATIA, and ROS with Gazebo.

📌 Features

Autonomous navigation and path tracking.

Differential-drive mobile robot design and simulation.

Hybrid control strategies:

Linear Quadratic Regulator (LQR) with full-state observer.

Lyapunov stability-based controllers.

Artificial Neural Networks for hyperparameter tuning.

MATLAB Simulink modeling of robot dynamics & control.

CATIA-based robot CAD design.

ROS + Gazebo simulation with Python implementation.

Image capture pipeline for 3D reconstruction.

🛠️ Technologies Used

Programming: MATLAB, Python (ROS integration)

Simulation: MATLAB/Simulink, ROS (Robot Operating System), Gazebo

CAD Design: CATIA

Control & AI: LQR, Lyapunov stability criterion, Neural Networks (MLP)

📂 Project Structure
├── docs/                   # Project documentation & reports
├── models/                 # CATIA CAD models of the robot
├── matlab/                 # MATLAB/Simulink models
├── ros_ws/                 # ROS workspace with Python scripts
├── gazebo/                 # Gazebo simulation world & configs
└── README.md               # Project README file

🚀 Getting Started
1. Prerequisites

MATLAB with Simulink

Python 3.x

ROS (Noetic / ROS2 compatible)

Gazebo simulator

CATIA (for CAD visualization)

2. Installation

Clone the repository:

git clone https://github.com/yourusername/3d-reconstruction-robot.git
cd 3d-reconstruction-robot


Install Python dependencies:

pip install -r requirements.txt

▶️ Usage
Running MATLAB Simulation
open('matlab/simulink_model.slx')
sim('simulink_model')

Running Gazebo with ROS
cd ros_ws
catkin_make
source devel/setup.bash
roslaunch robot_simulation gazebo.launch

Running Control Node (Python)
rosrun control path_tracking.py

📊 Results

Successful real-time simulation of robot dynamics.

Path-tracking with minimized error using LQR + Observer.

Neural networks improved hyperparameter tuning for control gains.

Robot design validated in Gazebo with ROS-based execution.

Images collected for 3D environment reconstruction.

📖 References

Kalman, R. E. Linear Quadratic Regulators.

Lyapunov, A. Stability Theory.

Related works on differential drive robots and hybrid control.

👨‍💻 Authors

Bisher Alsaleh

Ali Deeb

Supervised by Prof. Iyad Hatem

📜 License

This project is licensed under the MIT License
.
