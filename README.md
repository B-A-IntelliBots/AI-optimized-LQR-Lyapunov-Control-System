# AI-optimized-LQR-Lyapunov-Control-System

This project aims to design, model, simulate and control a camera-equipped robotic platform designated for environment mapping purposes. The robot should autonomously navigate the environment through a prebuilt discretized path involving a halt at each specified point on the path to capture images for the environment.
Afterdefining the project requirements and the primary purpose of the mobile platform, adifferential-drive wheeled mobile robot was selected due to its simplicity in design and control, as well as its effectiveness in achieving the intended objectives. 
The project workflow can be divided as follows:
## Vehicle Modeling
First the kinematic and dynamic models of the robot has been extracted. In this section we assumed  pure motion conditions, i.e., the robot does not encounter and slip, slide, or bounce.
## Control objective formulation
This robotic platform is designated for environmental mapping purposes "3D reconstruction". For that sake, the control system should be designed to acheive the objectives related to this task. These objectives are broken down as follow
### Trajectory adherence objective
The robot path between two consequtive reference points should converge to the straight line connecting these two points.
### Arrival heading angle objective
The arrival heading angle of the robot ubon reaching the target point should be as close to the inclination angle of the straight line connecting the starting and target point. This guarantees smooth motion over consequtive target points.
### Elapsid time
The time taken to traverse the path between two target points should be minimal.




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
