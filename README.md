# AI-optimized-LQR-Lyapunov-Control-System

This project aims to design, model, simulate and control a camera-equipped robotic platform designated for environment mapping purposes. The robot should autonomously navigate the environment through a prebuilt discretized path involving a halt at each specified point on the path to capture images for the environment.
Afterdefining the project requirements and the primary purpose of the mobile platform, adifferential-drive wheeled mobile robot was selected due to its simplicity in design and control, as well as its effectiveness in achieving the intended objectives. 
The project's workflow can be divided as follows:
## Vehicle Modeling
First the kinematic and dynamic models of the robot has been extracted. In this section we assumed  pure motion conditions, i.e., the robot does not encounter and slip, slide, or bounce.
## Control objective formulation
This robotic platform is designated for environmental mapping purposes "3D reconstruction". For that sake, the control system should be designed to acheive the objectives related to this task. These objectives are broken down as follow
### 1. Trajectory adherence objective
The robot path between two consequtive reference points should converge to the straight line connecting these two points.
### 2. Arrival heading angle objective
The arrival heading angle of the robot ubon reaching the target point should be as close to the inclination angle of the straight line connecting the starting and target point. This guarantees smooth motion over consequtive target points.
### 3. Elapsid time
The time taken to traverse the path between two target points should be minimal.
"it is worth noting that these three objectives are formulated into one performance metric error facilitaing the choice of control gains later"
## Control system design
The control system, that drives the state of the robot to the desired state, has been divided into two loops:
### 1. High level control loop
This loop is concerned about generating the linear and angular reference velocity profiles that guarantees carrying the robot fron the starting position to the target position. The high level control loop incorporates Lyapunov stability criterion to set stable velocity profiles using postion and orienation errors with control gains. In addtion, this loop utilizes an MLP neural network that adaptively choose the best high level control gains that yeilds the path satisfying the control objectives outlined previusly.
### 2. Low level control loop
This loop is concerned about governing the dynamic system to follow the reference velocity profiles as fast as possible. It incorporate LQR method with full order observer.
## Neural Network Training Process
As stated previously, the neural network is responisble for selecting the optimal high level control gains that achieve a tade off between the three control objectives yeilding the best path to follow. this network takes the target (x,y) postion as input and outputs (Kp,Kth) control gains that determine how the path should look like toward the target point starting from the current point. 
first of all, the training data is gathered usingn MATLAB. This done by spaning set of target points within the circle arround the robot with diameter of 1 meter, then for each target point , the error metric formulated from the control objectives is calcuated for all possible control gains (after setting lower and upper limits for each gain), then the control gains that yeilds the smallest error is saved as a ground truth for training the neural network.
you can find the codes for gathering the dataset in ********
then the neural network was trained using python within Google Colab, using the architechture shown below.
you can find the training code in ********
## MATLAB Simulation
After training the neural network, the model was used to control the robot through various types of paths in MATLAB to validate the control scheme.
You can find the simuation codes in *******
figures below showes some of the results on a single target point and circular and infinity-shaped trajectories.
## ROS and Gazebo simulation
The control algorithm detailed above was transferred to ROS environment in order to run it on Raspberry Pi board and also conduct simulations in Gazebo. It is noteworthy that an arduino board is used as a bridge between (motors, sensors) and Raspberry pi running ROS).Arduino is treated as a node with in the ROS environment via serial communication.
ROS nodes can be found in ***** and are organized as follows:
main_script.py---> defines the trajectory that the robot should follow, and for each reference point it calls the control loop server till it reaches the point with a specified tolerence. It then calls the capturing server to capture images of the environment for mapping purposes.
control_loop.py---> This node implements one step on of path-following controller for a differential-drive robot, incorportaing  LQR-control with the usgae of full state observer to estimate the unmeasured robot state (here the motors' currents) using the measured states (here wheels' velocities usinf Encoders).
It subscribes to robot pose and velocity published by arduino node, adapts control gains based on the neural network service, and publishes motor PWM signals to arduino for accurate trajectory tracking.
NN_server.py---> this node is a service server node that is responsible for taking target (x,y) as inputs and returns the optimal control gains (Kp,Kth) predicted by the trained network in MLP_net.h5 file.
capturing_server.py---> this node is a service servier node that is responsible for sweeping the camera atop the robot and capturing images of the environment then sending these images online to a Telegram bot for later usgae in 3D reconstruction.
arduino_node.py--->This Arduino node interfaces with encoders, an MPU6050 IMU, and motor drivers to estimate odometry and control wheel actuation.It publishes the robot’s pose and filtered wheel velocities to ROS, while subscribing to PWM motor commands and servo image-capture commands.
## Gazebo Simulation
Before real-world implementaion of the robot, a thorough simulation is coducted via Gazebo to test the design and control scheme on physical conditions. the simulated robot and environment are shown in the following figures. 






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
