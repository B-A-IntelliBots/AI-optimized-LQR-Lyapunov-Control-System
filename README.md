<div align="center">
  
# AI-optimized-LQR-Lyapunov-Control-System
This project aims to design, model, simulate and control a camera-equipped differential robotic platform designated for environment mapping purposes. The robot autonomously navigates the environment through a prebuilt discretized path involving a halt at each specified point on the path to capture images for the environment.
</div>

# 📖 Overview & Methodology
The project's workflow can be divided as follows:
- **Vehicle Modeling**
  
First the kinematic and dynamic models of the robot has been extracted. In this section we assumed  pure motion conditions, i.e., the robot does not encounter and slip, slide, or bounce.
- **Control objective formulation**
  
This robotic platform is designated for environmental mapping purposes "3D reconstruction". For that sake, the control system should be designed to acheive the objectives related to this task. These objectives are broken down as follow
  1. Trajectory adherence objective:
The robot path between two consequtive reference points should converge to the straight line connecting these two points.
  2. Arrival heading angle objective:
The arrival heading angle of the robot ubon reaching the target point should be as close to the inclination angle of the straight line connecting the starting and target point. This guarantees smooth motion over consequtive target points.
  3. Elapsid time:
The time taken to traverse the path between two target points should be minimal.
<div align="center">
  
**"it is worth noting that these three objectives are formulated into one performance metric error facilitaing the choice of control gains later"**
</div>

- **Control system design**
  
The control system, that drives the state of the robot to the desired state, has been divided into two loops:
  1. High level control loop:
This loop is concerned about generating the linear and angular reference velocity profiles that guarantees carrying the robot fron the starting position to the target position. The high level control loop incorporates Lyapunov stability criterion to set stable velocity profiles using postion and orienation errors with control gains. In addtion, this loop utilizes an MLP neural network that adaptively choose the best high level control gains that yeilds the path satisfying the control objectives outlined previously.
  2. Low level control loop:
This loop is concerned about governing the dynamic system to follow the reference velocity profiles as fast as possible. It incorporate LQR method with full order observer.
- **Neural Network Training Process**
As stated previously, the neural network is responisble for selecting the optimal high level control gains that achieve a tade off between the three control objectives yeilding the best path to follow. 
you can find the codes for gathering the dataset in [training_data_generation](https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System/tree/main/training_data_generation)
then the neural network was trained using python within Google Colab, using the architechture shown below.
you can find the training code in [MLP_training](https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System/tree/main/MLP_training)
- **MATLAB Simulation**
After training the neural network, the model was used to control the robot through various types of paths in MATLAB to validate the control scheme.
figures below showes some of the results on a single target point and circular and infinity-shaped trajectories.
- **ROS and Gazebo simulation**
The control algorithm detailed above was transferred to ROS environment in order to run it on Raspberry Pi board and also conduct simulations in Gazebo. It is noteworthy that an arduino board is used as a bridge between (motors, sensors) and Raspberry pi running ROS).Arduino is treated as a node with in the ROS environment via serial communication.
ROS nodes can be found in [src](https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System/tree/main/DDWMR/src)
- **Gazebo Simulation**
Before real-world implementaion of the robot, a thorough simulation is coducted via Gazebo to test the design and control scheme on physical conditions. the simulated robot and environment are shown in the following figures. 

## 🚀 Quick Start
### Prerequisites
- OS/Tooling: `Python ≥3.10`
### Installation and Usage

+ #### Clone the repository:
  `$ git clone https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System.git`
+ set the DDWMR package inside your workspace, install rosserial-noetic-devel inside your workspace that is responsible for serial communication then run "catkin_make"
+ make sure that the arduino is connected to your raspberry pi, then run the rosserial_node to establish the connection
+ run the command "roslaunch DDWMR start.launch" 
## 📊 Results & Benchmarks 
- The proposed neural network-based controller surpasses the classical control methods such as (PID, fixed_gains Lyapunov) in solving stabilization task.
- The neural network acheives a better trade off between all control objectives proposed for this project.
- The incorporation of LQR method in the low level control loop helped prioritizig energy efficiency.
  

## 🛠️ Technologies Used

Programming: MATLAB, Python (ROS integration)
Simulation: MATLAB/Simulink, ROS (Robot Operating System), Gazebo
## 📂 Project Structure
```
├ DDWMR (main project package)
│   ├── src (contains all node files)
│   ├── srv (contains services file)
│   ├── launch (contains launch file to run the project)
│   ├── CMakelists.txt
│   ├── package.xml
├ MLP_training (contains training data and training code)
├ arduino (contains the arduino node code)
├ training_data_generation ( codes used to generate the training data in MATLAB)
├ README.md (this file)
```

## 📖 Citation
@software{myproject2025,
  author    = {Ali Deeb, Bisher Alsaleh}, Supervised by: {Prof. Iyad Hatem}
  title     = {AI-optimized-LQR-Lyapunov-Control-System},
  year      = {2025},
  publisher = {GitHub},
  url       = {https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System}
}
