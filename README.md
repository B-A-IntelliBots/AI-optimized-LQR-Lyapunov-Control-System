<div align="center">

![MATLAB](https://img.shields.io/badge/v2014a-blue?style=plastic&label=MATLAB&labelColor=black)
![Python](https://img.shields.io/badge/v3.11-green?style=plastic&logo=python&label=Python3&labelColor=black)
![ROS](https://img.shields.io/badge/noetic-green?style=plastic&logo=ros&label=ROS&labelColor=black)
![Arduino](https://img.shields.io/badge/v2.3.6-green?style=plastic&logo=arduino&label=Arduino&labelColor=black)

# AI-Optimized LQR-Lyapunov Control System

This project presents the design, modeling, simulation, and control of a camera-equipped differential-drive robotic platform for **environment mapping and 3D reconstruction**.  
The robot autonomously navigates a discretized path, stopping at specific points to capture images for mapping.

<img width="300" height="300" alt="Discretized Path Tracking" src="https://github.com/user-attachments/assets/437ce3ed-46cc-4fcd-a637-b37ed2ebe77c" />

</div>

---

# 📖 Overview & Methodology

### 1. Vehicle Modeling
The kinematic and dynamic models of the robot were derived under **ideal motion conditions** (no slip, slide, or bounce).

### 2. Control Objectives
The control system was designed to meet the following objectives:
1. **Trajectory adherence**  
   The path between consecutive reference points should converge to the straight line connecting them.
2. **Arrival heading angle**  
   Upon reaching a target point, the robot’s heading should align with the inclination angle of the line connecting start and target points, ensuring smooth transitions.
3. **Elapsed time**  
   The traversal time between two target points should be minimal.

<div align="center">

*These objectives are combined into a single performance error metric, which facilitates optimal control gain selection.*

</div>

### 3. Control System Design
The controller was structured into two loops:

- **High-level control loop**  
  Generates linear and angular reference velocity profiles that guide the robot to the target.  
  - Uses Lyapunov stability criterion for stable velocity profiles (based on position/orientation errors).  
  - Incorporates an **MLP neural network** to adaptively choose optimal control gains.  

- **Low-level control loop**  
  Ensures the dynamic system tracks the reference velocities quickly.  
  - Uses **LQR method** with a full-order observer.  

### 4. Neural Network Training
The MLP neural network selects optimal high-level control gains that balance the three objectives.  

- Training dataset generation: [training_data_generation](https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System/tree/main/training_data_generation)  
- Training code: [MLP_training](https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System/tree/main/MLP_training)  

<div align="center">

<img width="400" height="300" alt="Neural Network Architecture" src="https://github.com/user-attachments/assets/fba1e442-bc7e-4196-b9a9-75da92416a82" />

</div>

### 5. MATLAB Simulation
The trained neural network was integrated into MATLAB simulations to evaluate performance on various paths.  

The table below shows:
- **Row 1**: Neural network-based controller (PNN) outperforming Lyapunov and PID controllers.  
- **Row 2**: PNN performance on discretized Line, Circle, and Infinity-shaped trajectories.  

<div align="center">
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/6d3335b8-d84e-4205-b8ea-6548814e42f6" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/52a6c05c-c784-4e03-8c66-b2f9bd3c0702" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/6401a9af-a8e9-4320-baf4-ef5d41fa9d2b" width="250"/></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/6fac5055-ba62-4be5-ab09-dd86ecb1435e" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/76f97641-03dd-48f0-a354-a2045fa198ad" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/da807151-d690-4c0d-9b6c-abf44ca3762a" width="250"/></td>
  </tr>
</table>
</div>

### 6. ROS and Gazebo Simulation
The control algorithm was ported to ROS for execution on a **Raspberry Pi**, with an **Arduino** acting as an interface for motors and sensors (via serial communication).  

- ROS nodes: [src](https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System/tree/main/DDWMR/src)  

#### Gazebo Simulation
Before real-world deployment, the robot was tested in Gazebo under realistic conditions:

<div align="center">
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/3716c441-91a2-4d59-bb3a-ddb8ae93d107" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/6ec03a10-f68e-47d6-b6be-fdb64476c3c6" width="250"/></td>
    <td><img src="https://github.com/user-attachments/assets/55403cbf-e00e-412a-a08e-24dbadfb5e6b" width="250"/></td>
  </tr>
</table>
</div>
---

## 🎥 Demo Video


## 🚀 Quick Start

### Prerequisites
- OS/Tooling: `Python ≥ 3.10`, `ROS Noetic`, `MATLAB 2014a+`

### Installation & Usage
```bash
# Clone repository
git clone https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System.git

# Build ROS workspace
cd ~/catkin_ws/src
ln -s /path/to/AI-optimized-LQR-Lyapunov-Control-System/DDWMR .
cd ~/catkin_ws
catkin_make
# Make sure that the serial communication betweeb Arduino and Raspbery pi is established via rosserial-noetic-devel package, then run the following command
roslaunch DDWMR start.launch


```

## 📊 Results & Benchmarks
 
- The proposed neural network-based controller surpasses the classical control methods such as (PID, fixed_gains Lyapunov) in solving stabilization task.
- The neural network achieves a better trade-off between all control objectives proposed for this project.
- The incorporation of LQR method in the low level control loop helped prioritizing energy efficiency.
  

## 🛠️ Technologies Used

- Programming: MATLAB, Python (with ROS integration)

- Simulation: MATLAB/Simulink, ROS, Gazebo

- Hardware: Raspberry Pi, Arduino

## 📂 Project Structure
```
├ DDWMR (main ROS package)
│   ├── src          # ROS node files
│   ├── srv          # Service definitions
│   ├── launch       # Launch files
│   ├── CMakeLists.txt
│   ├── package.xml
├ MLP_training       # Training data & code
├ arduino            # Arduino node code
├ training_data_generation  # MATLAB scripts for dataset generation
├ README.md

```

## 📖 Citation

```
@software{myproject2025,
  author    = {Ali Deeb and Bisher Alsaleh},
  title     = {Autonomous 3D Mapping: AI-Optimized LQR-Lyapunov Control System},
  year      = {2025},
  publisher = {GitHub},
  note      = {Supervised by Prof. Iyad Hatem},
  url       = {https://github.com/B-A-IntelliBots/AI-optimized-LQR-Lyapunov-Control-System}
}
```

