# Morphing-Drone-sim
<img width="270" height="213" alt="그림3" src="https://github.com/user-attachments/assets/a19363b6-0a2c-466e-8a3d-f8f6fe19c90d" />
<img width="270" height="213" alt="그림7" src="https://github.com/user-attachments/assets/b30d0ac0-e796-490b-8dc5-5255d72eeb5e" />

A simulation project for a 12-actuator drone featuring:
- 4 main motors
- 4 tilt servos
- 4 flare servos

## Simulation Structure
<img width="1972" height="1095" alt="KakaoTalk_20250814_162350736" src="https://github.com/user-attachments/assets/70d5f695-e329-49dd-be8e-d8e2fbe997e5" />


This project uses **ROS 2 Humble** and **Gazebo Classic** for simulation and visualization of the morphing drone dynamics.

## Dependencies
All required dependencies for ROS 2 and Gazebo Classic can be installed using the provided script.


## ROS 2 & Gazebo Setup Script
This script installs essential dependencies for using **ROS 2 (Humble)** with **Gazebo Classic**, and sets up a recommended development environment for working with URDF/XACRO models.

### What it installs

- **ROS 2 packages**:
  - `joint-state-publisher-gui`
  - `robot-state-publisher`
  - `xacro`
  - `gazebo-ros-pkgs`
  - `ros2_control` and controllers
- **Gazebo simulator** and plugins
- **Python** dependencies:
  - `catkin_pkg`
- (Optional) **VSCode extensions**:
  - XML syntax highlighting
  - URDF model tools

### How to Use

1. Clone this repository and navigate into it:
   ```bash
   git clone https://github.com/your-username/Morphing-Drone-sim.git
   cd Morphing-Drone-sim
   ```
2. Give execution permission to the setup script:
   ```bash
   chmod +x setup_dependencies.sh
   ```
3. Run the script to install all required packages:
   ```bash
   ./setup_dependencies.sh
   ```
4. Source your ROS2 environment:

   ```bash
   source /opt/ros/humble/setup.bash
   ```
5. Build the workspace:
   ```bash
   cd ROS2_ws
   colcon build
   source install/setup.bash
   ```
6. Launch the simulation (example):
   ```bash
   ros2 launch morphing_description morphing_launch.py
   ```
### Failsafe Algorithm
1. Open another terminal 
2. Source your ROS2 environment
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Publish ROS2 msg 
   ```bash
   ros2 topic pub /motor_fail std_msgs/msg/Int32 "{data: 1}"
   ```
### Small Gap Passing(matlab)
![ezgif-232945de83ba52](https://github.com/user-attachments/assets/0321cdfc-0a8b-4bf8-a2a3-b9c9816516ee)

### Flight recovery after main motor failure(matlab)
![ezgif-23e7b4b0f184f7](https://github.com/user-attachments/assets/bbf31fc0-f6bb-49b3-a913-7409ea3fcb4d)


### Flight recovery after main motor failure(Gazebo)
![새로운 프로젝트](https://github.com/user-attachments/assets/2aceca7f-54ab-496e-b35e-f2a901c88d0a)

### Youtube
https://youtu.be/Sj3WPfR2m7s?si=NNqQVpsa4pOoUwik

### Contact
godstar1112@naver.com
PROJECT by 정인영 (IN YOUNG JUNG)

