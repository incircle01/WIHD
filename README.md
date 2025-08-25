# Morphing-Drone-sim
<img width="270" height="213" alt="그림3" src="https://github.com/user-attachments/assets/a19363b6-0a2c-466e-8a3d-f8f6fe19c90d" />
<img width="270" height="213" alt="그림 4" src="https://github.com/user-attachments/assets/bb00d7a8-4dc5-43fc-afa2-1c91e8d97e67" />

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
![제목 없음](https://github.com/user-attachments/assets/3037f21c-05d4-496c-8fd8-ed92e114ae3a)


![제목 없음 (1)](https://github.com/user-attachments/assets/356a8263-5ce3-4ed8-8ce4-39aab4aa7461)


![새로운 프로젝트](https://github.com/user-attachments/assets/2aceca7f-54ab-496e-b35e-f2a901c88d0a)
