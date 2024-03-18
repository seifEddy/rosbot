
# ROSBOT: A ROS-based Mobile Robot

Welcome to the repository of ROSBOT, a mobile robot platform developed using the Robot Operating System (ROS). ROSBOT is designed with simplicity and modularity in mind, featuring a two-wheeled design with encoders for precise velocity control.

## Overview

ROSBOT aims to provide a versatile platform for robotics research and development, focusing on mobility tasks such as autonomous navigation, mapping, and obstacle avoidance. Equipped with wheel encoders and a robust hardware interface, ROSBOT integrates seamlessly with `ros_control` for effective velocity regulation, making it an ideal tool for education and prototyping in robotics.

### Features

- Two-wheeled differential drive system
- High-resolution wheel encoders for accurate velocity measurement
- Custom hardware interface compatible with `ros_control`
- Modular design for easy customization and upgrades

## Getting Started

### Prerequisites

Ensure you have ROS installed on your system. ROSBOT is compatible with ROS Noetic and newer versions. For installation instructions, refer to the [official ROS documentation](http://wiki.ros.org/ROS/Installation).

### Installation

1. Clone the ROSBOT repository into your catkin workspace's `src` folder:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/seifEddy/rosbot.git
   ```

2. Build your catkin workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. Source the setup file to add the workspace to your ROS environment:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

### Usage

To launch ROSBOT, use the provided launch files. For example, to start ROSBOT with the default configuration:

```bash
roslaunch rosbot bringup.launch
```

## Contact

For questions and support, please open an issue in the GitHub repository, or contact us directly at.
seghiri.free@gmail.com.

---

Hello Robots! :)