# Cable Driven Parallel Robot 2D

![distro](https://img.shields.io/badge/Ubuntu%2024-Jammy%20Jellyfish-orange)
![distro](https://img.shields.io/badge/ROS2-Rolling-blue)
[![jazzy](https://github.com/aleon2020/cable_driven_parallel_robot_2d/actions/workflows/main.yaml/badge.svg)](https://github.com/aleon2020/cable_driven_parallel_robot_2d/actions/workflows/main.yaml)

This repository is part of a Final Degree Project (FDP / TFG) focused on the design, construction, and commissioning of a **Cable-Driven Parallel Robot (CDPR)** capable of controlling a suspended end-effector to perform trajectory planning and motion control tasks.

The project integrates topics such as **kinematic modeling**, **control algorithms**, **software development in Python**, and **integration with ROS2 Rolling**, providing a modular and scalable framework for experimentation with cable-driven robotic platforms.

<p align="center">
  <img width="684" height="392" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/cable_driven_parallel_robot.png?raw=true">
</p>

---

## Features

- Management of a Cable Driven Parallel Robot using ROS2 and RViZ.
- Compatible with **ROS2 Rolling**.
- Implemented in **Python** with support for configuration via **ROS2 launch files**.
- Modular architecture:
  - **cdpr_2d/**: CDPR source code.
  - **config/**: RViZ configuration.
  - **description/**: CDPR model in URDF for RViZ.
  - **launch/**: Launch files.
  - **resource/**: Resource file.
  - **test/**: Test files.
  - **package.xml**: Package metadata.
  - **setup.cfg**: Build configuration.
  - **setup.py**: Build configuration.
- Built using **colcon**.

---

## Project Structure

```sh
src
├── cdpr_2d
│   ├── cdpr_2d
│   │   ├── cdpr_controller.py
│   │   └── __init__.py
│   ├── config
│   │   ├── params.yaml
│   │   └── robot.rviz
│   ├── description
│   │   ├── robot.xacro
│   │   └── urdf
│   │       └── robot.urdf
│   ├── launch
│   │   └── robot_state_publisher.launch.py
│   ├── package.xml
│   ├── resource
│   │   └── cdpr_2d
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── docs
├── media
├── README.md
└── scripts
```

---

## Installation

### Prerequisites

- **RViZ**.
- **ROS2 Rolling**.
- **Python ≥ 3.12.3**.
- **colcon** for building.

### Setup

In this case, the repository is the equivalent of the **src/** directory of a workspace, so it would be necessary to create a new workspace from scratch.

1. Clone the workspace:

```sh
mkdir <my_workspace>
```

```sh
cd <my_workspace>/
```

```sh
git clone https://<my_token>@github.com/aleon2020/cable_driven_parallel_robot_2d.git
```

2. Rename the cloned repository:

```sh
mv cable_driven_parallel_robot_2d src
```

3. Compile the workspace:

```sh
cd <my_workspace>/
```

```sh
colcon build --symlink-install
```

```sh
source ~/<my_workspace>/install/setup.bash
```

**IMPORTANT**: Add your token exactly as shown. This is to avoid having to enter the token in the terminal every time you want to update the repository using the 'git pull' command.

If you've already cloned this repository, run the following command before starting to work with it, as new changes or modifications may have been added. This is to ensure you have cloned the most recent version of the repository:

```sh
git pull
```

---

## Usage

1. Run the controller:

```sh
ros2 run cdpr_2d cdpr_controller
```

2. Launch the simulation and visualization in RViZ:

```sh
ros2 launch cdpr_2d robot_state_publisher.launch.py
```

3. Launch each of the topics to see data related to the end effector (each of them in a different terminal):

```sh
# Position (X,Y) of the end effector
ros2 topic echo /effector_coordinates
```

```sh
# Cable lengths and angles with the vertical of each cable
ros2 topic echo /cable_parameters
```

```sh
# Elongated / Retracted cable length and rotated angle of each pulley
ros2 topic echo /pulley_parameters
```

4. Publish ONE of the following commands messages in the corresponding topic to send the coordinates to which you want to move the end effector:

### Case 1: 1 point (Fixed Position)

```sh
ros2 topic pub --once /cdpr nav_msgs/msg/Path \
"{header: {frame_id: 'world'}, \
  poses: [ \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.3, y: 0.3, z: 0.0}}} \
  ] \
}"
```

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/1_point_execution.gif?raw=true">
</p>

To view the output generated in the controller when running this case, click on the [following link](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/files/1_point_output).

### Case 2: 2 points (Initial and Final Position)

```sh
ros2 topic pub --once /cdpr nav_msgs/msg/Path \
"{header: {frame_id: 'world'}, \
  poses: [ \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.3, y: 0.3, z: 0.0}}}, \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.7, y: 0.7, z: 0.0}}} \
  ] \
}"
```

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/2_points_execution.gif?raw=true">
</p>

To view the output generated in the controller when running this case, click on the [following link](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/files/2_points_output).

### Case 3: 3 or more points (Trajectory)

```sh
ros2 topic pub --once /cdpr nav_msgs/msg/Path \
"{header: {frame_id: 'world'}, \
  poses: [ \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.2, y: 0.2, z: 0.0}}}, \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.2, y: 0.8, z: 0.0}}}, \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.8, y: 0.8, z: 0.0}}}, \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.8, y: 0.2, z: 0.0}}}, \
    {header: {frame_id: 'world'}, \
     pose: {position: {x: 0.2, y: 0.2, z: 0.0}}} \
  ] \
}"
```

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/3_or_more_points_execution.gif?raw=true">
</p>

To view the output generated in the controller when running this case, click on the [following link](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/files/3_or_more_points_output).

---

### References

[https://ieeexplore.ieee.org/document/8920840](https://ieeexplore.ieee.org/document/8920840)

---

### Author

**Alberto León Luengo**  
Software Robotics Engineering at **Rey Juan Carlos University**.  
Github: [aleon2020](https:/github.com/aleon2020)  
Developed under the supervision of [Professor Juan Sebastián Cely Gutiérrez](https://github.com/juanscely), within the [Intelligent Robotics Lab](https://github.com/IntelligentRoboticsLabs).
