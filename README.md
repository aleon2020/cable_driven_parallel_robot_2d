# Cable Driven Parallel Robot 2D

![distro](https://img.shields.io/badge/Ubuntu%2024.04-Noble%20Numbat-orange)
![distro](https://img.shields.io/badge/ROS2-Jazzy-blue)
[![jazzy](https://github.com/aleon2020/cable_driven_parallel_robot_2d/actions/workflows/main.yaml/badge.svg)](https://github.com/aleon2020/cable_driven_parallel_robot_2d/actions/workflows/main.yaml)

This repository is part of my Final Degree Project (FDP / TFG), focused on the design, construction, and commissioning of a **Cable-Driven Parallel Robot (CDPR)** capable of controlling a suspended end-effector to perform trajectory planning and motion control tasks.

This project integrates topics such as **kinematic modeling**, **control algorithms**, **software development in Python**, and **integration with ROS 2 Jazzy**, providing a modular and scalable framework for experimentation with cable-driven robotic platforms.

This workspace is organized into multiple ROS 2 packages, separated into the **robot controller**, the **predefined figure generation logic** and the **custom communication interfaces** to improve modularity and maintainability.

<p align="center">
  <img width="684" height="392" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/cable_driven_parallel_robot.png?raw=true">
</p>

---

## Features

- Management of a Cable Driven Parallel Robot using ROS 2 and RViZ.
- Compatible with **ROS 2 Jazzy**.
- Implemented in **Python** with support for configuration via **launch files**, **RViZ** and **URDF descriptions**.
- Built using **colcon**.
- Modular ROS 2 architecture composed by 3 packages:
  - **cdpr_2d/**: [Main controller](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/cdpr_2d/cdpr_2d/cdpr_controller.py) of the CDPR that includes the [robot model](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/cdpr_2d/description/urdf/robot.urdf) in URDF, the kinematic model and trayectory execution.
  - **cdpr_figures/**: [Library of predefined figures](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/cdpr_figures/cdpr_figures/figures_library.py) and [ROS 2 service server](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/cdpr_figures/cdpr_figures/figures_service.py) used for generating trajectories automatically.
  - **cdpr_interfaces/**: [Custom ROS 2 service definition](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/cdpr_interfaces/srv/DrawFigure.srv) used in the communication between packages.

---

## Project Structure

```sh
src
├── cdpr_2d
│   ├── cdpr_2d
│   │   ├── cdpr_controller.py
│   │   ├── __init__.py
│   │   └── __pycache__
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
│   ├── setup.cfg
│   └── setup.py
├── cdpr_figures
│   ├── cdpr_figures
│   │   ├── figures_library.py
│   │   ├── figures_service.py
│   │   ├── __init__.py
│   │   └── __pycache__
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── resource
│   │   └── cdpr_figures
│   ├── setup.cfg
│   └── setup.py
├── cdpr_interfaces
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv
│       └── DrawFigure.srv
├── docs
├── media
├── memory
├── README.md
└── scripts
```

---

## Kinematic Model Validation

Before integrating the kinematic model in ROS 2, it's validated in three different programming languages. 

The objective of this step is to ensure that all the mathematical formulation is consistent in result terms regardless of the implementation language and warn about potential errors before the model is integrated into ROS 2.

### MATLAB prototype

<p align="center">
  <img width="684" height="392" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/matlab_kinematic_model.png?raw=true">
</p>

### Python implementation

<p align="center">
  <img width="228" height="131" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v1.png?raw=true">
  <img width="228" height="131" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v2.png?raw=true">
  <img width="228" height="131" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v3.png?raw=true">
</p>

### C++ implementation

<p align="center">
  <img width="228" height="131" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/cpp_kinematic_model_v1.png?raw=true">
  <img width="228" height="131" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v2.png?raw=true">
  <img width="228" height="131" src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v3.png?raw=true">
</p>

**NOTE**: To see with more detail this process, take a look at the matlab/, python and c++/ folders inside the scripts/ directory.

---

## Installation

### Prerequisites

- **RViZ**.
- **ROS 2 Jazzy**.
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

3. (OPTIONAL) Run the figures service node that generates predefined trajectories and sends them to the controller:

```sh
ros2 run cdpr_figures figures_service
```

4. Launch each of the topics to see data related to the end effector (each of them in a different terminal):

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

5. Publish ONE of the following commands messages in the corresponding topic to send the coordinates to which you want to move the end effector:

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

### Case X: Drawing predefined figures using ROS 2 services

As an alternative way of publishing coordinates to the **/cdpr** topic, you can generate predefined figures divided in four different categories: **letters**, **numbers**, **polygons** or **shapes**.

The **cdpr_figures/** package provides a server that creates the requested trajectory and sends it to the controller, eliminating the need to specify each coordinate manually.

Before calling the service, make sure that the corresponding node is running:

```sh
ros2 run cdpr_figures figures_service
```

To determine the type associated with the figure drawing service:

```sh
ros2 service type /draw_figure
```

The custom service definition can also be inspected with:

```sh
ros2 interface show cdpr_interfaces/srv/DrawFigure
```

Finally, a predefined figure can be requested by calling the service:

```sh
ros2 service call /draw_figure cdpr_interfaces/srv/DrawFigure \
    "{category: letter, name: W, size: small}"
```

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/w_small_letter_execution.gif?raw=true">
</p>

```sh
ros2 service call /draw_figure cdpr_interfaces/srv/DrawFigure \
    "{category: polygon, name: square, size: medium}"
```

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/square_medium_polygon_execution.gif?raw=true">
</p>

```sh
ros2 service call /draw_figure cdpr_interfaces/srv/DrawFigure \
    "{category: shape, name: house, size: large}"
```

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/house_large_shape_execution.gif?raw=true">
</p>

**NOTE**: The request fields depend on the definition provided in [figures_library.py](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/cdpr_figures/cdpr_figures/figures_library.py).

---

### Author

**Alberto León Luengo**  
Software Robotics Engineering at **Rey Juan Carlos University**.  
Github: [aleon2020](https://github.com/aleon2020)  
Developed under the supervision of [Professor Juan Sebastián Cely Gutiérrez](https://github.com/juanscelyg), within the [Intelligent Robotics Lab](https://github.com/IntelligentRoboticsLabs).
