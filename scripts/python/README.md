# Kinematics in Python

## 1. Introduction

This README shows the execution results of a cable-guided robot in the Python programming language.

To do this, the matplotlib library is used to visualize all the elements in the interface.

This Python implementation evolves from a static simulation, through a trajectory animation that goes from a source point to a destination point, to a trajectory animation with multiple points.

## 2. Versions

### 2.1 Version 1 (Static)

**Version 1 (file [cinematic_model_v1.py](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/python/cinematic_model_v1.py))**: It consists of a basic simulation of a 2D robot supported by cables with a fixed end-effector position. The length of each cable (L1 and L2) and their respective angles (q1 and q2) are calculated, the workspace boundaries are verified, and the structure is graphed.

To compile and run this version, run the following command in a terminal:

```sh
python3 cinematic_model_v1.py
```

And finally, an image is attached showing the final result of this version:

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v1.png?raw=true">
</p>

### 2.2 Version 2 (Animation between 2 points)

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/python_kinematic_model_v2_execution.gif?raw=true">
</p>

**Version 2 (file [cinematic_model_v2.py](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/python/cinematic_model_v2.py))**: It consists of an extension of Version 1 in which a movement animation is added between an initial position and a final position, all while showing the trajectory that the end effector is taking through a blue dashed line while dynamically updating the position of the cables and the end effector.

To compile and run this version, run the following command in a terminal:

```sh
python3 cinematic_model_v2.py
```

And finally, a video is attached showing the complete execution of this version, as well as an image corresponding to the final result once the animation is finished:

VIDEO V2 PYTHON

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v2.png?raw=true">
</p>

### 2.3 Version 3 (Trajectory with N points)

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/python_kinematic_model_v3_execution.gif?raw=true">
</p>

**Version 3 (file [cinematic_model_v3.py](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/python/cinematic_model_v3.py))**: It consists of an extension of Version 2, with the difference that multiple positions are now supported (from 2 to 10) to be able to create a more complex trajectory, where the animation is divided into segments between each pair of points, maintaining all the functionalities previously implemented (data calculation, limit verification and graphical display of the animation).

To compile and run this version, run the following command in a terminal:

```sh
python3 cinematic_model_v3.py
```

And finally, a video is attached showing the complete execution of this version, as well as an image corresponding to the final result once the animation is finished:

VIDEO V3 PYTHON

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/python_kinematic_model_v3.png?raw=true">
</p>