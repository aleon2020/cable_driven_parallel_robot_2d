# Kinematics in C++

## 1. Introduction

This README shows the execution results of a cable-powered robot in the C++ programming language.

To do this, the [**matplotlibcpp.h**](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/c%2B%2B/matplotlibcpp.h) library is used to display all the elements in the interface.

This C++ implementation evolves from a static simulation, through a trajectory animation that goes from a source point to a destination point, to a trajectory animation with multiple points.

## 2. Versions

### 2.1 Version 1 (Static)

**Version 1 (file [cinematic_model_v1.cpp](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/c%2B%2B/cinematic_model_v1.cpp))**: It consists of a basic simulation of a 2D robot held by cables with a fixed position of the end effector, in which the length of each of the cables (L1 and L2) and their respective angles (q1 and q2) are calculated, the limits of the workspace are verified and the structure is graphed.

To compile and run this version, run the following commands in a terminal:

```sh
g++ cinematic_model_v1.cpp -o cinematic_model_v1 $(python3-config --embed --cflags) -I $(python3 -c "import numpy; print(numpy.get_include())") $(python3-config --embed --ldflags) -Wno-deprecated-declarations
```

```sh
./cinematic_model_v1
```

And finally, an image is attached showing the final result of this version:

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/cpp_kinematic_model_v1.png?raw=true">
</p>

### 2.2 Version 2 (Animation between 2 points)

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/cpp_kinematic_model_v2_execution.gif?raw=true">
</p>

**Version 2 (file [cinematic_model_v2.cpp](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/c%2B%2B/cinematic_model_v2.cpp))**: It consists of an extension of Version 1 in which a movement animation is added between an initial position and a final position, all while showing the trajectory that the end effector is taking through a blue dashed line while dynamically updating the position of the cables and the end effector.

To compile and run this version, run the following commands in a terminal:

```sh
g++ cinematic_model_v2.cpp -o cinematic_model_v2 $(python3-config --embed --cflags) -I $(python3 -c "import numpy; print(numpy.get_include())") $(python3-config --embed --ldflags) -Wno-deprecated-declarations
```

```sh
./cinematic_model_v2
```

And finally, a video is attached showing the complete execution of this version, as well as an image corresponding to the final result once the animation is finished:

https://github.com/user-attachments/assets/eda6e7cd-b6bb-4175-8fa7-64bbac433423

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/cpp_kinematic_model_v2.png?raw=true">
</p>

### 2.3 Version 3 (Trajectory with N points)

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/gifs/cpp_kinematic_model_v3_execution.gif?raw=true">
</p>

**Version 3 (file [cinematic_model_v3.cpp](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/scripts/c%2B%2B/cinematic_model_v3.cpp))**: It consists of an extension of Version 2, with the difference that now multiple positions (from 2 to 10) are supported in order to perform a more complex trajectory, where the animation is divided into segments between each pair of points, maintaining all the previously implemented functionalities (data calculation, limit verification and graphical visualization of the animation).

To compile and run this version, run the following commands in a terminal:

```sh
g++ cinematic_model_v3.cpp -o cinematic_model_v3 $(python3-config --embed --cflags) -I $(python3 -c "import numpy; print(numpy.get_include())") $(python3-config --embed --ldflags) -Wno-deprecated-declarations
```

```sh
./cinematic_model_v3
```

And finally, a video is attached showing the complete execution of this version, as well as an image corresponding to the final result once the animation is finished:

https://github.com/user-attachments/assets/57b63029-464f-4fad-a872-5e934b025f10

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/cpp_kinematic_model_v3.png?raw=true">
</p>
