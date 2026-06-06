# Kinematics in Python

## 1. Introduction

This third post will show the execution results of a cable-guided robot, whose kinematic model is explained in detail in the first post on this wiki, in the Python programming language.

To do this, the matplotlib library is used to visualize all the elements in the interface.

This Python implementation evolves from a static simulation, through a trajectory animation that goes from a source point to a destination point, to a trajectory animation with multiple points.

## 2. Versions

### 2.1 Version 1 (Static)

This consists of a basic simulation of a 2D robot supported by cables with a fixed end-effector position. The length of each cable (L1 and L2) and their respective angles (q1 and q2) are calculated, the workspace boundaries are verified, and the structure is graphed.

**VERSION 2.1.1: [SINGLE RUN](https://github.com/aleon2020/TFG_GIRS_2025/blob/main/src/python/cinematic_model_v1.py)**

To compile and run this version, run the following command in a terminal:

```sh
python3 cinematic_model_v1.py
```

**VERSION 2.1.2: [MODULAR EXECUTION](https://github.com/aleon2020/TFG_GIRS_2025/blob/main/src/python/version_1/version_1/cinematic_model_v1.py)**

To compile and run this version, run the following commands in a terminal:

```sh
cd version_1/
```

```sh
ls
setup.py  version_1/
```

```sh
pip install -e . --break-system-packages
```

```sh
python3 -m version_1.cinematic_model_v1
[ program execution ... ]
```

```sh
pip uninstall version_1 --break-system-packages
```

And finally, an image is attached showing the final result of this version:

<p align="center">
  <img src="https://github.com/aleon2020/TFG_GIRS_2025/blob/main/media/Imagen%20Modelo%20Cinem%C3%A1tico%20V1%20Python.png?raw=true">
</p>

### 2.2 Version 2 (Animation between 2 points)

<p align="center">
  <img src="https://github.com/aleon2020/TFG_GIRS_2025/blob/main/media/GIF%20Ejecuci%C3%B3n%20Modelo%20Cinem%C3%A1tico%20V2%20Python.gif?raw=true">
</p>

It consists of an extension of Version 1 in which a movement animation is added between an initial position and a final position, all while showing the trajectory that the end effector is taking through a blue dashed line while dynamically updating the position of the cables and the end effector.

**VERSION 2.2.1: [SINGLE EXECUTION](https://github.com/aleon2020/TFG_GIRS_2025/blob/main/src/python/cinematic_model_v2.py)**

To compile and run this version, run the following command in a terminal:

```sh
python3 cinematic_model_v2.py
```

**VERSION 2.2.2: [MODULAR RUN](https://github.com/aleon2020/TFG_GIRS_2025/blob/main/src/python/version_2/version_2/cinematic_model_v2.py)**

To compile and run this version, run the following commands in a terminal:

```sh
cd version_2/
```

```sh
ls
setup.py  version_2/
```

```sh
pip install -e . --break-system-packages
```

```sh
python3 -m version_2.cinematic_model_v2
[ program execution ... ]
```

```sh
pip uninstall version_2 --break-system-packages
```

And finally, a video is attached showing the complete execution of this version, as well as an image corresponding to the final result once the animation is finished:

https://github.com/user-attachments/assets/f2a3b80b-28e0-4924-bdea-eb742a9902e0

<p align="center">
  <img src="https://github.com/aleon2020/TFG_GIRS_2025/blob/main/media/Imagen%20Modelo%20Cinem%C3%A1tico%20V2%20Python.png?raw=true">
</p>

### 2.3 Version 3 (Trajectory with N points)

<p align="center">
  <img src="https://github.com/aleon2020/TFG_GIRS_2025/blob/main/media/GIF%20Ejecuci%C3%B3n%20Modelo%20Cinem%C3%A1tico%20V3%20Python.gif?raw=true">
</p>

It consists of an extension of Version 2, with the difference that multiple positions are now supported (from 2 to 10) to be able to create a more complex trajectory, where the animation is divided into segments between each pair of points, maintaining all the functionalities previously implemented (data calculation, limit verification and graphical display of the animation).

**VERSION 2.3.1: [SINGLE RUN](https://github.com/aleon2020/TFG_GIRS_2025/blob/main/src/python/cinematic_model_v3.py)**

To compile and run this version, run the following command in a terminal:

```sh
python3 cinematic_model_v3.py
```

**VERSION 2.3.2: [MODULAR RUN](https://github.com/aleon2020/TFG_GIRS_2025/blob/main/src/python/version_3/version_3/cinematic_model_v3.py)**

To compile and run this version, run the following commands in a terminal:

```sh
cd version_3/
```

```sh
ls
setup.py  version_3/
```

```sh
pip install -e . --break-system-packages
```

```sh
python3 -m version_3.cinematic_model_v3
[ program execution ... ]
```

```sh
pip uninstall version_3 --break-system-packages
```

And finally, a video is attached showing the complete execution of this version, as well as an image corresponding to the final result once the animation is finished:

https://github.com/user-attachments/assets/eaead8f6-be54-425f-93f6-d0fe2454d968

<p align="center">
  <img src="https://github.com/aleon2020/TFG_GIRS_2025/blob/main/media/Imagen%20Modelo%20Cinem%C3%A1tico%20V3%20Python.png?raw=true">
</p>