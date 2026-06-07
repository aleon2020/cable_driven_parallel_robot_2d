# Shapes Scripts

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

4. After fulfilling all installation prerequisites and configuring and compiling the entire workspace, as well as following all the steps mentioned in the [**Installation**](https://github.com/aleon2020/cable_driven_parallel_robot_2d#installation) section of the [**main README**](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/README.md) of this repository, you can send the coordinates to which you want to move the end effector using custom scripts that make the end-effector pass through a series of coordinates, obtaining as a final result a letter, a number, a polygon, or another type of geometric shape.

```sh
cd scripts/shapes/
```

5. In this case, the script name determines which shape will be drawn. Below is the complete list of shapes that can be tested:

```sh
# <shape.sh>
horizontal_hg.sh
house.sh
lightning.sh
shield.sh
vertical_hg.sh
```

**NOTE**: To see how each shape would look, you can take a look at the following [document](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/docs/geometric_figures_and_shapes_image_glossary.pdf).

6. Finally, the only argument passed to each of these scripts can be one of these three options:

- [small](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/docs/geometric_figures_and_shapes_coordinates_glossary_40x40.pdf): A 0.4 x 0.4 meter figure concentric to the 1 x 1 meter end-effector movement region.

- [medium](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/docs/geometric_figures_and_shapes_coordinates_glossary_60x60.pdf): A 0.6 x 0.6 meter figure concentric to the 1 x 1 meter end-effector movement region.

- [large](https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/docs/geometric_figures_and_shapes_coordinates_glossary_80x80.pdf): A 0.8 x 0.8 meters figure concentric to the 1 x 1 meter end effector movement region.

7. So, knowing all this, the command to run any of these scripts would follow this naming convention:

```sh
# EXAMPLE: ./lightning.sh medium
./<shape.sh> [ small | medium | large ]
```
