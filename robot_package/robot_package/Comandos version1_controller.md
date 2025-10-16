## ORDEN DE EJECUCIÓN DEL FICHERO VERSION1_CONTROLLER.PY

### TERMINAL 1

```sh
colcon build --symlink-install
```

### TERMINAL 2

```sh
ros2 run robot_package version1_controller
```

### TERMINAL 3

```sh
ros2 topic echo /effector_coordinates
```

### TERMINAL 4

```sh
ros2 topic echo /cable_parameters
```

### TERMINAL 5

```sh
ros2 topic echo /pulley_parameters
```

### TERMINAL 6

```sh
ros2 topic pub --once /version1 interfaces_package/msg/EffectorPosition "{position: {x: 0.30, y: 0.30}}"
```