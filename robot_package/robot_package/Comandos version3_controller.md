## ORDEN DE EJECUCIÓN DEL FICHERO VERSION3_CONTROLLER.PY

### TERMINAL 1

```sh
colcon build --symlink-install
```

### TERMINAL 2

```sh
ros2 run robot_package version3_controller
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
ros2 topic echo /effector_log
```

### TERMINAL 7

```sh
ros2 topic echo /cable_log
```

### TERMINAL 8

```sh
ros2 topic echo /pulley_log
```

### TERMINAL 9

```sh
ros2 topic pub --once /version3 interfaces_package/msg/EffectorPath "
points_number: 7
points:
- {x: 0.30, y: 0.50}
- {x: 0.40, y: 0.70}
- {x: 0.60, y: 0.70}
- {x: 0.70, y: 0.50}
- {x: 0.60, y: 0.30}
- {x: 0.40, y: 0.30}
- {x: 0.30, y: 0.50}
"
```