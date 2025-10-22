# Trabajo de Fin de Grado 2025-2026

¡Bienvenido! En este repositorio se encuentran todos los materiales correspondientes al Trabajo de Fin de Grado realizado en el Grado en Ingeniería de Robótica Software.

**TEMA**: Implementación de un robot por cables para el control de un efector final en diversas tareas

**TUTOR**: [Juan Sebastián Cely Gutiérrez](https://github.com/juanscelyg)

## 1. Compilación y ejecución del workspace

En este caso, el repositorio es el equivalente al **directorio src/** de un workspace, por lo que sería necesario crear un workspace nuevo desde cero.

A continuación se detallan los pasos a seguir para compilar y ejecutar correctamente todos los paquetes que componen este repositorio:

### 1.1 PASO 1: Clonación del workspace

Para clonar este repositorio es recomendable abrir la terminal desde el **directorio HOME/** (carpeta personal):

```sh
mkdir tfg_girs_ws
```

```sh
cd tfg_girs_ws/
```

```sh
git clone https://<token>@github.com/aleon2020/TFG_GIRS_2025_SRC.git
```

**IMPORTANTE**: Añade tu token tal y como se muestra. Esto se hace con el objetivo de no tener que introducir el token en la terminal cada vez que se quiera actualizar el repositorio utilizando el comando 'git pull'.

Si ya has clonado este repositorio, ejecuta el siguiente comando antes de que empieces a trabajar con él, ya que pueden haberse añadido nuevos cambios o modificaciones. Esto se hace con el objetivo de asegurarse de que se tiene clonada la versión más reciente del repositorio:

```sh
git pull
```

### 1.2 PASO 2: Cambio de nombre del repositorio

Una vez clonado el repositorio en tu **directorio HOME/** (carpeta personal), éste aparecerá con el nombre **"TFG_GIRS_2025_SRC"**, por lo que se renombrará con otro nombre para evitar el uso de mayúsculas y números. Para ello, se debe ejecutar el siguiente comando en la terminal:

```sh
mv TFG_GIRS_2025_SRC src
```

### 1.3 PASO 3: Compilación del workspace

Con el repositorio ya renombrado de una forma más acorde a la de un workspace, se debe compilar el workspace ejecutando los siguientes comandos en una nueva terminal:

```sh
cd tfg_girs_ws/
```

```sh
colcon build --symlink-install
```

Una vez terminada la compilación del workspace con colcon, añade la siguiente línea en el fichero .bashrc desde el **directorio HOME/** (carpeta personal):

```sh
nano .bashrc
```

```sh
source ~/tfg_girs_ws/install/setup.bash
```

Una vez guardados los cambios, se cierran tanto el fichero como la terminal. Esto se hace con el objetivo de que los cambios realizados en el fichero .bashrc funcionen correctamente.

## 2. Comandos de ejecución de los controladores

### 2.1 Versión 1: version1_controller.py

```sh
# TERMINAL 1
colcon build --symlink-install
```

```sh
# TERMINAL 2
ros2 run cdpr_2d version1_controller
```

```sh
# TERMINAL 3
ros2 topic echo /effector_coordinates
```

```sh
# TERMINAL 4
ros2 topic echo /cable_parameters
```

```sh
# TERMINAL 5
ros2 topic echo /pulley_parameters
```

```sh
# TERMINAL 6
ros2 topic pub --once /version1 geometry_msgs/msg/PoseStamped "{pose: {position: {x: 0.30, y: 0.30, z: 0.0}}}"
```

### 2.2 Versión 2: version2_controller.py

```sh
# TERMINAL 1
colcon build --symlink-install
```

```sh
# TERMINAL 2
ros2 run cdpr_2d version2_controller
```

```sh
# TERMINAL 3
ros2 topic echo /effector_coordinates
```

```sh
# TERMINAL 4
ros2 topic echo /cable_parameters
```

```sh
# TERMINAL 5
ros2 topic echo /pulley_parameters
```

```sh
# TERMINAL 6
ros2 topic pub --once /version2 nav_msgs/msg/Path "{poses: [{pose: {position: {x: 0.30, y: 0.30, z: 0.0}}}, {pose: {position: {x: 0.70, y: 0.70, z: 0.0}}}]}"
```

### 2.3 Versión 3: version3_controller.py

```sh
# TERMINAL 1
colcon build --symlink-install
```

```sh
# TERMINAL 2
ros2 run cdpr_2d version3_controller
```

```sh
# TERMINAL 3
ros2 topic echo /effector_coordinates
```

```sh
# TERMINAL 4
ros2 topic echo /cable_parameters
```

```sh
# TERMINAL 5
ros2 topic echo /pulley_parameters
```

```sh
# TERMINAL 6
ros2 topic pub --once /version3 geometry_msgs/msg/PoseArray "
poses:
- {position: {x: 0.30, y: 0.50, z: 0.0}}
- {position: {x: 0.40, y: 0.70, z: 0.0}}
- {position: {x: 0.60, y: 0.70, z: 0.0}}
- {position: {x: 0.70, y: 0.50, z: 0.0}}
- {position: {x: 0.60, y: 0.30, z: 0.0}}
- {position: {x: 0.40, y: 0.30, z: 0.0}}
- {position: {x: 0.30, y: 0.50, z: 0.0}}
"
```