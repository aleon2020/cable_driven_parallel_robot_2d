# Trabajo de Fin de Grado 2025-2026

¡Bienvenido! En este repositorio se encuentran todos los materiales correspondientes al Trabajo de Fin de Grado realizado en el Grado en Ingeniería de Robótica Software.

**TEMA**: Implementación de un robot por cables para el control de un efector final en diversas tareas

**TUTOR**: [Juan Sebastián Cely Gutiérrez](https://github.com/juanscelyg)

## 1. Compilación y ejecución del workspace

En este caso, el repositorio es el workspace completo, por lo que no sería necesario crear un workspace desde cero.

A continuación se detallan los pasos a seguir para compilar y ejecutar correctamente todos los paquetes que componen este repositorio:

### 1.1 PASO 1: Clonación del workspace

Para clonar este repositorio es recomendable abrir la terminal desde el **directorio HOME** (carpeta personal):

```sh
git clone https://<token>@github.com/aleon2020/TFG_GIRS_2025_WS.git
```

**IMPORTANTE**: Añade tu token tal y como se muestra. Esto se hace con el objetivo de no tener que introducir el token en la terminal cada vez que se quiera actualizar el repositorio utilizando el comando 'git pull'.

Si ya has clonado este repositorio, ejecuta el siguiente comando antes de que empieces a trabajar con él, ya que pueden haberse añadido nuevos cambios o modificaciones. Esto se hace con el objetivo de asegurarse de que se tiene clonada la versión más reciente del repositorio:

```sh
git pull
```

### 1.2 PASO 2: Cambio de nombre del repositorio

Una vez clonado el repositorio en tu **directorio HOME** (carpeta personal), éste aparecerá con el nombre **"TFG_GIRS_2025_WS"**, por lo que se renombrará con otro nombre para evitar el uso de mayúsculas y números. Para ello, se debe ejecutar el siguiente comando en la terminal:

```sh
mv TFG_GIRS_2025_WS tfg_girs_ws
```

### 1.3 PASO 3: Compilación del workspace

Con el repositorio ya renombrado de una forma más acorde a la de un workspace, se debe compilar el workspace ejecutando los siguientes comandos en una nueva terminal:

```sh
cd tfg_girs_ws/
```

```sh
colcon build --symlink-install
```

Una vez terminada la compilación del workspace con colcon, añade la siguiente línea en el fichero .bashrc desde el **directorio HOME** (carpeta personal):

```sh
nano .bashrc
```

```sh
source ~/tfg_girs_ws/install/setup.bash
```

Una vez guardados los cambios, se cierran tanto el fichero como la terminal. Esto se hace con el objetivo de que los cambios realizados en el fichero .bashrc funcionen correctamente.

## 2. Comandos de ejecución de cada paquete

### 2.1 Paquete **"actions_package"**

```sh
# TERMINAL 1
ros2 run actions_package count_until_server
```

```sh
# TERMINAL 2
ros2 run actions_package count_until_client
```

### 2.2 Paquete **"interfaces_package"**

```sh
# TERMINAL 1
ros2 interface show interfaces_package/action/CountUntil
```

### 2.3 Paquete **"nodes_package"**

```sh
# TERMINAL 1
ros2 run nodes_package versionX_node
```

```sh
# TERMINAL 2
ros2 run nodes_package versionX_subscriber
```

```sh
# TERMINAL 3
ros2 topic echo /effector_coordinates
```

**IMPORTANTE**: Este comando se debe ejecutar cuando se está probando cualquiera de las 3 versiones de los nodos (version1, version2 o version3).

```sh
# TERMINAL 4
ros2 topic echo /cable_parameters
```

**IMPORTANTE**: Este comando se debe ejecutar cuando se está probando cualquiera de las 3 versiones de los nodos (version1, version2 o version3).

```sh
# TERMINAL 5
ros2 topic echo /pulley_parameters
```

**IMPORTANTE**: Este comando se debe ejecutar cuando se están probando las versiones version2 o version3 de los nodos.

### 2.4 Paquete **"robot_package"**

```sh
# TERMINAL 1
ros2 launch robot_package robot_state_publisher.launch.py
```

```sh
# TERMINAL 2
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```