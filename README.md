# Trabajo de Fin de Grado 2025-2026

¡Bienvenido! En este repositorio se encuentran todos los materiales correspondientes al Trabajo de Fin de Grado realizado en el Grado en Ingeniería de Robótica Software.

**TEMA**: Implementación de un robot por cables para el control de un efector final en diversas tareas

**TUTOR**: [Juan Sebastián Cely Gutiérrez](https://github.com/juanscelyg)

## 1. Activación de ROS2

Se abre una terminal desde el directorio HOME (carpeta personal) y ejecuta el siguiente comando en la terminal:

```sh
nano .bashrc
```

A continuación, se escribe la siguiente línea dentro del fichero:

```sh
source /opt/ros/rolling/setup.bash
```

Una vez guardados los cambios, se cierran tanto el fichero como la terminal. Esto se hace con el objetivo de que los cambios realizados en el fichero .bashrc funcionen correctamente.

Y por último, para comprobar que ROS2 funciona correctamente, se debe ejecutar el siguiente comando en la terminal:

```sh
ros2
```

## 2. Compilación y ejecución del workspace

En este caso, el repositorio es el propio workspace, por lo que no sería necesario crearlo desde cero.

A continuación se detallan los pasos a seguir para compilar y ejecutar correctamente todos los paquetes que componen este repositorio:

**PASO 1: Clonación del workspace**

Para clonar este repositorio es recomendable abrir la terminal desde el directorio HOME (carpeta personal):

```sh
git clone https://<token>@github.com/aleon2020/TFG_GIRS_2025_WS.git
```

**IMPORTANTE**: Añade tu token tal y como se muestra. Esto se hace con el objetivo de no tener que introducir el token en la terminal cada vez que se quiera actualizar el repositorio utilizando el comando 'git pull'.

Si ya has clonado este repositorio, ejecuta el siguiente comando antes de que empieces a trabajar con él, ya que pueden haberse añadido nuevos cambios o modificaciones. Esto se hace con el objetivo de asegurarte de que tienes clonada la versión más reciente del repositorio:

```sh
git pull
```

**PASO 2: Renombramiento del repositorio**

Una vez clonado el repositorio en tu HOME, éste aparecerá con el nombre **TFG_GIRS_2025_WS**, por lo que se renombrará con otro nombre para evitar el uso de mayúsculas y números. Para ello, se debe ejecutar el siguiente comando en la terminal:

```sh
mv TFG_GIRS_2025_WS tfg_girs_ws
```

**PASO 3: Compilación del workspace**

Con el repositorio ya renombrado de una forma más acorde a la de un workspace, se debe compilar el workspace ejecutando los siguientes comandos en una nueva terminal:

```sh
cd tfg_girs_ws/
```

```sh
colcon build --symlink-install
```

Una vez terminada la compilación del workspace con colcon, añade la siguiente línea en el fichero .bashrc desde el directorio HOME (carpeta personal):

```sh
nano .bashrc
```

```sh
source ~/tfg_girs_ws/install/setup.bash
```

Una vez guardados los cambios, se cierran tanto el fichero como la terminal. Esto se hace con el objetivo de que los cambios realizados en el fichero .bashrc funcionen correctamente.

## 3. Comandos de ejecución de cada paquete

### 3.1 Paquete "my_robot"

```sh
cd tfg_girs_ws/
```

**TERMINAL 1**

```sh
ros2 launch view.launch.py
```

**TERMINAL 2**

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**TERMINAL 3**

```sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### 3.2 Paquete "tfg_actions"

```sh
cd tfg_girs_ws/
```

**TERMINAL 1**

```sh
ros2 run tfg_actions count_until_server
```

**TERMINAL 2**

```sh
ros2 run tfg_actions count_until_client
```

### 3.3 Paquete "tfg_interfaces"

```sh
cd tfg_girs_ws/
```

```sh
ros2 interface show tfg_interfaces/action/CountUntil
```

### 3.4 Paquete "tfg_package"

```sh
cd tfg_girs_ws/
```

**TERMINAL 1**

```sh
ros2 run tfg_package versionX_node
```

**TERMINAL 2**

```sh
ros2 run tfg_package versionX_subscriber
```

**TERMINAL 3**

```sh
ros2 topic echo /effector_coordinates
```

**NOTA**: Este comando se debe ejecutar cuando se está probando cualquiera de las 3 versiones de los nodos (version1, version2 o version3).

**TERMINAL 4**

```sh
ros2 topic echo /cable_parameters
```

**NOTA**: Este comando se debe ejecutar cuando se está probando cualquiera de las 3 versiones de los nodos (version1, version2 o version3).

**TERMINAL 5**

```sh
ros2 topic echo /pulley_parameters
```

**NOTA**: Este comando se debe ejecutar cuando se están probando las versiones version2 o version3 de los nodos.
