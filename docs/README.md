# Micro-ROS Configuration and Execution using ESP-IDF

## Features

This README describes the procedure necessary to compile, upload, and run the **[int32_publisher](https://github.com/micro-ROS/micro_ros_espidf_component/tree/jazzy/examples/int32_publisher)** example included in the **[micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component/tree/jazzy)** using an **ESP32-S2 Mini** microcontroller and an agent (**micro-ROS Agent**), where much of this process is carried out within a Docker container.

The execution is performed using three independent terminals:

### Terminal 1

Responsible for:

- Running the ESP-IDF container.
- Downloading the micro-ROS component.
- Configuring the project.
- Compiling the firmware.
- Programming the ESP32-S2 Mini microcontroller.

### Terminal 2

Responsible for:

- Running the micro-ROS Agent.

### Terminal 3

Responsible for:

- Verifying ROS 2 communication.
- Inspecting topics and messages.

---

## Installation

### Prerequisites

- **ESP32-S2 Mini** microcontroller.
- **USB Type-C cable** for data transmission.
- **Docker** container.
- **ROS 2 Jazzy**.
- **Wi-Fi network access**.

---

## Setup

1. Launch the Docker corresponding to the micro-ros-espidf-component:

```sh
docker run --name micro-ros-espidf-component -it --rm \
-v /dev:/dev \
--privileged \
espressif/idf:release-v5.5 bash
```

2. Inside the Docker, download the micro-ros-espidf-component:

```sh
git clone -b jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git
```

3. Access the repository:

```sh
cd micro_ros_espidf_component/
```

4. Install all additional dependencies:

```sh
pip install catkin_pkg colcon-common-extensions lark
```

5. Access the **int32_publisher** example repository:

```sh
cd examples/int32_publisher
```

6. Configure the compilation process for the **ESP32-S2 Mini** microcontroller:

```sh
idf.py set-target esp32s2
```

If the compilation has finished successfully, a message like this should appear at the end:

```sh
-- Configuring done (530.8s)
-- Generating done (0.3s)
-- Build files have been written to: /micro_ros_espidf_component/examples/int32_publisher/build
```

7. Open the settings menu and enter the following information:

```sh
idf.py menuconfig
```

**Wi-Fi Network SSID**
- micro-ROS Settings > WiFi Configuration > WiFi SSID

**Wi-Fi Network Password**
- micro-ROS Settings > WiFi Configuration > WiFi Password

**IP address of the computer where the micro-ROS Agent will be run**
- micro-ROS Settings > micro-ROS Agent IP

**NOTE**: The IP address used both in the microcontroller configuration and when launching the agent is the one underlined in the image below, and can be found in the output of the `ip addr` command. This address will obviously vary for each computer.

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/ip_addr_output.png?raw=true">
</p>

---

## Usage

1. Compile the project:

```sh
idf.py build
```

If the compilation has finished successfully, a message like this should appear at the end:

```sh
Project build complete. To flash, run:
 idf.py flash
or
 idf.py -p PORT flash
```

2. Open **Terminal 2** and start the micro-ROS Agent:

```sh
docker run -it --rm --net=host \
microros/micro-ros-agent:jazzy \
udp4 --port 8888 --address <ip> -v6
```

**NOTE**: `<ip>` is the IP address of the computer where the agent is running (the same one that was configured on the microcontroller).

<p align="center">
  <img src="https://github.com/aleon2020/cable_driven_parallel_robot_2d/blob/main/media/images/microros_agent_output.png?raw=true">
</p>

3. Return to **Terminal 1** and flash the program to the microcontroller:

```sh
idf.py -p /dev/ttyACM0 flash
```

**IMPORTANT**: If this command returns an error the first time, put the microcontroller into Bootloader mode (press and hold the BOOT and RESET buttons, release the RESET button and 2-3 seconds later release the BOOT button).

4. Open **Terminal 3** and verify that the available topics are the following:

```sh
ros2 topic list
```

```sh
/freertos_int32_publisher
/parameter_events
/rosout
```

5. Finally, to view the messages being published, you must run the following command in the terminal:

```sh
ros2 topic echo /freertos_int32_publisher
```

```sh
data: 101
---
data: 102
---
data: 103
---
data: 104
---
data: 105
---
...
```
