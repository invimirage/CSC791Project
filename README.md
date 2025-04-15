# ROS2-CARLA LiDAR Integration Project

## Prerequisites

This project requires:
- **Operating System:** Ubuntu 22.04
- **ROS Version:** ROS 2 Humble
- **CARLA Version:** 0.9.14
- **Python Version:** 3.10

---

## Building ROS Nodes

### 1. ROS-CARLA Bridge

```bash
cd workspace/src
git clone --recurse-submodules https://github.com/gezp/carla_ros.git -b humble-carla-0.9.14
cd ..
rosdep install --from-paths src --ignore-src -r
pip3 install pygame
colcon build --symlink-install
source install/setup.bash
```

### 2. LiDAR-to-CARLA Node

```bash
cd lidar_to_carla/src
ros2 pkg create lidar_to_carla --build-type ament_python --dependencies rclpy sensor_msgs std_msgs
colcon build --symlink-install
source install/setup.bash
```

### Setup Environment Variables

```bash
export PYTHONPATH=$PYTHONPATH:<YOUR_PATH>/carla-0.9.14-py3.10-linux-x86_64.egg
export PYTHONPATH=<YOUR_PATH>/PythonAPI/carla:$PYTHONPATH
pip install -r requirements.txt
```

---

## Execution Steps

### 1. Connect LiDAR Device (Windows 11 with WSL)

Run the following commands in PowerShell as Administrator:

```powershell
usbipd list
usbipd bind --busid <port_id>
usbipd attach --busid <port_id> --wsl
```

In WSL Ubuntu terminal:

```bash
lsusb
ls /dev/ttyUSB*
```

Launch ROS node for RPLiDAR:

```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py serial_port:=/dev/ttyUSB<id> serial_baudrate:=115200
```

Check LiDAR data:

```bash
ros2 topic echo /scan
```

---

### 2. Launch LiDAR-to-CARLA Bridge

```bash
ros2 launch lidar_to_carla lidar_bridge_launch.py
```

Test detection by placing a metal object about 30cm from the LiDAR:

```bash
ros2 topic echo /detected/obstacle_relative_position
```

---

### 3. Launch CARLA and Demo Agent

#### 3.1 Remote CARLA Server and VNC

On the remote server (e.g., c25):

```bash
export LIBGL_DRIVERS_PATH="/usr/lib64/dri"
export PATH=$PATH:/opt/TurboVNC/bin
export LC_ALL=en_US.UTF-8
vncpasswd # First-time setup
vncserver -depth 24 -geometry 1680x1050 :1
```

On your local machine (Windows terminal):

```bash
ssh arc -L 5901:c25:5901
vncviewer localhost:5901
```

Alternatively, open in your browser:
```
vnc://localhost:5901
```

Inside the remote VNC session terminal:

```bash
cd hw4/p5
export UE4_ROOT=~fmuelle/carla-9.14/UnrealEngine_4.26/
export PYTHONPATH=~fmuelle/carla-9.14/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg
export LD_LIBRARY_PATH=`pwd`:$LD_LIBRARY_PATH
pip3 install --user --upgrade pip
pip3 install --user pygame==2.5.1
sh ~fmuelle/carla-9.14/CarlaUE4.sh -fps=10 -benchmark-quality-level=Low &
```

Establish SSH tunnels to the remote CARLA server from WSL:

```bash
ssh -i <path_to_ssh_pub_key> uid@arc.csc.ncsu.edu -L 2000:cxx:2000 -L 2001:cxx:2001
```

#### 3.2 Run CARLA Autonomous Driving Demo

```bash
ros2 launch carla_ad_demo carla_ad_demo.launch.py
```

A GUI should open locally. In the VNC window, observe the CARLA map loading and the ego vehicle (Tesla) beginning to move.

Place a metallic object 15-50cm in front of the LiDAR sensor to observe the appearance of an obstacle car. Moving or removing the object updates or removes the obstacle car accordingly.

---

**Author: Ruifeng Zhang**

