# Fairino_sim
[![license - MIT](https://img.shields.io/:license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple.svg)](https://index.ros.org/doc/ros2/Releases/)

# Note

# Build
### *This Package is implemented at ROS2-Jazzy.*
```
### I assume that you have installed the ros-jazzy-desktop package using the apt-get command.
### I recommand the /home/<user_home>/fairino_ws/src
### Before activate simulation, please install moveit2 and gazebo-harmonic



$ mkdir -p ~/fairino_ws/src
$ cd ~/fairino_ws/src
$ git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git
$ git clone https://github.com/jjh1214/fairino_sim.git

$ cd ~/fairino_ws
$ colcon build
$ . install/setup.bash
$ source fairino_ws/install/local_setup.bash



### if you want
# $ echo 'source ~/fairino_ws/install/local_setup.bash' >> ~/.bashrc 
```

# GZ Launch
## Default Model : fairino5_v6
```
ros2 launch fairino_sim fairino_gz.launch.py
```
## Other Models
### *NOTE : fairino3_mt_v6 is not supported.*
```
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino3_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino10_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino16_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino20_v6
ros2 launch fairino_sim fairino_gz.launch.py robot_name:=fairino30_v6

```

# Move Model
```
ros2 topic pub -1 /frcobot_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0, -1.571, -1.571, -1.571, -1.571, -1.571]}"

ros2 topic pub -1 /frcobot_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
# Move example

[Screencast from 2025-07-31 09-28-38.webm](https://github.com/user-attachments/assets/c4b7654f-bd6c-4130-88a2-4ee26a23a3ae)

# GZ with Moveit Launch
## Default Model : fairino5_v6
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py
```
## Other Models
### *NOTE : fairino3_mt_v6 is not supported.*
```
ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino3_v6

ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino10_v6

ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino16_v6

ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino20_v6

ros2 launch fairino_sim fairino_gz_moveit2.launch.py robot_name:=fairino30_v6

```

# Move example

[Screencast from 2025-07-31 09-30-59.webm](https://github.com/user-attachments/assets/f1d53846-4953-4821-be4b-f542406576af)

