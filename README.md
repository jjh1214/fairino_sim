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
