# ME570-Final-Project
ME 570 Final Project

## Tutorials
The tutorials used to create this package are listed below
- https://youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&si=Fj2g6RRPw6FBHq-R
- https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html

## required installations
-  ros-humble-gazebo-ros-pkgs
-  ros-humble-xacro (I think)
-  there was one more but I forgot

## How to use it
mkdir somewhere (ex. ros2_ws). I think the root is usually the place.
mkdir src inside that directory (Now the structure should be like ros2_ws/src)
breaing_formation_control directory should be in the src directory.

build the package in the workspace directory (e.g. ros2_ws)
- colcon build --symlink-install

source it
- source install/setup.bash

run one of these commands
- ros2 launch bearing_formation_control sim_gazebo.launch.py
- ros2 launch bearing_formation_control sim_gazebo.launch.py world:=./src/bearing_formation_control/worlds/random.world

This launch file should launch gazebo and rviz to visualize.

To drive it around using teleop key, run this command. (Note that the topic is not /cmd_vel)
- ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=diff_cont/cmd_vel_unstamped
