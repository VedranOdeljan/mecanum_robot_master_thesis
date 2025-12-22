# mecanum_robot_master_thesis
Code for a mecanum-wheeled robot developed as part of my master’s thesis, using ros2_control with the mecanum_drive_controller on an NVIDIA Jetson Xavier NX, deployed via Docker.


1. terminal:
cd ~/ros2_foxy_v2/
ros2 run mecanum_control mecanum_kinematics

2. terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
