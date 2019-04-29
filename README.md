# Gazebo ROS Double Pendulum

A simple example of simulating a (balancing) double pendulum with a foot. Based on the [Gazebo ROS Demos](https://github.com/ros-simulation/gazebo_ros_demos).

## Tutorials

[ROS URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)

## Quick Start

Rviz:

    roslaunch double_pendulum_description double_pendulum_rviz.launch

Gazebo:

    roslaunch double_pendulum_gazebo double_pendulum_world.launch

ROS Control:

    roslaunch double_pendulum_control double_pendulum_control.launch

Example of Moving Joints:

    rostopic pub /double_pendulum/joint1_torque_controller/command std_msgs/Float64 "data: 5"

