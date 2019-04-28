# Gazebo ROS Double Pendulum

A simple example of simulating a (balancing) double pendulum with a foot. Based on the [Gazebo ROS Demos](https://github.com/ros-simulation/gazebo_ros_demos).

## Tutorials

[ROS URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)

## Quick Start

Rviz:

    roslaunch rrbot_description rrbot_rviz.launch

Gazebo:

    roslaunch rrbot_gazebo rrbot_world.launch

ROS Control:

    roslaunch rrbot_control rrbot_control.launch

Example of Moving Joints:

    rostopic pub /rrbot/joint2_position_controller/command std_msgs/Float64 "data: -0.9"

