# Ball-Chaser-Robot
This project contains a mobile robot and the world it resides in created using Gazebo and URDF. Also, the C++ nodes in ROS to chase white colored balls.
## How to Launch the simulation?
### Clone the package in your repositry
```sh
$ cd <your repository> ### or skip if you are already there
$ git clone https://github.com/IsmailAlr/Ball-Chaser-Robot.git
```
### Initiate a workspace
```sh
$ cd Ball-Chaser-Robot/src
$ catkin_init_workspace
$ cd ..
```
### Build the ball_chaser package
```sh
$ cd /Ball-Chaser-Robot/ 
$ catkin_make
```
### After building the package, source your environment
```sh
$ cd /Ball-Chaser-Robot/
$ source devel/setup.bash
```
### Make sure to check and install any missing dependencies
```sh
$ rosdep install -i ball_chaser
```
### Once the ball_chaser package has been built, you can launch the simulation environment using
```sh
$ roslaunch my_robot world.launch
```
## ROS nodes
### ROS Node: drive_bot
This server node provides a ball_chaser/command_robot service to drive the robot around by setting its linear x and angular z velocities. The service server publishes messages containing the velocities for the wheel joints.  
with this node, we will be able to request the ball_chaser/command_robot service, either from the terminal or from a client node, to drive the robot by controlling its linear x and angular z velocities.
#### Test created DriveToTarget.srv
Open a new terminal and execute the following:
```sh
$ cd /Ball-Chaser-Robot/ 
$ source devel/setup.bash
$ rossrv show DriveToTarget
```
we should receive this response:
```sh
[ball_chaser/DriveToTarget]:
float64 linear_x
float64 angular_z
---
string msg_feedback
```
#### Test drive_bot.cpp

To test if the service, first launch the robot inside the world. Then call the /ball_chaser/command_robot service to drive the robot forward, left, and then right.

##### 1- Launch the robot inside your world
```sh
$ cd /Ball-Chaser-Robot/ 
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
##### 2- Request a ball_chaser/command_robot service

Test the service by requesting different sets of velocities from the terminal.

Open a new terminal while all the nodes are running and type:
```sh
$ cd /Ball-Chaser-Robot/ 
$ source devel/setup.bash

$ rosservice call /ball_chaser/command_robot "linear_x: 0.5
angular_z: 0.0"  # This request should drive your robot forward

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.5"  # This request should drive your robot left

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: -0.5"  # This request should drive your robot right

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.0"  # This request should bring your robot to a complete stop
```
### ROS Node: process_image

The second node in this project is the process_image node. This client node will subscribe to the robot’s camera images and analyze them to determine the position of the white ball. Once the ball position is determined, the client node will request a service from the drive_bot server node to drive the robot toward the ball. The robot can drive either left, right or forward, depending on the robot position inside the image.

#### Build Package

compile it with:
```sh
$ cd /Ball-Chaser-Robot/
$ catkin_make
```

#### Test process_image

To test, first launch the robot inside your world and then run both the drive_bot and process_image nodes.

##### 1- Launch the robot inside your world

This can be done by launching the world.launch file:
```sh
$ cd /Ball-Chaser-Robot/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
##### 2- Run drive_bot and process_image

This can be done by executing ball_chaser.launch:
```sh
$ cd /Ball-Chaser-Robot/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
##### 3- Visualize

To visualize the robot’s camera images, you can subscribe to camera RGB image topic from RViz. Or you can run the rqt_image_view node:
```sh
$ cd /Ball-Chaser-Robot/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view  
```
