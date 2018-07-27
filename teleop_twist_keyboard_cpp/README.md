# teleop_twist_keyboard_cpp
C++ Implementation of the Generic Keyboard Teleop for ROS: https://github.com/ros-teleop/teleop_twist_keyboard

## Features

This particular implementation does away with keeping the history of previous speed settings, and heavily cuts down on the amount of printing that is done to the terminal via the use of carriage returns (\r).

Furthermore, the last command that was sent is reflected, and invalid commands are identified as such.



## Installing the Package

As per standard ROS practice, make a workspace, go to the workspace's src directory, and clone this repository, then run catkin_make in the root of the workspace, and source the resulting setup.bash!

```bash
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
$ cd ..
$ catkin_make

$ source devel/setup.bash
```



## Running the Node

```bash
# In one terminal, run
$ roscore

# In another terminal, run
$ rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard

# If you want to see the outputs, check the /cmd_vel topic
$ rostopic echo /cmd_vel
```



## Usage

Same as the original

```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```



------

Author: [methylDragon](https://github.com/methylDragon)

[![Yeah! Buy the DRAGON a COFFEE!](./.assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)