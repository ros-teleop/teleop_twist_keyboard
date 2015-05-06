#teleop_twist_keyboard
Generic Keyboard Teleop for ROS

#Launch
To run: 

````
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch teleop_twist_keyboard teleop_keyboard.launch
````

#Usage

````
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
````
