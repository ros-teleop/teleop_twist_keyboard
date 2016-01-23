# teleop_twist_keyboard
Generic Keyboard Teleop for ROS
#Launch
To run: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

#Usage
```
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

For Holonomic mode (strafing), hold down the shift key:
 U  I  O
 J  K  L
 M  <  >

For vertical movement, use t for up and b for down, at the same speed.

Holonomic mode and vertical mode both use the linear speed setting.

CTRL-C to quit
```

