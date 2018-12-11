#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, TwistStamped

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to TwistStamped!
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

For Holonomic mode (strafing), hold down the shift key:
---------------------------
        8     
   4    5    6
        2    

7 : roll left (-x)
9 : roll right (+x)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        # [Translation] Basic moving mode 
        'i':(1,0,0,0,0,0),
        'o':(1,0,0,0,0,-1),
        'j':(0,0,0,0,0,1),
        'l':(0,0,0,0,0,-1),
        'u':(1,0,0,0,0,1),
        ',':(-1,0,0,0,0,0),
        '.':(-1,0,0,0,0,1),
        'm':(-1,0,0,0,0,-1),
        # [Translation] Holonomic moving mode
        'O':(1,-1,0,0,0,0),
        'I':(1,0,0,0,0,0),
        'J':(0,1,0,0,0,0),
        'L':(0,-1,0,0,0,0),
        'U':(1,1,0,0,0,0),
        '<':(-1,0,0,0,0,0),
        '>':(-1,-1,0,0,0,0),
        'M':(-1,1,0,0,0,0),
        # [Translation] height change
        't':(0,0,1,0,0,0),
        'b':(0,0,-1,0,0,0),
        # [Orientation]
        '8':(0,0,0,0,1,0),
        '2':(0,0,0,0,-1,0),
        '4':(0,0,0,0,0,1),
        '6':(0,0,0,0,0,-1),
        '7':(0,0,0,-1,0,0),
        '9':(0,0,0,1,0,0),         
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('CartesianVelocityMove', TwistStamped, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.01)
    turn = rospy.get_param("~turn", 0.05)
    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                roll = moveBindings[key][3]
                pitch = moveBindings[key][4]
                yaw = moveBindings[key][5]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                roll = 0
                pitch = 0
                yaw = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = roll*turn; twist.angular.y = pitch*turn; twist.angular.z = yaw*turn
            twist_pub = TwistStamped()
            twist_pub.twist = twist
            pub.publish(twist_pub)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        twist_pub = TwistStamped()
        twist_pub.twist = twist
        pub.publish(twist_pub)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
