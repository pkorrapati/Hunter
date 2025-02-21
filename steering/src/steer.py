#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JoyFeedback

# Move to a YAML file if possible
max_lin_vel = 1.5
max_rot = 0.461

max_lin_acc = 0.2
max_rot_acc = 0.2

# axes
ACCEL_INDX = 2
BRAKE_INDX = 3
STEER_INDX = 0

# buttons
UP_SHFT_INDX = 4
DN_SHFT_INDX = 5


def limit(value, lLim, uLim):
    return min(max(lLim, value), uLim)

class Steer:
    def __init__(self):        
        rospy.init_node('joy_map', anonymous=True)
        
        self.vel = Twist()
        self.rev = False # Reverse

        # Subscribers
        self.sub_input = rospy.Subscriber('/joy', Joy, self.mobilize)        
    
        # Publishers
        self.pub_motion = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def mobilize(self, data):
        
        if data.buttons[UP_SHFT_INDX]:
            self.rev = False
        elif data.buttons[DN_SHFT_INDX]:
            self.rev = True

        acc = limit((data.axes[ACCEL_INDX] + 1) * 0.5, 0.0, 1.0) * pow(-1, self.rev)
        brk = limit((data.axes[BRAKE_INDX] + 1) * 0.5, 0.0, 1.0)
        str = limit(data.axes[STEER_INDX] * 1, -1.0, 1.0)

        self.vel.linear.x = acc
        self.vel.angular.z = str*max_rot
        
        self.pub_motion.publish(self.vel)     
        

if __name__ == '__main__':
    try:
        sNode = Steer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass