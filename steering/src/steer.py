#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JoyFeedback

# Move to a YAML file if possible
MAX_LIN_VEL = 1.5
MAX_STEER = 0.461

max_lin_acc = 0.2
max_rot_acc = 0.2

# axes
STEER_INDX = 0
CLUCH_INDX = 1
ACCEL_INDX = 2
BRAKE_INDX = 3

# buttons
UP_SHFT_INDX = 4
DN_SHFT_INDX = 5

# Returns a value between lLim and uLim
def limit(value, lLim, uLim):
    return min(max(lLim, value), uLim)

class Steer:
    def __init__(self):        
        rospy.init_node('joy_map', anonymous=True)
        
        self.STOP = Twist()     # All zeros for emergency stop
        self.vel = Twist()      # Actual control commands
        self.rev = False # Reverse

        # Subscribers
        self.sub_input = rospy.Subscriber('/joy', Joy, self.mobilize)       # Subscribe to Joy        
    
        # Publishers
        self.pub_motion = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.pub_connect = rospy.Publisher('/alive', Bool, queue_size=1)

    def mobilize(self, data):        
        if data.buttons[UP_SHFT_INDX]:
            self.rev = False                # On upshift, set motion to forward direction

        elif data.buttons[DN_SHFT_INDX]:
            self.rev = True                 # On downshift, set motion to reverse direction

        acc = limit((data.axes[ACCEL_INDX] + 1) * 0.5, 0.0, 1.0) * pow(-1, self.rev)
        brk = limit((data.axes[BRAKE_INDX] + 1) * 0.5, 0.0, 1.0)
        clt = limit((data.axes[CLUCH_INDX] + 1) * 0.5, 0.0, 1.0)

        d = limit(data.axes[STEER_INDX] * 1, -1.0, 1.0)

        self.vel.linear.x = acc
        self.vel.angular.z = d * MAX_STEER
        
        self.pub_motion.publish(self.vel)     
        
if __name__ == '__main__':
    try:
        sNode = Steer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass