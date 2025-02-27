#!/usr/bin/env python3
# sudo apt-get install libasio-dev
import rospy

from std_msgs.msg import Bool, Int8, Int16
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

# gear index
MIN_GEAR_IDX = 0
MAX_GEAR_IDX = 3
REV_GEAR_IDX = 0 # Reverse
PAR_GEAR_IDX = 1 # Park
ONE_GEAR_IDX = 2 # Slow Speed
TWO_GEAR_IDX = 3 # Fast Speed

# Returns a value between lLim and uLim
def limit(value, lLim, uLim):
    return min(max(lLim, value), uLim)

class Steer:
    def __init__(self):        
        rospy.init_node('steer', anonymous=True)
        
        self.STOP = Twist()     # All zeros for emergency stop
        self.vel = Twist()      # Actual control commands
        self.selectedGear = Int8()

        # self.rev = False # Reverse
        
        self.gear = PAR_GEAR_IDX
        self.selectedGear.data = self.gear
        self.buttonHigh = False

        # Subscribers
        self.sub_pulse = rospy.Subscriber('/pulse', Int16, self.mobilize)
        self.sub_input = rospy.Subscriber('/joy', Joy, self.input)       # Subscribe to Joy                
    
        # Publishers
        self.pub_motion = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_gear = rospy.Publisher('/gear', Int8, queue_size=2)

        self.pub_gear.publish(self.selectedGear)

    def mobilize(self, data):
        self.pub_motion.publish(self.vel)     
    
    def input(self, data):        
        if data.buttons[UP_SHFT_INDX] and not self.buttonHigh:
            self.buttonHigh = True
            self.gear = int(limit(self.gear + 1, MIN_GEAR_IDX, MAX_GEAR_IDX))
            self.selectedGear.data = self.gear
            self.pub_gear.publish(self.selectedGear)
            # self.rev = False                # On upshift, set motion to forward direction

        elif data.buttons[DN_SHFT_INDX] and not self.buttonHigh:
            self.buttonHigh = True
            self.gear = int(limit(self.gear - 1, MIN_GEAR_IDX, MAX_GEAR_IDX))
            self.selectedGear.data = self.gear
            self.pub_gear.publish(self.selectedGear)
            # self.rev = True                 # On downshift, set motion to reverse direction
        
        elif not data.buttons[UP_SHFT_INDX] and not data.buttons[DN_SHFT_INDX] and self.buttonHigh:
            self.buttonHigh = False

        acc = limit((data.axes[ACCEL_INDX] + 1) * 0.5, 0.0, 1.0) * pow(-1, self.rev)
        brk = limit((data.axes[BRAKE_INDX] + 1) * 0.5, 0.0, 1.0)
        clt = limit((data.axes[CLUCH_INDX] + 1) * 0.5, 0.0, 1.0)

        d = limit(data.axes[STEER_INDX] * 1, -1.0, 1.0)

        mul = 0

        if self.gear == REV_GEAR_IDX:
            mul = -1.0
        elif self.gear == ONE_GEAR_IDX:
            mul == 2 * MAX_LIN_VEL / 3
        elif self.gear == TWO_GEAR_IDX:
            mul == MAX_LIN_VEL

        self.vel.linear.x = acc * mul
        self.vel.angular.z = d * MAX_STEER
        
if __name__ == '__main__':
    try:
        sNode = Steer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass