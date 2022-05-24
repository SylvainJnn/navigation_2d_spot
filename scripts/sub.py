#!/usr/bin/env python

import rospy
import roslib
import math
import copy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
from spot_msgs.msg import BatteryStateArray

class TestBatteryStates():


    def __init__(self):
        rospy.init_node("monitor_battery_states")
        self.t = rospy.get_rostime()
        self.charge = None
        self.initial_charge = None
        rospy.Subscriber('/spot/status/battery_states', BatteryStateArray, self.batterySatate)
        rospy.spin()



    def batterySatate(self, msg):
        if self.charge is None:
            self.initial_charge = msg.battery_states[0].charge_percentage
	self.charge = msg.battery_states[-1].charge_percentage
        time_passed = rospy.get_rostime() - self.t
        if (self.initial_charge - self.charge) != 0:
            change = (self.initial_charge - self.charge)/time_passed.secs
            print("Charge drop: " + str(change))
        print("Time passed: " + str(time_passed.secs))
        print("Battery procentage: " + str(self.charge))
        
        #rospy.sleep(2)
    

if __name__ == '__main__':
    """ main """
    try:
        test_acc = TestBatteryStates()
    except rospy.ROSInterruptException:
        passs
