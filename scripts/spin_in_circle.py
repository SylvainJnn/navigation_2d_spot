#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_circle():
    rospy.init_node('spin_in_circle', anonymous=True)

    # Create a publisher which can "talk" to Turtlesim and tell it to move
    pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=1)
     
    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.angular.z = 0.5 #1.0

    # Save current time and set publish rate at 10 Hz
    now = rospy.Time.now()
    rate = rospy.Rate(10)

    # For the next 6 seconds publish cmd_vel move commands to Turtlesim
    #while rospy.Time.now() < now + rospy.Duration.from_sec(300):
    while rospy.Time.now() < (now + rospy.Duration.from_sec(9)):
    #while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()
    move_cmd = Twist()
    move_cmd.angular.z = 0.0 #1.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass
