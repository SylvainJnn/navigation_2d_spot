#! /usr/bin/env python3
import rospy

from spot_msgs.msg import TrajectoryAction, TrajectoryResult, TrajectoryFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration

from geometry_msgs.msg import Point 
from geometry_msgs.msg import Quaternion 
import actionlib
import spot_msgs.msg 

from tf.transformations import quaternion_from_euler
import numpy
import tf

from navigation_2d_spot.srv import GoalSpotTrajectory, GoalSpotTrajectoryResponse



class MoveTrajectoryCMD():

    def __init__(self):
        self.tf_listener = None      # Tf listener
        self.frame_to = None         # Frame the fiducial will be transformed to ("map")
        self.x = 0.0
        self.y = 0.0   # Meter
        self.yaw = 0.0 # Degrees
        self.converted_transform = None

            
    def main(self):
        rospy.init_node('move_trajectory_cmd')
        self.tf_listener = tf.TransformListener()  

        self.client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryAction)
        print("waiting for server")

        self.client.wait_for_server()  
        print("server received")
        self.trajectory_goal = spot_msgs.msg.TrajectoryGoal()

        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'body'
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.pose.position.x = 1.0
        self.goal_msg.pose.position.y = 0.0
        self.goal_msg.pose.orientation.z = 0.0
        self.goal_msg.pose.orientation.w = 1.0

        #q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        #p.pose.orientation = Quaternion(*q)

        #rospy.Duration(req.duration.data.secs, req.duration.data.nsecs)
        duration =  rospy.Duration(10)


        self.trajectory_goal.target_pose = self.goal_msg
        self.trajectory_goal.duration.data.secs = duration.secs
        self.trajectory_goal.duration.data.nsecs = duration.nsecs
        self.trajectory_goal.precise_positioning = True
        

        print(self.trajectory_goal)

        # x, y , yaw in degrees, duration, acuracy
  

        self.client.send_goal(self.trajectory_goal)
        #/trajectory/goal
        

        # bool precise_positioning

        

        #q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        #p.pose.orientation = Quaternion(*q)
       # rospy.Service('/save_fiducials', Empty, self.save_fiducials)
       # rospy.Subscriber('/spot/fiducial', FiducialLocalization, self.save_fiducials_to_list        self.goal_msg = PoseStamped()
        


        rospy.spin()



if __name__ == "__main__":
    SR = MoveTrajectoryCMD()
    SR.main()
