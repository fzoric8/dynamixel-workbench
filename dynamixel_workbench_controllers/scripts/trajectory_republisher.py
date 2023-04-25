#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Bool

class TrajectoryRepublisher():

    def __init__(self):

        self.run_ready = False
        self.trajectory_reciv = False

        self.trajectory_sub_name = "arm_controller/follow_joint_trajectory/goal"
        self.trajectory_pub_name = "joint_trajectory"
        self.ready_sub_name = "ready"

        self._init_publishers(); 
        self._init_subscribers(); 
        self.run_ready = True
        rospy.loginfo("Initialized Trajectory Republisher")

    def _init_subscribers(self):
        self.trajectory_sub = rospy.Subscriber(self.trajectory_sub_name, FollowJointTrajectoryActionGoal, self.trajectory_cb)
        self.ready_sub = rospy.Subscriber(self.ready_sub_name, Bool, self.ready_cb)

    def _init_publishers(self): 
        self.trajectory_pub = rospy.Publisher(self.trajectory_pub_name, JointTrajectory, queue_size=1)

    def trajectory_cb(self, msg):
         
         self.trajectory_reciv = True
         self.trajectory_msg = JointTrajectory()
         rospy.loginfo("Type of the trajectory is: {}".format(type(msg.goal.trajectory)))
         self.trajectory_msg.points = msg.goal.trajectory.points
         self.trajectory_msg.joint_names = msg.goal.trajectory.joint_names

         rospy.loginfo("Recieved trajectory: {}".format(self.trajectory_msg))

    def ready_cb(self, msg): 
         
         self.ready = msg.data
         rospy.loginfo("Recieved ready: {}".format(self.ready))

    def run(self): 

        while not rospy.is_shutdown(): 

            if self.run_ready: 

                if self.trajectory_reciv and self.ready:  
                    self.trajectory_pub.publish(self.trajectory_msg)
                    self.ready = False
                    self.trajectory_reciv = False

                rospy.sleep(0.1)


if __name__ == "__main__": 
    
        rospy.init_node("trajectory_republisher") 
        tRp = TrajectoryRepublisher() 
        tRp.run()


