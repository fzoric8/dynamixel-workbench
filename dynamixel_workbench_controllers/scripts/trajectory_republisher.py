#!/usr/bin/env python
import rospy
import numpy as np  
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

class TrajectoryRepublisher():

    def __init__(self):

        self.run_ready = False
        self.ready = False
        self.trajectory_reciv = False

        self.trajectory_sub_name = "arm_controller/follow_joint_trajectory/goal"
        self.trajectory_pub_name = "joint_trajectory"
        self.ready_sub_name = "ready"
        self.joint_states_sub_name = "joint_states"

        self._init_publishers(); 
        self._init_subscribers(); 
        self.run_ready = True
        self.first_run = True
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        rospy.loginfo("Initialized Trajectory Republisher")

    def _init_subscribers(self):
        self.trajectory_sub = rospy.Subscriber(self.trajectory_sub_name, FollowJointTrajectoryActionGoal, self.trajectory_cb)
        self.ready_sub = rospy.Subscriber(self.ready_sub_name, Bool, self.ready_cb)
        self.joint_states_sub = rospy.Subscriber(self.joint_states_sub_name, JointState, self.joint_states_cb)

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

    def joint_states_cb(self, msg):
         
         self.joint_states = msg 

    def homing(self):

         joint_positions = self.joint_states.position
         homing_trajectory = self.create_homing_trajectory(joint_positions, 50, 0.1)
         self.trajectory_pub.publish(homing_trajectory)
    
    def create_homing_trajectory(self, curr_joint_positions, num_meas, period):
         
        cmd_positions = []
        trajectory_msg = JointTrajectory() 
        for joint in curr_joint_positions:
            cmd_positions.append(list(np.linspace(joint, 0, num_meas)))

        i = 0
        for q1, q2, q3, q4 in zip(cmd_positions[0], cmd_positions[1], cmd_positions[2], cmd_positions[3]): 
            i+=1
            trajectory_msg.points.append(JointTrajectoryPoint(positions=[q1, q2, q3, q4], time_from_start=rospy.Duration(i*period)))

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = trajectory_msg.points

        return trajectory

    def run(self): 

        while not rospy.is_shutdown(): 

            if self.run_ready: 
                
                # Homing
                if self.first_run and self.ready: 
                    rospy.loginfo("Arm is homing!")
                    self.homing()
                    self.first_run = False
                    self.ready = False

                # Execute trajectory from MoveIt!
                if self.trajectory_reciv and self.ready:  
                    self.trajectory_pub.publish(self.trajectory_msg)
                    self.ready = False
                    self.trajectory_reciv = False

                rospy.sleep(0.1)


if __name__ == "__main__": 
    
        rospy.init_node("trajectory_republisher") 
        tRp = TrajectoryRepublisher() 
        tRp.run()


