
import rospy
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool

class TrajectoryRepublisher():

    def __init__(self):

        self.run_ready = False
        self.trajectory_reciv = False
        self._init_publishers(); 
        self._init_subscribers(); 
        self.run_ready = True

        self.trajectory_sub_name = "arm_controller/follow_joint_trajectory/goal"
        self.trajectory_pub_name = "joint_trajectory"
        self.ready_sub_name = "ready"

    def _init_subscribers(self):
        self.trajectory_sub = rospy.Subscriber(self.trajectory_sub_name, JointTrajectory, self.trajectory_cb)
        self.ready_sub = rospy.Subscriber(self.ready_sub_name, Bool, self.ready_callback)

    def _init_publishers(self): 
        self.trajectory_pub = rospy.Publisher(self.trajectory_pub_name, JointTrajectory, queue_size=1)

    def trajectory_cb(self, msg):
         
         self.trajectory_reciv = True
         self.trajectory_msg = msg.goal.trajectory
         rospy.loginfo("Recieved trajectory: {}".format(self.trajectory_msg))

    def ready_cb(self, msg):
         
         self.ready = msg.data

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


