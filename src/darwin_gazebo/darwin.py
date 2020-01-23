import random
from threading import Thread
import math
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench
from geometry_msgs.msg import Twist, Point, Wrench, Vector3
import math


class Darwin:
    """
    Client ROS class for manipulating Darwin OP in Gazebo
    """
    
    def __init__(self,ns="/darwin/"):
        """
        Default constructor, initializes joint state subscriber and service proxy to read parameters and 
        control darwin model.
        """
        self.ns=ns
        self.joints=None
        self.angles=None
        self.pan_index = 10
        
        self._sub_joints=rospy.Subscriber(ns+"joint_states",JointState,self._cb_joints,queue_size=1)
        self._sub_position = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self._wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench, True)
        rospy.loginfo("Waiting for joints to be populated...")
        while not rospy.is_shutdown():
            if self.joints is not None: break
            rospy.sleep(0.1)            
            rospy.loginfo("Waiting for joints to be populated...")
        rospy.loginfo("Joints populated")
        rospy.loginfo("Creating joint command publishers")
        self._pub_joints={}
        for j in self.joints:
            p=rospy.Publisher(self.ns+j+"_position_controller/command",Float64, queue_size=1)
            self._pub_joints[j]=p
        
        rospy.sleep(1)
        self._pub_cmd_vel=rospy.Publisher(ns+"cmd_vel",Twist)

    def move_low_arm(self, theta):
        """
        Sends command to left and right arm controllers setting specified angle (radius).
        Please not that left arm is set with minus theta to mirror behaviour of the right low arm.
        """
        self._pub_joints["j_low_arm_l"].publish(-theta)
        self._pub_joints["j_low_arm_r"].publish(theta)

    
    def get_head_position(self):
        """Returns position and twist of the darwin's head."""
        return self._sub_position("darwin", "MP_HEAD")

    def get_arm_position(self):
        """Returns position and twist of darwin's right arm."""
        return self._sub_position("darwin", "base_link")

    def move_arms_straight(self):
        """Sets both robots' arms straight (sets low arm and arm to pi/2 angle)./."""
        self.move_low_arm(3.14/2.0)
        self.move_arm_left(3.14/2.0)
        self.move_arm_right(3.14/2.0)

    def get_body_rotation(self):
        """
        Returns orientation of darwin's model in workspace in relation to base point (0,0,0).
        This function transforms quaternion to 3D vector of angles (in radian).
        """
        q = self._sub_position("darwin", "").pose.orientation
        x = math.atan2(2*(q.w*q.x+q.y*q.z), 1 - 2*(math.pow(q.x, 2) + math.pow(q.y, 2)))
        y = math.asin(2*(q.w*q.y - q.z*q.x))
        z = math.atan2(2*(q.w*q.z+q.y*q.x), 1 - 2*(math.pow(q.y, 2) + math.pow(q.z, 2)))
        return [x,y,z]

    def move_arm_left(self, theta):
        """
        Sends command to left arm controller to set up theta angle and then sends command to left 
        shoulder controller to set up -theta angle 
        """
        self._pub_joints["j_high_arm_l"].publish(theta)
        self._pub_joints["j_shoulder_l"].publish(-theta)

    def move_arm_right(self, theta):
        """
        Sends command to right arm controller to set up theta angle and then sends command to right 
        shoulder controller to set up theta angle 
        """
        self._pub_joints["j_high_arm_r"].publish(theta)
        self._pub_joints["j_shoulder_r"].publish(theta)

    def move_tibia(self, theta):
        """
        Sends command to right tibia controller to set up specified angle at the knee and sets left 
        tibia controller to mirror that position.
        """
        self._pub_joints["j_tibia_l"].publish(-theta)
        self._pub_joints["j_tibia_r"].publish(theta)

    def _cb_joints(self,msg):
        if self.joints is None:
            self.joints=msg.name
        self.angles=msg.position

    def apply_fallback_force(self):
        point = Point(0, 0, 0)
        wrench = Wrench(force=Vector3(10000, 0, 0), torque=Vector3(0, 0, 0))
        response = self._wrench_service("darwin::MP_HEAD", "", point, wrench, rospy.Time(1), rospy.Duration(0, 1000000))
