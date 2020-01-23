#!/usr/bin/env python
import time
import rospy
import math
from darwin_gazebo.darwin import Darwin
# from ros_cqi.ros_cqi.darwin_interface import DarwinInterface

if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Darwin Client")
    darwin=Darwin()
    rospy.sleep(1) 
    rospy.loginfo("Darwin Walker Demo Starting")

    darwin.apply_fallback_force()
    while True:
        head = darwin.get_head_position()
        angles = darwin.get_body_rotation()

        if (head.twist.linear.x > 0.01):
            darwin.move_arms_straight()
            darwin.move_low_arm(angles[1])
            darwin.move_tibia(angles[1])

    rospy.loginfo("Darwin Walker Demo Finished")
