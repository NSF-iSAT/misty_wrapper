#!/usr/bin/env python

import rospy
import mistyPy
from std_msgs.msg import Float32MultiArray

class MistyROSNode:
    def __init__(self):
        self.robot = mistyPy.Robot(rospy.get_param('robot_ip')) if rospy.get_param('use_robot', default=False) else None

        rospy.init_node('mistyROS', anonymous=True, log_level=rospy.DEBUG)
        self.head_sub = rospy.Subscriber("mistyROS/move_head", Float32MultiArray, self.head_cb) 

        rospy.spin()

    def head_cb(self, msg):
        # NOTE: Currently using raw roll/pitch/yaw format. The transformations.py pkg has
        # utlities for conversion to/from quaternions<->euler rpy if this is needed.
        # Also consider stamping header info onto 
        rospy.debuginfo("received cb to move head with RPY value %f, %f, %f", msg[0], msg[1], msg[2])
        if self.robot:
            self.robot.moveHead(*msg)

if __name__ == "__main__":
    try:
        MistyROSNode()
    except rospy.ROSInterruptException:
        pass