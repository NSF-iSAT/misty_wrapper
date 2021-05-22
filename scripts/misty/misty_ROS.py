#!/usr/bin/env python
import websocket
import rospy

from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo

import mistyPy
from isat_robot_control.msg import MoveArms, MoveHead

class MistyNode:
    def __init__(self, idx=0, ip=None):
        self.ip = ip
        if rospy.get_param("/misty_ROS/use_robot") == True:
            if ip is None:
                self.ip = rospy.get_param("/misty_ROS/robot_ip")
            assert self.ip, "Node failed to launch in use_robot mode; provide a valid Misty IP"
            print('IP: ', self.ip)
            self.robot = mistyPy.Robot(self.ip)


        # SUBSCRIPTIONS
        self.speech_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/speech", String, self.speech_cb)
        self.arms_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/arms", MoveArms, self.arms_cb)
        self.head_usb = rospy.Subscriber("/misty/id_" + str(idx) + "/head", MoveHead, self.head_cb)

        rospy.init_node("misty_" + str(idx), anonymous=False)

        # while not rospy.is_shutdown():
        #     # loop: do something with current cam stream, publish frames, etc.
        #     latest_frame = self.vid_cb()
        #     if latest_frame:
        #         img_msg = self.bridge.cv2_to_imgmsg(latest_frame, encoding="passthrough")
        #         self.cam_pub.publish(img_msg)
        rospy.spin()

        self.cleanup()


    def vid_cb(self):
        latest_frame = self.vid_stream.frame
        return latest_frame

    def speech_cb(self, msg):
        self.robot.speak(msg.data)

    def arms_cb(self, msg):
        left_arm_msg = msg.leftArm
        right_arm_msg = msg.rightArm

        self.robot.moveArms(right_arm_msg.value, left_arm_msg.value, right_arm_msg.velocity, left_arm_msg.velocity, 
            units=msg.units)

    def head_cb(self, msg):
        self.robot.moveHead(msg.roll, msg.pitch, msg.yaw, msg.velocity, units=msg.units)

    def cleanup(self):
        # self.vid_stream.stop()
        # self.robot.stopAvStream()
        pass

MistyNode()