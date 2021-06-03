#!/usr/bin/env python3
import rospy


import mistyPy
from std_msgs.msg import String, Int8MultiArray
from isat_robot_control.msg import MoveArms, MoveHead

class MistyNode:
    def __init__(self, idx=0, ip=None):
        self.ip = ip
        if rospy.get_param("/misty_ROS_"+ str(idx) + "/use_robot") == True:
            if ip is None:
                while not self.ip:
                    self.ip = rospy.get_param("/misty/id_" + str(idx) + "/robot_ip")
                    rospy.sleep(1.0)
            print('IP: ', self.ip)
            self.robot = mistyPy.Robot(self.ip)


        # SUBSCRIPTIONS
        self.speech_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/speech", String, self.speech_cb)
        self.arms_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/arms", MoveArms, self.arms_cb)
        self.head_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/head", MoveHead, self.head_cb)
        self.face_img_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/face_img", String, self.face_img_cb)
        self.led_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/led", Int8MultiArray, self.led_cb )

        rospy.init_node("misty_" + str(idx), anonymous=False)
        rospy.spin()

        self.cleanup()

    def led_cb(self, msg):
        r, g, b = msg.data
        self.robot.changeLED(r, g, b)

    def face_img_cb(self, msg):
        self.robot.changeImage(msg.data)

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