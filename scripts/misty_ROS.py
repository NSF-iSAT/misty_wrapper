#!/usr/bin/env python3
import rospy
from misty_wrapper.mistyPy import Robot

from std_msgs.msg import String, Int8MultiArray
from misty_wrapper.msg import MoveArms, MoveHead

# movement limits, in degrees
PITCH_UP = -35
PITCH_DOWN = 20
YAW_LEFT = 70
YAW_RIGHT = -70
ROLL_LEFT = -35
ROLL_RIGHT = 35

class MistyNode:
    """
    Recieve and execute motor and display commands to Misty.
    """

    def __init__(self, idx=0, ip=None):
        self.ip = ip
        if rospy.get_param("/misty_ROS_"+ str(idx) + "/use_robot") == True:
            if ip is None:
                while not self.ip:
                    self.ip = rospy.get_param("/misty/id_" + str(idx) + "/robot_ip")
                    rospy.sleep(1.0)
            self.robot = Robot(self.ip)

        # SUBSCRIPTIONS
        self.speech_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/speech", String, self.speech_cb)
        self.arms_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/arms", MoveArms, self.arms_cb)
        self.head_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/head", MoveHead, self.head_cb)
        self.face_img_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/face_img", String, self.face_img_cb)
        self.led_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/led", Int8MultiArray, self.led_cb )
        self.action_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/action", String, self.action_cb)

        # reset degreess
        self.robot.MoveHead(0, 0, 0, 90, "degrees")
        
        self.head_pos = {
            "roll"  : 0,
            "pitch" : 0,
            "yaw"   : 0
        }

        rospy.init_node("misty_" + str(idx), anonymous=False)
        rospy.spin()

    def led_cb(self, msg):
        r, g, b = msg.data
        self.robot.ChangeLED(r, g, b)

    def face_img_cb(self, msg):
        self.robot.DisplayImage(msg.data)

    def speech_cb(self, msg):
        self.robot.Speak(msg.data)

    def arms_cb(self, msg):
        left_arm_msg = msg.leftArm
        right_arm_msg = msg.rightArm

        self.robot.MoveArms(left_arm_msg.value, right_arm_msg.value,  left_arm_msg.velocity, right_arm_msg.velocity, 
            units="degrees")

    def head_cb(self, msg):
        roll  = max(min(msg.roll, ROLL_RIGHT),  ROLL_LEFT)
        pitch = max(min(msg.pitch, PITCH_DOWN), PITCH_UP)
        yaw   = max(min(msg.yaw, YAW_LEFT),     YAW_RIGHT)

        self.robot.MoveHead( roll, pitch, yaw, velocity=msg.velocity, units=msg.units, duration=msg.duration)
        # update internal representation of head degrees
        self.head_pos["roll"] = msg.roll
        self.head_pos["pitch"] = msg.pitch
        self.head_pos["yaw"] = msg.yaw

    def action_cb(self, msg):
        if msg.data == "nod":
            self.nod()
        elif msg.data == "unsure":
            self.unsure()
        elif msg.data == "celebrate":
            self.celebrate()
        elif msg.data == "laugh":
            self.laugh()
        elif msg.data == "surprise":
            self.surprise()

    def reset_head_pos(self):
        # resets Misty's head to match stored values -
        # useful for resetting after a temporary head gesture
        self.robot.MoveHead(self.head_pos["roll"], self.head_pos["pitch"], 
            self.head_pos["yaw"], 95, "degrees")

    # # # action macros # # #

    def nod(self):
        new_pitch = self.pitch + 15
        if new_pitch > PITCH_DOWN:
            new_pitch -= 30

        self.robot.MoveHead(0, new_pitch, self.head_pos["yaw"], 100, "degrees")
        rospy.sleep(0.5)
        self.robot.MoveHead(self.head_pos["roll"], self.head_pos["pitch"],
                                self.head_pos["yaw"], 100, "degrees")

    def tilt_head(self, direction):
        if direction == "left":
            self.head_pos["roll"] = ROLL_LEFT
        elif direction == "right":
            self.head_pos["roll"] = ROLL_RIGHT
        else:
            self.head_pos["roll"] = 0

        self.robot.MoveHead(self.head_pos["roll"], self.head_pos["pitch"], self.head_pos["yaw"], 95, "degrees")

    def unsure(self):
        self.tilt_head("left")
        self.robot.DisplayImage("e_ApprehensionConcern.jpg")
        self.robot.MoveArms(10, 10, 80, 80, "position")
        rospy.sleep(1.5)

        self.tilt_head("straight")
        self.robot.DisplayImage("e_DefaultContent.jpg")
        self.robot.MoveArms(5, 5, 80, 80, "position")
        
    def celebrate(self):
        self.tilt_head("left")
        self.robot.DisplayImage("e_Admiration.jpg")
        self.robot.MoveArm("left", 10, 80, "position")
        self.robot.MoveArm("right", 0, 80, "position")
        rospy.sleep(0.5)

        self.tilt_head("right")
        self.robot.MoveArm("left", 0, 80, "position")
        self.robot.MoveArm("right", 10, 80, "position")
        rospy.sleep(0.5)

        self.tilt_head("straight")
        self.robot.DisplayImage("e_EcstacyStarryEyed.jpg")
        self.robot.MoveArms(5, 5, 80, 80, "position")
        rospy.sleep(1.0)

        self.robot.DisplayImage("e_DefaultContent.jpg")

    def laugh(self):
        self.robot.DisplayImage("e_Admiration.jpg")
        self.robot.MoveHead(0, -25, 0, 100, "degrees")
        rospy.sleep(2.0)
        self.reset_head_pos()
        self.robot.DisplayImage("e_DefaultContent.jpg")

    def surprise(self):
        self.robot.DisplayImage("e_Surprise.jpg")
        self.tilt_head("right")
        self.robot.MoveArms(10, 10, 100, 100, "position")
        rospy.sleep(1.0)

        self.tilt_head("straight")
        self.robot.MoveArms(0, 0, 80, 80, "position")
        self.robot.DisplayImage("e_DefaultContent.jpg")

if __name__ == "__main__":
    MistyNode()