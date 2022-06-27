#!/usr/bin/env python3
import rospy
from misty_wrapper.mistyPy import Robot

from std_msgs.msg import String, Int8MultiArray
from misty_wrapper.msg import MoveArms, MoveHead

def tilt(robot, direction):
    print("TILT", direction)
    if direction == "left": # house left
        # parameter order: roll, pitch, yaw, velocity
        robot.moveHeadPosition(5, 0, 0, 100)
    elif direction == "right": # house right
        robot.moveHeadPosition(-5, 0, 0, 100)

def look(robot, direction):
    print("LOOK", direction)
    if direction == "up": # negative pitch
        # parameter order: roll, pitch, yaw, velocity
        robot.moveHeadPosition(0, -5, 0, 100)
    elif direction == "down":
        robot.moveHeadPosition(0, 5, 0, 100)
    elif direction == "left": # house left, negative yaw
        robot.moveHeadPosition(0, 0, -5, 100)
    elif direction == "right": # house right
        # doesn't go as far right as left
        # looks like 45 degree yaw, versus 90 degree yaw seen with "left"
        robot.moveHeadPosition(0, 0, 5, 100)
    elif direction == "straight":
        robot.moveHeadPosition(0, 0, 0, 100)

def led(robot, color):
    print("LED", color)
    if color == "red":
        robot.ChangeLED(255, 0, 0)
    elif color == "green":
        robot.ChangeLED(0, 255, 0)
    elif color == "blue":
        robot.ChangeLED(0, 0, 255)

def move_arms(robot, arm, move):
    # NOTE: feel free to change the input vars -- just adjust the functions_mapping entry accordingly
    if arm == "both": # move both arms
        # parameters for moveArms(self, rightArmPosition, leftArmPosition, rightArmVelocity, leftArmVelocity, units)
        print("move", arm, "arm", move)
        if move == "up":
            robot.MoveArms(10, 10, 80, 80, units = "position")
        elif move == "down":
            robot.MoveArms(0, 0, 80, 80, units = "position")
        elif move == "straight":
            robot.MoveArms(5, 5, 80, 80, units = "position")
    else: # move one arm
        # parameters for moveArm(self, arm, position, velocity, units)
        print("move", arm, "arm", move)
        if move == "up":
            robot.MoveArm(arm, 10, 80, "position")
        elif move == "down":
            robot.MoveArm(arm, 0, 80, "position")
        elif move == "straight":
            robot.MoveArm(arm, 5, 80, "position")

def set_expression(robot, expression):
    # NOTE: one nice feature for this function would be to add a "duration" parameter
    #      so that the expression can be set for a certain amount of time (or forever)
    #      and then revert to a "default" expression
    # used robot.printImageList() to get list of saved images
    # use a dictionary instead of if/elif statement
    # dictionary maps key word to full jpg file name of expression
    
    expr_dict = {
                "DEFAULT" : "e_DefaultContent.jpg",
                 "admiration": 'e_Admiration.jpg',
                 "amazement": 'e_Amazement.jpg',
                 "anger": 'e_Anger.jpg',
                 "apprehension": 'e_ApprehensionConcerned.jpg',
                 "contempt": 'e_Contempt.jpg',
                 "contentLeft": 'e_ContentLeft.jpg', #stage left
                 "contentRight": 'e_ContentRight.jpg', #stage right
                 "fear": 'e_Fear.jpg',
                 "grief": 'e_Grief.jpg',
                 "hilarious": 'e_EcstacyHilarious.jpg',
                 "joy": 'e_Joy.jpg',
                 "joyGoofy": 'e_JoyGoofy.jpg',
                 "love": 'e_Love.jpg',
                 "remorse": 'e_RemorseShame.jpg',
                 "sadness": 'e_Sadness.jpg',
                 "sleeping": 'e_Sleeping.jpg',
                 "sleepy": 'e_Sleepy.jpg',
                 "starryEyed": 'e_EcstacyStarryEyed.jpg',
                 "surprise": 'e_Surprise.jpg',
                 "blackScreen": 'e_SystemBlackScreen.jpg',
                 "blinkingLarge": 'e_SystemBlinkLarge.jpg',
                 "blinkingStandard": 'e_SystemBlinkStandard.jpg',
                }

    print("Show", expression, "expression")
    robot.DisplayImage(expr_dict.get(expression))

def nod(robot):
    print("Nod")
    robot.MoveHead(0, -10, 0, 100)
    rospy.sleep(0.5)
    robot.MoveHead(0, 10, 0, 100)
    rospy.sleep(0.5)
    robot.MoveHead(0, 0, 0, 100)

def unsure(robot):
    print("Unsure")
    tilt(robot, "left")
    set_expression(robot, "apprehension")
    move_arms(robot, "both", "up")
    rospy.sleep(1.5)
    look(robot, "straight")
    move_arms(robot, "both", "straight")
    set_expression(robot, "DEFAULT")

def celebrate(robot):
    print("Celebrate")
    tilt(robot, "left")
    set_expression(robot, "admiration")
    move_arms(robot, "right", "up")
    move_arms(robot, "left", "down")
    rospy.sleep(0.5)
    tilt(robot, "right")
    move_arms(robot, "right", "down")
    move_arms(robot, "left", "up")
    rospy.sleep(0.5)
    look(robot, "straight")
    set_expression(robot, "starryEyed")
    move_arms(robot, "both", "straight")
    rospy.sleep(1.0)
    set_expression(robot, "DEFAULT")

def laugh(robot):
    print("Laughing")
    set_expression(robot, "admiration")
    robot.MoveHead(0, -25, 0, 100)
    rospy.sleep(2.0)
    look(robot, "straight")
    set_expression(robot, "DEFAULT")

def surprise(robot):
    print("Surprise")
    set_expression(robot, "surprise")
    tilt(robot, "right")
    move_arms(robot, "both", "up")
    rospy.sleep(1.0)
    look(robot, "straight")
    move_arms(robot, "both", "straight")
    set_expression(robot, "DEFAULT")
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
            units="position")

    def head_cb(self, msg):
        self.robot.MoveHead( roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, velocity=msg.velocity, units=msg.units, duration=msg.duration)

    def action_cb(self, msg):
        if msg.data == "nod":
            nod(self.robot)
        elif msg.data == "unsure":
            unsure(self.robot)
        elif msg.data == "celebrate":
            celebrate(self.robot)
        elif msg.data == "laugh":
            laugh(self.robot)
        elif msg.data == "surprise":
            surprise(self.robot)

if __name__ == "__main__":
    MistyNode()