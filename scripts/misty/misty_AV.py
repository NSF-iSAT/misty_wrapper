#!/usr/bin/env python3
import websocket
import rospy

# from cv_bridge import CvBridge
from py3_cv_bridge import cv2_to_imgmsg
import imutils
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo

import mistyPy
from simple_av_client import VidStreamer

class MistyAVNode:
    def __init__(self, idx=0):
        self.ip = rospy.get_param("/misty_ROS/robot_ip")

        self.cam_pub = rospy.Publisher("/misty/id_" + str(idx) + "/camera", Image, queue_size=10)
        self.caminfo_pub = rospy.Publisher("misty/id_" + str(idx) + "/camera_info", CameraInfo, queue_size=10)
    
        self.robot = mistyPy.Robot(self.ip)
        self.M_rotation = None
        # self.bridge = CvBridge()
        rospy.init_node("misty_AV_" + str(idx), anonymous=False)

        self._setup()

        while not rospy.is_shutdown():
            self.video_cb()
            # rospy.sleep(0.1)

        self._cleanup()

    def _cleanup(self):
        self.vid_stream.stop()
        self.robot.stopAvStream()

    def _setup(self, port_no="1935"):
        url = "rtsp://" + self.ip + ":" + port_no
        self.robot.startAvStream("rtspd:" + port_no, dimensions=(640, 480))
        rospy.sleep(2)
        self.vid_stream     = VidStreamer(url).start()

    def video_cb(self):
        frame = self.vid_stream.frame
        if frame is None or not frame.any():
            rospy.sleep(0.5)
            return
        frame = imutils.rotate_bound(frame, 90)
        img_msg = cv2_to_imgmsg(frame)
        self.cam_pub.publish(img_msg)

if __name__ == "__main__":
    MistyAVNode()
