#!/usr/bin/env python3
import rospy
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from misty_wrapper.py3_cv_bridge import cv2_to_imgmsg
from misty_wrapper import mistyPy
from misty_wrapper.simple_av_client import VidStreamer
from misty_wrapper.msg import DetectedFace


class MistyAVNode:
    def __init__(self, idx=0):
        self.ip = ""
        while not self.ip:
            self.ip = rospy.get_param("/misty/id_" + str(idx) + "/robot_ip")
            rospy.sleep(1.0)
        self.stream_res = (rospy.get_param("/misty_AV_" + str(idx) + "/stream_resolution/W"), rospy.get_param("/misty_AV_" + str(idx) + "/stream_resolution/H"))

        self.learn_face_sub = rospy.Subscriber("/misty/id_" + str(idx) + "/learn_face", String, self.learn_face_cb)

        self.cam_pub = rospy.Publisher("/misty/id_" + str(idx) + "/camera", Image, queue_size=10)
        self.caminfo_pub = rospy.Publisher("misty/id_" + str(idx) + "/camera_info", CameraInfo, queue_size=10)
        self.face_pub = rospy.Publisher("/misty/id_" + str(idx) + "/face_recognition", DetectedFace, queue_size=2)
    
        self.robot = mistyPy.Robot(self.ip)
        self.M_rotation = None
        # self.bridge = CvBridge()
        rospy.init_node("misty_AV_" + str(idx), anonymous=False)

        self._av_setup()
        self._face_setup()

        while not rospy.is_shutdown():
            self.video_cb()
            # TODO debug: self.face_cb()
            self.face_cb()
            rospy.sleep(0.01)

        self._cleanup()

    def _cleanup(self):
        self.vid_stream.stop()
        print("stopped streaming video")
        self.robot.stopAvStream()
        print("stopped misty av stream")
        self.robot.unsubscribe("FaceRecognition")
        print("unsubscribed from facerec")

    def _av_setup(self, port_no="1935"):
        url = "rtsp://" + self.ip + ":" + port_no
        self.robot.startAvStream("rtspd:" + port_no, dimensions=self.stream_res)
        rospy.sleep(2)
        self.vid_stream     = VidStreamer(url).start()

    def _face_setup(self):
        # self.robot.startFaceRecognition()
        self.robot.subscribe("FaceRecognition")

    def video_cb(self):
        frame = self.vid_stream.frame
        if frame is None or not frame.any():
            rospy.sleep(0.5)
            return
        frame = imutils.rotate_bound(frame, 90)
        img_msg = cv2_to_imgmsg(frame)
        self.cam_pub.publish(img_msg)

    def face_cb(self):
        # print("facerec callback")
        data = self.robot.faceRec()
        # TODO publish data
        try:
            msg = DetectedFace({"name": data["personName"], "distance" : data["distance"],
            "elevation" : data["elevation"]})
        except KeyError:
            return
        self.face_pub.publish(msg)
        # print(data)

    def learn_face_cb(self, name):
        self.robot.learnFace(name)

if __name__ == "__main__":
    MistyAVNode()
