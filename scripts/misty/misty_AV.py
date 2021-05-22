#!/usr/bin/env python
import websocket
import rospy

from cv_bridge import CvBridge
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo

import mistyPy
from simple_av_client import VidStreamer, AudioStreamer

def make_camera_info_message(stamp, frame_id, image_width, image_height, cx, cy, fx, fy):
    """
    Build CameraInfo message
    :param stamp: timestamp for the message
    :param frame_id: frame id of the camera
    :param image_width: image width
    :param image_height: image height
    :param cx: x-coordinate of principal point in terms of pixel dimensions
    :param cy: y-coordinate of principal point in terms of pixel dimensions
    :param fx: focal length in terms of pixel dimensions in the x direction
    :param fy: focal length in terms of pixel dimensions in the y direction
    :return: CameraInfo message with the camera calibration parameters.
    """
    camera_info_msg = CameraInfo()
    camera_info_msg.header.stamp = stamp
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.width = image_width
    camera_info_msg.height = image_height
    camera_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info_msg.D = [0, 0, 0, 0, 0]
    camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    camera_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
    camera_info_msg.distortion_model = "plumb_bob"
    return camera_info_msg

class MistyAVNode:
    def __init__(self, idx=0):
        self.ip = rospy.get_param("/misty_ROS/robot_ip")

        self.cam_pub = rospy.Publisher("/misty/id_" + str(idx) + "/camera", Image, queue_size=10)
        self.caminfo_pub = rospy.Publisher("misty/id_" + str(idx) + "/camera_info", CameraInfo, queue_size=10)
    
        self.robot = mistyPy.Robot(self.ip)

        self.bridge = CvBridge()
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
        # TODO FIX
        url = "rtsp://" + self.ip + ":" + port_no
        self.robot.startAvStream("rtspd:" + port_no, dimensions=(640, 480))
        rospy.sleep(2)
        self.vid_stream     = VidStreamer(url).start()
        # self.audio_stream   = AudioStreamer(url)

    def video_cb(self):
        frame = self.vid_stream.frame
        if frame is None or not frame.any():
            rospy.sleep(0.5)
            return
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.cam_pub.publish(img_msg)

if __name__ == "__main__":
    MistyAVNode()
