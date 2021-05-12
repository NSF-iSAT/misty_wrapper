#!/usr/bin/env python
import mistyPy
import websocket
import rospy

from cv_bridge import CvBridge
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo

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

class MistyNode:
    def __init__(self, idx=0, ip=None):
        self.ip = ip
        if rospy.get_param("/mistyROS/use_robot") == True:
            if ip is None:
                self.ip = rospy.get_param("/mistyROS/robot_ip")
            assert self.ip, "Node failed to launch in use_robot mode; provide a valid Misty IP"
            print('IP: ', self.ip)
            self.robot = mistyPy.Robot(self.ip)
            self._setup_av_streaming()

        self.bridge = CvBridge()

        self.cam_pub = rospy.Publisher("/misty/id_" + str(idx) + "/camera", Image, queue_size=10)
        self.caminfo_pub = rospy.Publisher("misty/id_" + str(idx) + "/camera_info", CameraInfo, queue_size=10)
        # self.caminfo_msg = make_camera_info_message()

        rospy.init_node("misty_" + str(idx), anonymous=False)

        while not rospy.is_shutdown():
            # loop: do something with current cam stream, publish frames, etc.
            latest_frame = self.vid_cb()
            if latest_frame:
                img_msg = self.bridge.cv2_to_imgmsg(latest_frame, encoding="passthrough")
                self.cam_pub.publish(img_msg)
            # self.caminfo_pub
            # pass

        self.cleanup()

    def _setup_av_streaming(self, port_no="1935"):
        url = "rtsp://" + self.ip + ":" + port_no
        self.robot.startAvStream("rtspd:" + port_no, dimensions=(640, 480))

        self.vid_stream     = VidStreamer(url)
        self.audio_stream   = AudioStreamer(url)

    def vid_cb(self):
        latest_frame = self.vid_stream.frame
        return latest_frame

    def cleanup(self):
        self.vid_stream.stop()
        self.robot.stopAvStream()

MistyNode()