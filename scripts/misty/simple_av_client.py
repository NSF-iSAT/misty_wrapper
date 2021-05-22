import cv2
import numpy as np
from threading import Thread

"""
Based heavily on Github user @CPsridharCP's implementation of Misty teleoperation: https://github.com/CPsridharCP/MistySkills/tree/master/Apps/Teleop/02_pythonTeleop

"""

class VidStreamer:
    def __init__(self, path):
        self.stream = cv2.VideoCapture(path)
        self.stopped = False
        self.frame = np.array([])

        # Another option would be to handle the video through python av as well;
        # for now i'm sticking with cv2 to allow for any OpenCV applications we might
        # want to use down the line.

    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # while not self.stopped:
        #     grabbed, frame = self.stream.read()
        #     # if not grabbed:
        #     #     return

        #     self.frame = frame

        # self.stream.release()
        while not self.stopped:
            ret_val, self.frame = self.stream.read()
        self.stream.release()
    def stop(self):
        self.stopped = True


def test_streaming():
    # v = VidStreamer("rtsp://192.168.50.154:1935").start()
    # cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
    # while True:
    #     img = v.frame
    #     cv2.imshow('demo', img)
    #     cv2.waitKey(10)

    # # cv2.destroyAllWindows()
    pass

if __name__ == "__main__":
    test_streaming()