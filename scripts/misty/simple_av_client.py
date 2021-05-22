import cv2
import av
import numpy as np
from threading import Thread
from collections import deque
import time

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

class AudioStreamer:
    def __init__(self, path):
        self.stream_path = path
        self.stopped = False

    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
    
    def update(self):
        queue = deque()
        def play_thread():
            # for now the stream is just passed to the pyAudio player;
            # we will probably want to do something else with it later
            p = pyaudio.PyAudio()
            stream = p.open(format=pyaudio.paFloat32,
                            channels=1,
                            rate=11025,
                            output=True)
            while not self.stopped:
                if len(queue) == 0:
                    time.sleep(0.25)
                    continue
                frame = queue.popleft()
                stream.write(frame.to_ndarray().astype(np.float32).tostring())

        t = Thread(target=play_thread)
        t.start()
        
        container = av.open(self.stream_path)
        input_stream = container.streams.get(audio=0)[0]
        
        for frame in container.decode(input_stream):
            frame.pts = None
            queue.append(frame)

def test_streaming():
    v = VidStreamer("rtsp://192.168.50.154:1935").start()
    cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
    while True:
        img = v.frame
        cv2.imshow('demo', img)
        cv2.waitKey(10)

    # cv2.destroyAllWindows()


if __name__ == "__main__":
    test_streaming()