import os
import rospy

from std_msgs.msg import String

import google.cloud.texttospeech_v1beta1 as tts
from misty_wrapper.mistyPy import Robot

class MistyGoogleTTS:
    def __init__(self, idx=0):
        self.ip = ""
        while not self.ip:
            self.ip = rospy.get_param("/misty/id_" + str(idx) + "/robot_ip")
            rospy.sleep(1.0)
        # self.ip = "192.168.50.50"

        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/kaleb/code/ros_ws/src/ros_speech2text/ros-speech2text-google-stt-cred.json'

        self.language = 'en-US'
        self.voice = 'en-US-Standard-C'

        self.voice_params = tts.VoiceSelectionParams(
            language_code=self.language,
            name=self.voice,
            ssml_gender = tts.SsmlVoiceGender.FEMALE
        )

        self.audio_config = tts.AudioConfig(audio_encoding=tts.AudioEncoding.MP3)
        self.client = tts.TextToSpeechClient()
        self.robot = Robot(self.ip)

        rospy.init_node('misty_tts', anonymous=True)
        rospy.Subscriber('/text_to_speech', String, self.tts_callback)

    def tts_callback(self, msg):

        text = "<speak><prosody pitch=\"+2st\">{}</prosody></speak>".format(msg.data)
        text_input = tts.SynthesisInput(ssml=text)
        response = self.client.synthesize_speech(input=text_input, voice=self.voice_params, audio_config=self.audio_config)
        with open("tts.mp3", "wb") as out:
            out.write(response.audio_content)
        
        # send to Misty
        self.robot.uploadAudio("tts.mp3", apply=True, overwrite=True)

if __name__ == "__main__":
    node = MistyGoogleTTS()
    rospy.spin()