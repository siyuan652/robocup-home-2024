#!/home/orin_nano/catkin_ws/src/gemini_ai/scripts/.venv/bin/python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os

def callback(data):
    rospy.loginfo("Input: %s", data.data)

    text = data.data
    tts = gTTS(text, lang='en', tld='com.au')
    
    tts.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")
    
def googletts():
    rospy.init_node('googletts', anonymous=True)

    rospy.Subscriber("samantha_topic", String, callback)

    rospy.spin()

if __name__ == '__main__':
    googletts()