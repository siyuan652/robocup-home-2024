#!/home/orin_nano/catkin_ws/src/gemini_ai/scripts/.venv/bin/python3
import rospy
from std_msgs.msg import String, Bool
from gtts import gTTS
import os

def callback(data):
    rospy.loginfo("Input: %s", data.data)
    text = data.data
    tts = gTTS(text, lang='en', tld='com.au')
    
    tts.save("speech.mp3")
    
    # Indicate TTS is starting
    status_pub.publish(True)
    rospy.sleep(0.5)  # Give subscribers time to react

    # Play the TTS audio
    os.system("mpg321 speech.mp3")
    os.remove("speech.mp3")
    
    # Indicate TTS has finished
    status_pub.publish(False)

def googletts():
    global status_pub
    rospy.init_node('googletts', anonymous=True)

    rospy.Subscriber("Alice_topic", String, callback)
    
    status_pub = rospy.Publisher('tts_status', Bool, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        googletts()
    except rospy.ROSInterruptException:
        pass
