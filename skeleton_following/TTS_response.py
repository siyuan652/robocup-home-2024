import rospy
from std_msgs.msg import String
import pyttsx3

class TextToSpeechNode:
    def __init__(self):
        rospy.init_node('fall_tts_node', anynomous=True)
        self.engine = pyttsx3.init()
        rospy.Subscriber('/fall', String, self.callback)
        
    def callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Recieved message: {text}")
        
        self.engine.say(text)
        self.engine.runAndWait()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tts_node = TextToSpeechNode()
        tts_node.run()
    except rospy.ROSInterruptException:
        pass