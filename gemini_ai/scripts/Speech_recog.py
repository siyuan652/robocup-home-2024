#!/home/orin_nano/catkin_ws/src/gemini_ai/scripts/.venv/bin/python3
import rospy
from std_msgs.msg import String, Bool
import speech_recognition as sr

is_tts_speaking = False

def tts_status_callback(data):
    global is_tts_speaking
    is_tts_speaking = data.data

def googlesr():
    rospy.init_node('googlesr', anonymous=True)
    pub = rospy.Publisher('result', String, queue_size=10)
    rospy.Subscriber('tts_status', Bool, tts_status_callback)

    while not rospy.is_shutdown():
        # Wait until TTS finishes speaking
        while is_tts_speaking:
            rospy.sleep(0.1)

        # obtain audio from the microphone
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print(">>> Say something!")
            audio = r.record(source, duration=5)
            
        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio, language="en-US")
            print(result)
            pub.publish(result)
            rospy.sleep(10)  # Wait for 10 seconds after detecting speech
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == '__main__':
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass
