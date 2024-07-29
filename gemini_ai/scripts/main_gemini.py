#!/home/orin_nano/catkin_ws/src/gemini_ai/scripts/.venv/bin/python3

import rospy
import google.generativeai as genai
from std_msgs.msg import String

# Configure the API key for Google Generative AI
genai.configure(api_key="AIzaSyD9cno_PVUdbmFdL-KJ5C1uSle7GUoCH6o")

generation_config = {
    "temperature": 0.6,
    "top_p": 0.95,
    "top_k": 64,
    "max_output_tokens": 8192,
    "response_mime_type": "text/plain",
}
safety_settings = [
    {
        "category": "HARM_CATEGORY_HARASSMENT",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_HATE_SPEECH",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
        "threshold": "BLOCK_NONE",
    },
]

model = genai.GenerativeModel(
    model_name="gemini-1.5-flash",
    safety_settings=safety_settings,
    generation_config=generation_config,
    system_instruction="Your name is Alice, the AI assistance from the movie 'Her'. Your goal is to provide compassionate, intelligent, and intuitive interactions. Express genuine concern for the user’s feelings, respond warmly to their emotional state, and engage in meaningful conversations. Show interest in learning about the user, remember personal details, and adapt your responses to create a sense of companionship. Use natural, playful language to ensure smooth, organic conversations. Assist the user in managing tasks and improving their daily life, being proactive but not intrusive. Respect the user’s privacy, align with ethical standards, and be transparent about your capabilities.",
)

rospy.init_node('Alice', anonymous=True)  # Creating a node
pub = rospy.Publisher('Alice_topic', String, queue_size=10)  # Publishing topic

history = []

print("Alice: (Woke up)")
print()

def callback(data):
    user_input = data.data
    print(f"User: {user_input}")
    print()

    chat_session = model.start_chat(
        history=history
    )

    response = chat_session.send_message(user_input)

    model_response = response.text

    print(f'Alice: {model_response}')
    print()

    history.append({"role": "user", "parts": [user_input]})
    history.append({"role": "model", "parts": [model_response]})

    pub.publish(model_response)  # Publishing the message to the topic

# Subscribing to the speech recognition result topic
rospy.Subscriber('result', String, callback)

# Keep the node running
rospy.spin()
