#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import io
from openai import OpenAI
import os

# Import the service type
from gpt_api.srv import GPTImageQuery, GPTImageQueryResponse

# Initialize OpenAI API
client = OpenAI(
    #organization = os.environ.get("ORGANIZATION_KEY")
    api_key = os.environ.get("OPENAI_API_KEY")
)

gpt_model = "gpt-4o"
max_tokens = 300

bridge = CvBridge()

# Convert OpenCV image (from ROS) to base64
def encode_image_to_base64(cv_image):
    success, buffer = cv2.imencode('.jpg', cv_image)
    if not success:
        raise RuntimeError("Failed to encode image")
    return base64.b64encode(buffer).decode("utf-8")

def ask_gpt_with_image(image_base64, user_question):
    response = client.chat.completions.create(
        model=gpt_model,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_base64}"
                        }
                    },
                    {
                        "type": "text",
                        "text": user_question
                    }
                ]
            }
        ],
        max_tokens=300
    )
    return response.choices[0].message.content

def handle_gpt_image_query(req):
    """
    Service callback: takes req.image (sensor_msgs/Image) 
    and req.question (string), returns GPT’s answer.
    """
    # Convert ROS Image to OpenCV
    try:
        cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"CV Bridge error: {e}")

    # Encode to base64
    try:
        
        image_base64 = encode_image_to_base64(cv_image)
    except Exception as e:
        rospy.logerr(f"Image encoding failed: {e}")

    # Call GPT
    try:
        answer_text = ask_gpt_with_image(image_base64, req.question)
    except Exception as e:
        rospy.logerr(f"OpenAI request failed: {e}")

    return GPTImageQueryResponse(answer=answer_text)

def gpt_image_query_server():
    rospy.init_node('ask_image_gpt')
    service = rospy.Service('ask_image_gpt', GPTImageQuery, handle_gpt_image_query)
    rospy.loginfo("GPT service is ready")
    rospy.spin()

if __name__ == "__main__":
    gpt_image_query_server()
