#!/usr/bin/env python3

import rospy
from gpt_api.srv import GPTAsk
from std_msgs import String


def ask_gpt(question):
    rospy.wait_for_service('ask_gpt')
    try:
        ask = rospy.ServiceProxy('ask_gpt', GPTAsk)
        response = ask(question.data)
        print(f"Answer: {response.answer}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('ai_client')
    gpt_sub = rospy.Subscriber("qr_codes", String, ask_gpt, queue_size=1)
    
