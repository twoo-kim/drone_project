#!/usr/bin/env python3

import rospy
from gpt_api.srv import GPTAsk

def ask_gpt(question):
    rospy.wait_for_service('ask_gpt')
    try:
        ask = rospy.ServiceProxy('ask_gpt', GPTAsk)
        response = ask(question)
        print(f"Answer: {response.answer}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('ai_client')
    ask_gpt("What is the capital of South Korea?")
