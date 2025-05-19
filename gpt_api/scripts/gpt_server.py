#!/usr/bin/env python3
# reference: https://github.com/openai/openai-python
import os
import rospy
from gpt_api.srv import GPTAsk, GPTAskRequest, GPTAskResponse
from openai import OpenAI

# Initialize OpenAI API
client = OpenAI(
    api_key = os.environ.get("OPENAI_API_KEY"),
)

def gpt_ask(req):
    question = req.question
    rospy.loginfo(f"[GPT] Received question: {question}")

    ## Prompt design ##
    prompt = f"Solve the following math problem and return \
               only the integer answer without explanation: {question}"

    try:
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "user", "content": prompt}
            ],
            temperature=0.5
        )
        answer = response.choices[0].message.content.strip()
        rospy.loginfo(f"[GPT] Answer: {answer}")
        return GPTAskResponse(answer)
    except Exception as e:
        rospy.logerr(f"[GPT] OpenAI API error: {e}")
        return GPTAskResponse("Error communicating with OpenAI API.")
    
def gpt_server():
    rospy.init_node('chatgpt_service_node')
    service = rospy.Service('ask_gpt', GPTAsk, gpt_ask)
    rospy.loginfo("GPT service is ready")
    rospy.spin()

if __name__ == '__main__':
    gpt_server()

