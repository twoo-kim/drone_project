#!/usr/bin/env python3
# reference: https://github.com/openai/openai-python
import os
import rospy
from gpt_api.srv import GPTAsk, GPTAskRequest, GPTAskResponse
from openai import OpenAI

# Initialize OpenAI API
client = OpenAI(
    #organization = os.environ.get("ORGANIZATION_KEY")
    api_key = os.environ.get("OPENAI_API_KEY")
)

gpt_model = "gpt-4o"
max_tokens = 300

def gpt_ask(req):
    question = req.question
    rospy.loginfo(f"[GPT] Received question: {question}")

    try:
        response = client.chat.completions.create(
            model=gpt_model,
            messages=[
                {"role": "system","content": "Forget the previous conversation."},
                {"role": "user", "content": question}
            ],
            max_tokens = max_tokens,
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



""" When we use web search
try:
    response = client.responses.create(
        model=gpt_model,
        tools=[{ "type": "web_search_preview" }],
        input = prompt,
    )
    answer = response.output_text.strip()

"""