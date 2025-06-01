#!/usr/bin/env python3

import rospy
from gpt_api.srv import GPTAsk
from std_msgs.msg import String, Bool

class GoalSelector:
    def __init__(self):
        rospy.init_node('goal_selector')
        # Publisher, Subscriber
        self.pub = rospy.Publisher("/goal", String, queue_size=1)
        self.qr_sub = rospy.Subscriber("/qr_code", String, self.gptCallback, queue_size=1)
        self.gate_sub = rospy.Subscriber("/gate_passed", Bool, self.gateCallback, queue_size=1)

        # GPT service
        rospy.wait_for_service('ask_gpt')
        self.ask_gpt = rospy.ServiceProxy('ask_gpt', GPTAsk)

        # Current state
        self.current_gate = 0
        self.direction = String()
        self.questions = dict()

    def publish_direction(self, dir):
        # Publish the goal with the given direction
        self.direction.data = dir
        self.pub.publish(self.direction)

    def blink_LED(self, num):
        # Blink LED with the given number times
        self.publish_direction("LEFT")
        return   

    def gptCallback(self, msg):
        ## Prompt design ##
        prompt = f"Possible answers: \"True\", \"False\", \"A\", \"B\", or a single number (e.g., 1, 2, 3).\
                For the following question, return the key (e.g. LEFT or RIGHT, or possible answers) that \
                corresponds to the correct value of the answer. The question will not specify this explicitly. \
                Reply with only one key or the answer if there's no key — no explanation, no punctuation, no extra text. \
                Question: {msg.data}"

        ## GPT answer ##
        if (self.current_gate not in self.questions):
            # Store Gate pair and Answer to prevent redundant API call
            answer = self.ask_gpt(prompt).answer.strip().upper()
            self.questions[self.current_gate] = answer
        else:
            return

        ## Goal Select ##
        if (answer.isdigit()):
            # If the answer is a number, do the LED action
            rospy.loginfo("Blink LED and turn LEFT")
            self.blink_LED(int(answer))
        elif (answer in {"LEFT", "True", "A"}):
            rospy.loginfo("Turn LEFT")
            self.publish_direction("LEFT")
        elif (answer in {"RIGHT", "False", "B"}):
            rospy.loginfo("Turn RIGHT")
            self.publish_direction("RIGHT")
        else:
            # Invlaid answer
            rospy.logwarn(f"Invalid GPT answer: {answer}")
        return
    
    def gateCallback(self, msg):
        # Check if the current gate has passed
        if (msg.data):
            self.current_gate += 1
        return
                
# Run goal selector node
if __name__ == "__main__":
    gs = GoalSelector()
    rospy.spin()
    
