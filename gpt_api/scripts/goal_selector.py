#!/usr/bin/env python3

import rospy
from gpt_api.srv import GPTAsk
from std_msgs.msg import String, Bool

# import Jetson.GPIO as GPIO
# import time

# GPIO setting
# led_pin = 15
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(led_pin, GPIO.OUT)

# Goal selector class
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

    # def blink_once(self):
    #     GPIO.output(led_pin, GPIO.HIGH)
    #     time.sleep(0.5)
    #     GPIO.output(led_pin, GPIO.LOW)
    #     time.sleep(0.5)

    def blink_LED(self, num):
        # Blink LED with the given number times
        # for _ in range(num):
        #     self.blink_once()
        # Turn left
        self.publish_direction("LEFT")
        return

    def gptCallback(self, msg):
        ## Prompt design ##
        prompt = f"For the following question, return the LEFT or RIGHT that corresponds to the answer.\
            Reply only the answer — no explanation, no extra text. Assume Tag ID X = X. Use python. {msg.data} {msg.data}"

        ## GPT answer ##
        if (self.current_gate not in self.questions):
            # Check if the gate has already passed before
            key = self.current_gate%4
            if (key in self.questions):
                answer = self.questions[key]
            # Store Gate pair and Answer to prevent redundant API call
            else:
                answer = self.ask_gpt(prompt).answer.strip().upper()
                self.questions[self.current_gate] = answer
        else:
            return

        ## Goal Select ##
        if (answer.isdigit()):
            # If the answer is a number, do the LED action
            rospy.loginfo("Blink LED and turn LEFT")
            self.blink_LED(int(answer))
        elif (answer in {"LEFT", "TRUE", "A"}):
            rospy.loginfo("Turn LEFT")
            self.publish_direction("LEFT")
        elif (answer in {"RIGHT", "FALSE", "B"}):
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
    
