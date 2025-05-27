#!/usr/bin/env python3

import rospy
from gpt_api.srv import GPTAsk
from std_msgs import String
from geometry_msgs.msg import PoseStamped, Point

# Gate position
gates = {0:[Point(5, -4, 0.75), Point(5, -4, 0.75)],
         1:[Point(15, -4, 0.75), Point(15, -4, 0.75)],
         2:[Point(15, 4, 0.75), Point(15, 4, 0.75)],
         3:[Point(5, 4, 0.75), Point(5, 4, 0.75)]
        }

class GoalSelector:
    def __init__(self):
        rospy.init_node('goal_selector')
        # Publisher, Subscriber
        self.pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber("qr_codes", String, self.gpt_callback, queue_size=1)

        # GPT service
        rospy.wait_for_service('ask_gpt')
        self.ask_gpt = rospy.ServiceProxy('ask_gpt', GPTAsk)

        # Current state
        self.current_gate = 0
        self.questions = dict()

    def gpt_callback(self, msg):
        # GPT answer; LEFT or RIGHT
        if (self.current_gate not in self.questions):
            answer = self.ask_gpt(msg.data).strip().upper()
            self.questions[self.current_gate] = answer
        else:
            return

        # Do the action
        if (answer == "LEFT"):
            rospy.loginfo("Turn LEFT")
            self.publish_point(0)
        elif (answer == "RIGHT"):
            rospy.loginfo("Turn RIGHT")
            self.publish_point(1)
        else:
            # Invlaid answer
            rospy.logwarn(f"Invalid GPT answer: {answer}")
    
    def publish_goal(self, direction):
        # diretion; Left: 0, Right: 1
        gate = gates[self.current_gate][direction]
        # Publish waypoint
        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "odom"
        waypoint.pose.position = gate
        waypoint.pose.orientation.w = 1.0
        self.pub.publish(waypoint)

        # New gates
        self.current_gate += 1
                
# Run goal selector node
if __name__ == "__main__":
    gs = GoalSelector()
    rospy.spin()
    
