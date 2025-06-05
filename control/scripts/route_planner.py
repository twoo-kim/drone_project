#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {0: {'LEFT': np.array([[ 3.65649712,  3.77434825,  3.89219938,  4.01005051,  4.12790164,
           4.24575277,  4.3636039 ,  4.3636039 ,  4.48734758,  4.61109127,
           4.73483496,  4.85857864,  4.98232233,  5.10606602,  5.2298097 ,
           5.35355339,  5.47729708,  5.60104076,  5.72478445,  5.84852814],
         [-2.65649712, -2.53864599, -2.42079486, -2.30294373, -2.18509259,
          -2.06724146, -1.94939033, -1.94939033, -2.07313402, -2.19687771,
          -2.32062139, -2.44436508, -2.56810877, -2.69185245, -2.81559614,
          -2.93933983, -3.06308351, -3.1868272 , -3.31057089, -3.43431458]]),
  'RIGHT': np.array([[ 3.65649712,  3.53864599,  3.42079486,  3.30294373,  3.18509259,
           3.06724146,  2.94939033,  2.94939033,  3.07313402,  3.19687771,
           3.32062139,  3.44436508,  3.56810877,  3.69185245,  3.81559614,
           3.93933983,  4.06308351,  4.1868272 ,  4.31057089,  4.43431458],
         [-2.65649712, -2.77434825, -2.89219938, -3.01005051, -3.12790164,
          -3.24575277, -3.3636039 , -3.3636039 , -3.48734758, -3.61109127,
          -3.73483496, -3.85857864, -3.98232233, -4.10606602, -4.2298097 ,
          -4.35355339, -4.47729708, -4.60104076, -4.72478445, -4.84852814]])},
 1: {'LEFT': np.array([[13.65649712, 13.53864599, 13.42079486, 13.30294373, 13.18509259,
          13.06724146, 12.94939033, 12.94939033, 13.07313402, 13.19687771,
          13.32062139, 13.44436508, 13.56810877, 13.69185245, 13.81559614,
          13.93933983, 14.06308351, 14.1868272 , 14.31057089, 14.43431458],
         [-5.34350288, -5.22565175, -5.10780062, -4.98994949, -4.87209836,
          -4.75424723, -4.6363961 , -4.6363961 , -4.51265242, -4.38890873,
          -4.26516504, -4.14142136, -4.01767767, -3.89393398, -3.7701903 ,
          -3.64644661, -3.52270292, -3.39895924, -3.27521555, -3.15147186]]),
  'RIGHT': np.array([[13.65649712, 13.77434825, 13.89219938, 14.01005051, 14.12790164,
          14.24575277, 14.3636039 , 14.3636039 , 14.48734758, 14.61109127,
          14.73483496, 14.85857864, 14.98232233, 15.10606602, 15.2298097 ,
          15.35355339, 15.47729708, 15.60104076, 15.72478445, 15.84852814],
         [-5.34350288, -5.46135401, -5.57920514, -5.69705627, -5.81490741,
          -5.93275854, -6.05060967, -6.05060967, -5.92686598, -5.80312229,
          -5.67937861, -5.55563492, -5.43189123, -5.30814755, -5.18440386,
          -5.06066017, -4.93691649, -4.8131728 , -4.68942911, -4.56568542]])},
 2: {'LEFT': np.array([[16.34350288, 16.22565175, 16.10780062, 15.98994949, 15.87209836,
          15.75424723, 15.6363961 , 15.6363961 , 15.51265242, 15.38890873,
          15.26516504, 15.14142136, 15.01767767, 14.89393398, 14.7701903 ,
          14.64644661, 14.52270292, 14.39895924, 14.27521555, 14.15147186],
         [ 2.65649712,  2.53864599,  2.42079486,  2.30294373,  2.18509259,
           2.06724146,  1.94939033,  1.94939033,  2.07313402,  2.19687771,
           2.32062139,  2.44436508,  2.56810877,  2.69185245,  2.81559614,
           2.93933983,  3.06308351,  3.1868272 ,  3.31057089,  3.43431458]]),
  'RIGHT': np.array([[16.34350288, 16.46135401, 16.57920514, 16.69705627, 16.81490741,
          16.93275854, 17.05060967, 17.05060967, 16.92686598, 16.80312229,
          16.67937861, 16.55563492, 16.43189123, 16.30814755, 16.18440386,
          16.06066017, 15.93691649, 15.8131728 , 15.68942911, 15.56568542],
         [ 2.65649712,  2.77434825,  2.89219938,  3.01005051,  3.12790164,
           3.24575277,  3.3636039 ,  3.3636039 ,  3.48734758,  3.61109127,
           3.73483496,  3.85857864,  3.98232233,  4.10606602,  4.2298097 ,
           4.35355339,  4.47729708,  4.60104076,  4.72478445,  4.84852814]])},
 3: {'LEFT': np.array([[6.34350288, 6.46135401, 6.57920514, 6.69705627, 6.81490741,
          6.93275854, 7.05060967, 7.05060967, 6.92686598, 6.80312229,
          6.67937861, 6.55563492, 6.43189123, 6.30814755, 6.18440386,
          6.06066017, 5.93691649, 5.8131728 , 5.68942911, 5.56568542],
         [5.34350288, 5.22565175, 5.10780062, 4.98994949, 4.87209836,
          4.75424723, 4.6363961 , 4.6363961 , 4.51265242, 4.38890873,
          4.26516504, 4.14142136, 4.01767767, 3.89393398, 3.7701903 ,
          3.64644661, 3.52270292, 3.39895924, 3.27521555, 3.15147186]]),
  'RIGHT': np.array([[6.34350288, 6.22565175, 6.10780062, 5.98994949, 5.87209836,
          5.75424723, 5.6363961 , 5.6363961 , 5.51265242, 5.38890873,
          5.26516504, 5.14142136, 5.01767767, 4.89393398, 4.7701903 ,
          4.64644661, 4.52270292, 4.39895924, 4.27521555, 4.15147186],
         [5.34350288, 5.46135401, 5.57920514, 5.69705627, 5.81490741,
          5.93275854, 6.05060967, 6.05060967, 5.92686598, 5.80312229,
          5.67937861, 5.55563492, 5.43189123, 5.30814755, 5.18440386,
          5.06066017, 4.93691649, 4.8131728 , 4.68942911, 4.56568542]])}}


# {0: {'LEFT': np.array([[ 3.65649712,  3.92538936,  4.18127408,  4.42183634,  4.64458198, 4.84775894,  5.03083297,  5.19451771,  5.34009121,  5.46913317,5.58344091,  5.68484494],
#          [-2.65649712, -2.67846449, -2.70312949, -2.73132214, -2.76461536, -2.80492727, -2.85433924, -2.91454888, -2.98721429, -3.07367665, -3.17484421, -3.29119973]]),
#   'RIGHT': np.array([[ 3.65649712,  3.67846449,  3.70312949,  3.73132214,  3.76461536, 3.80492727,  3.85433924,  3.91454888,  3.98721429,  4.07367665, 4.17484421,  4.29119973],
#          [-2.65649712, -2.92538936, -3.18127408, -3.42183634, -3.64458198, -3.84775894, -4.03083297, -4.19451771, -4.34009121, -4.46913317, -4.58344091, -4.68484494]])},
#  1: {'LEFT': np.array([[13.65649712, 13.67846449, 13.70312949, 13.73132214, 13.76461536, 13.80492727, 13.85433924, 13.91454888, 13.98721429, 14.07367665,14.17484421, 14.29119973],
#          [-5.34350288, -5.07461064, -4.81872592, -4.57816366, -4.35541802,-4.15224106, -3.96916703, -3.80548229, -3.65990879, -3.53086683,-3.41655909, -3.31515506]]),
#   'RIGHT': np.array([[13.65649712, 13.92538936, 14.18127408, 14.42183634, 14.64458198,14.84775894, 15.03083297, 15.19451771, 15.34009121, 15.46913317,15.58344091, 15.68484494],
#          [-5.34350288, -5.32153551, -5.29687051, -5.26867786, -5.23538464,-5.19507273, -5.14566076, -5.08545112, -5.01278571, -4.92632335,-4.82515579, -4.70880027]])},
#  2: {'LEFT': np.array([[16.34350288, 16.07461064, 15.81872592, 15.57816366, 15.35541802,15.15224106, 14.96916703, 14.80548229, 14.65990879, 14.53086683,14.41655909, 14.31515506],
#          [ 2.65649712,  2.67846449,  2.70312949,  2.73132214,  2.76461536, 2.80492727,  2.85433924,  2.91454888,  2.98721429,  3.07367665,3.17484421,  3.29119973]]),
#   'RIGHT': np.array([[16.34350288, 16.32153551, 16.29687051, 16.26867786, 16.23538464,16.19507273, 16.14566076, 16.08545112, 16.01278571, 15.92632335, 15.82515579, 15.70880027],
#          [ 2.65649712,  2.92538936,  3.18127408,  3.42183634,  3.64458198, 3.84775894,  4.03083297,  4.19451771,  4.34009121,  4.46913317,4.58344091,  4.68484494]])},
#  3: {'LEFT': np.array([[6.34350288, 6.32153551, 6.29687051, 6.26867786, 6.23538464, 6.19507273, 6.14566076, 6.08545112, 6.01278571, 5.92632335, 5.82515579, 5.70880027],
#          [5.34350288, 5.07461064, 4.81872592, 4.57816366, 4.35541802, 4.15224106, 3.96916703, 3.80548229, 3.65990879, 3.53086683,  3.41655909, 3.31515506]]),
#   'RIGHT': np.array([[6.34350288, 6.07461064, 5.81872592, 5.57816366, 5.35541802,5.15224106, 4.96916703, 4.80548229, 4.65990879, 4.53086683, 4.41655909, 4.31515506],
#          [5.34350288, 5.32153551, 5.29687051, 5.26867786, 5.23538464, 5.19507273, 5.14566076, 5.08545112, 5.01278571, 4.92632335,4.82515579, 4.70880027]])}}


# Start-End points of each gate pair
SE_point = {0: {'S': np.array([ 3.65649712, -2.65649712]),
  'E_LEFT': np.array([ 5.84852814, -3.43431458]),
  'E_RIGHT': np.array([ 4.43431458, -4.84852814])},
 1: {'S': np.array([13.65649712, -5.34350288]),
  'E_LEFT': np.array([14.43431458, -3.15147186]),
  'E_RIGHT': np.array([15.84852814, -4.56568542])},
 2: {'S': np.array([16.34350288,  2.65649712]),
  'E_LEFT': np.array([14.15147186,  3.43431458]),
  'E_RIGHT': np.array([15.56568542,  4.84852814])},
 3: {'S': np.array([6.34350288, 5.34350288]),
  'E_LEFT': np.array([5.56568542, 3.15147186]),
  'E_RIGHT': np.array([4.15147186, 4.56568542])}}


# {0: {'S': np.array([ 3.65649712, -2.65649712]),
#   'E_LEFT': np.array([ 5.68484494, -3.29119973]),
#   'E_RIGHT': np.array([ 4.29119973, -4.68484494])},
#  1: {'S': np.array([13.65649712, -5.34350288]),
#   'E_LEFT': np.array([14.29119973, -3.31515506]),
#   'E_RIGHT': np.array([15.68484494, -4.70880027])},
#  2: {'S': np.array([16.34350288,  2.65649712]),
#   'E_LEFT': np.array([14.31515506,  3.29119973]),
#   'E_RIGHT': np.array([15.70880027,  4.68484494])},
#  3: {'S': np.array([6.34350288, 5.34350288]),
#   'E_LEFT': np.array([5.70880027, 3.31515506]),
#   'E_RIGHT': np.array([4.31515506, 4.70880027])}}


# Initial Line origin(0,0) -> First start point
x_init = np.linspace(0, SE_point[0]['S'][0], 20)
y_init = np.linspace(0, SE_point[0]['S'][1], 20)
initial_line = np.array(list(zip(x_init, y_init)))

# Connect each start-end points of the gate pairs
lines = {0:{}, 1:{}, 2:{}, 3:{}}

for i in range(len(SE_point)):
    S_next = SE_point[i]["S"]
    # Left and Right end point
    E_L = SE_point[(3+i)%len(SE_point)]["E_LEFT"]
    E_R = SE_point[(3+i)%len(SE_point)]["E_RIGHT"]

    x_line_L = np.linspace(E_L[0], S_next[0], 30)
    y_line_L = np.linspace(E_L[1], S_next[1], 30)
    lines[i]["LEFT"] = np.array(list(zip(x_line_L, y_line_L)))

    x_line_R = np.linspace(E_R[0], S_next[0], 30)
    y_line_R = np.linspace(E_R[1], S_next[1], 30)
    lines[i]["RIGHT"] = np.array(list(zip(x_line_R, y_line_R)))

# Route Planner class
class RoutePlanner:
    def __init__(self, curves, SE_point, initial_line, lines):
        rospy.init_node("route_planner")

        self.curves = curves
        self.SE_point = SE_point
        self.initial_line = initial_line
        self.lines = lines
        
        self.yaw_S = [-1*np.pi/4, 1*np.pi/4, 3*np.pi/4, -3*np.pi/4]
        self.yaw_E = [[
            np.arctan2(lines[i]["LEFT"][-1][1]-lines[i]["LEFT"][0][1], lines[i]["LEFT"][-1][0]-lines[i]["LEFT"][0][0]),\
            np.arctan2(lines[i]["RIGHT"][-1][1]-lines[i]["RIGHT"][0][1], lines[i]["RIGHT"][-1][0]-lines[i]["RIGHT"][0][0])]
            for i in range(len(SE_point))]
    
        self.yaw_initial = np.arctan2(SE_point[0]['S'][1], SE_point[0]['S'][0])
        
        self.Z_tag = rospy.get_param("~Z_tag", 0.70)  # alwasys keep z as the height of tags 
        
        # Threshold for distance check
        self.threshold_start = rospy.get_param("~threshold_start", 0.10)
        self.threshold_curve = rospy.get_param("~threshold_curve", 0.10)
        self.threshold_line = rospy.get_param("~threshold_line", 0.10)

        self.current_pose = None
        self.prev_pose = None
        self.direction = "None"
        
        # ROS interfaces
        self.pose_sub = rospy.Subscriber("/pose_topic", PoseStamped, self.pose_callback)   # subsribe the EKF solution
        self.direction_sub = rospy.Subscriber("/goal", String, self.direction_callback)
        self.waypoint_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        self.gatepass_pub = rospy.Publisher("/gate_passed", Bool, queue_size = 1)
        self.run()

    def pose_callback(self, msg):
        self.prev_pose = self.current_pose
        self.current_pose = msg
    
    def direction_callback(self, msg):
        if msg.data in ["LEFT", "RIGHT"]:
            self.direction = msg.data
            rospy.loginfo(f"Direction set to: {self.direction}")
        else:
            rospy.logwarn(f"Invalid direction received: {msg.data}")
            self.direction = "None"

    def compute_distance(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
   
    def target_yaw(self, target_x, target_y):
        dx = target_x - self.current_waypoint.pose.position.x
        dy = target_y - self.current_waypoint.pose.position.y
        yaw = math.atan2(dy, dx)
        return yaw
    
    def generate_initial(self, idx):
        # Initial line to get the first gate pair from the origin
        x = self.initial_line[idx][0]
        y = self.initial_line[idx][1]
        z = self.Z_tag
        yaw = self.yaw_initial
        quat = quaternion_from_euler(0,0,yaw)
        
        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp

    def generate_curve(self, name, idx, direction):
        # Curve for passing through a gate
        x = self.curves[name][direction][0][idx]
        y = self.curves[name][direction][1][idx]
        z = self.Z_tag
        yaw = self.target_yaw(x, y)
        quat = quaternion_from_euler(0,0,yaw)
        
        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp

    def generate_line(self, name, idx, direction):
        # Line for connects the gate pairs e.g. 0th -> 1st
        x = self.lines[name][direction][idx][0]
        y = self.lines[name][direction][idx][1]
        z = self.Z_tag
        if (direction == "LEFT"):
            yaw = self.yaw_E[name][0]
        if (direction == "RIGHT"):
            yaw = self.yaw_E[name][1]
        quat = quaternion_from_euler(0,0,yaw)
        
        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp
    
    def generate_SE(self, type, name, direction):
        if type == "E":
            type = type + f"_{direction}"
        x = self.SE_point[name][type][0]
        y = self.SE_point[name][type][1]
        z = self.Z_tag
        if type=="S":
            yaw = self.yaw_S[name]
        if type == "E_LEFT":
            yaw = self.yaw_E[name][0]
        if type == "E_RIGHT":
            yaw = self.yaw_E[name][1]
        quat = quaternion_from_euler(0,0,yaw)
        
        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp

    def run(self):
        rate = rospy.Rate(20)
        self.current_waypoint = None
        self.mode = "initialize"
        self.direction = "None"
        self.name = 0

        rospy.loginfo("[Planner] Route Planner has intialized ")
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rate.sleep()
                continue

            if self.current_waypoint is None:
                self.idx = 0
                self.current_waypoint = self.select_waypoint(self.name, self.idx, self.direction)
                self.waypoint_pub.publish(self.current_waypoint)
                self.waiting_for_arrival = True
                
            if self.waiting_for_arrival:
                # Check distance
                dist = self.compute_distance(self.current_pose.pose.position, self.current_waypoint.pose.position)

                rospy.loginfo_throttle(1.0, f"[Planner] Sending pose {self.mode}: x={self.current_pose.pose.position.x:.2f} y={self.current_pose.pose.position.y:.2f} z={self.current_pose.pose.position.z:.2f}")
                rospy.loginfo_throttle(1.0, f"[Planner] Sending waypoint : x={self.current_waypoint.pose.position.x:.2f} y={self.current_waypoint.pose.position.y:.2f} z={self.current_waypoint.pose.position.z}")
                rospy.loginfo_throttle(1.0, f"[Planner] Current distance : {dist:.2f}")
                
                # Check mode
                if self.mode == 'initialize' or 'line':
                    dist_threshold = self.threshold_line
                if self.mode == "curve":
                    dist_threshold == self.threshold_curve
                if self.mode == "S" or "E":
                    dist_threshold == self.threshold_start
                
                # Check if waypoint has reached
                if dist < dist_threshold:
                    # Waypoint reached
                    rospy.loginfo_throttle(1.0, "[Planner] Reached waypoint")

                    # waiting for the QR code answer on S
                    if self.mode == "S" and self.direction == "None":
                        rospy.loginfo_throttle(0.5, "[Planner] Waiting for the direction ...")
                    else:
                        self.advance_route()
                        self.current_waypoint = self.select_waypoint(self.name, self.idx, self.direction)
                self.waypoint_pub.publish(self.current_waypoint)
            
            rate.sleep()

    def select_waypoint(self, name, idx, direction):
        if self.mode == "line":
            return self.generate_line(name, idx, direction)
        elif self.mode == "S":
            return self.generate_SE("S", name, direction)
        elif self.mode == "E":
            return self.generate_SE("E", name, direction)
        elif self.mode == "curve":
            return self.generate_curve(name, idx, direction)
        elif self.mode == "initialize":
            return self.generate_initial(idx)
        else:
            rospy.loginfo("invalid mode")
            return PoseStamped()

    def advance_route(self):
        # Find the next state of the FSM
        if self.mode == "line":
            # Follow the line or go to the start point state
            if self.idx < len(self.lines[0])-1:
                self.idx +=1
            else:
                self.idx = 0
                self.mode = "S"
                self.direction = "None"
        elif self.mode == "S":
            # Pass the gate along the curve
            self.idx = 0
            self.name = self.name
            self.mode = "curve"
        elif self.mode == "E":
            # Passed the gate, follow the line for the next gate pair
            self.idx = 0
            self.name = (self.name + 1) % 4
            self.mode = "line"
        elif self.mode == "curve":
            # Followed the curve, change to the end point state
            if self.idx < curves[self.name]["LEFT"].shape[1]-1:
                self.idx +=1
            else:
                self.idx = 0
                self.mode = "E"
                gate_pass = Bool()
                gate_pass.data = True
                self.gatepass_pub.publish(gate_pass)
        elif self.mode == "initialize":
            # Initial state; origin -> first start point
            if self.idx < len(self.initial_line)-1:
                self.idx +=1
            else:
                self.idx = 0
                self.mode = "S"
        else:
            rospy.loginfo("invalid mode")
            return

if __name__ == '__main__':
    RoutePlanner(curves, SE_point, initial_line, lines)
    rospy.spin()
