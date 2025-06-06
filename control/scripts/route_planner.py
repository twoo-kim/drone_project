#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {0: {'LEFT': np.array([[ 3.65649712,  3.92538936,  4.18127408,  4.42183634,  4.64458198, 4.84775894,  5.03083297,  5.19451771,  5.34009121,  5.46913317,5.58344091,  5.68484494],
         [-2.75649712, -2.77846449, -2.80312949, -2.83132214, -2.86461536, -2.90492727, -2.95433924, -3.01454888, -3.08721429, -3.17367665, -3.27484421, -3.39119973]]),
  'RIGHT': np.array([[ 3.65649712,  3.67846449,  3.70312949,  3.73132214,  3.76461536,
           3.80492727,  3.85433924,  3.91454888,  3.98721429,  4.07367665,
           4.17484421,  4.29119973],
         [-2.65649712, -2.92538936, -3.18127408, -3.42183634, -3.64458198,
          -3.84775894, -4.03083297, -4.19451771, -4.34009121, -4.46913317,
          -4.58344091, -4.68484494]])},
 1: {'LEFT': np.array([[13.45649712, 13.47846449, 13.50312949, 13.53132214, 13.56461536,
          13.60492727, 13.65433924, 13.71454888, 13.78721429, 13.87367665,
          13.97484421, 14.09119973],
         [-5.34350288, -5.07461064, -4.81872592, -4.57816366, -4.35541802,
          -4.15224106, -3.96916703, -3.80548229, -3.65990879, -3.53086683,
          -3.41655909, -3.31515506]]),
  'RIGHT': np.array([[13.65649712, 13.92538936, 14.18127408, 14.42183634, 14.64458198,
          14.84775894, 15.03083297, 15.19451771, 15.34009121, 15.46913317,
          15.58344091, 15.68484494],
         [-5.34350288, -5.32153551, -5.29687051, -5.26867786, -5.23538464,
          -5.19507273, -5.14566076, -5.08545112, -5.01278571, -4.92632335,
          -4.82515579, -4.70880027]])},
 2: {'LEFT': np.array([[16.44350288, 16.17461064, 15.91872592, 15.67816366, 15.45541802,
          15.25224106, 15.06916703, 14.90548229, 14.75990879, 14.63086683,
          14.51655909, 14.41515506],
         [ 3.05649712,  3.07846449,  3.10312949,  3.13132214,  3.16461536,
           3.20492727,  3.25433924,  3.31454888,  3.38721429,  3.47367665,
           3.57484421,  3.69119973]]),
  'RIGHT': np.array([[16.34350288, 16.32153551, 16.29687051, 16.26867786, 16.23538464,
          16.19507273, 16.14566076, 16.08545112, 16.01278571, 15.92632335,
          15.82515579, 15.70880027],
         [ 2.65649712,  2.92538936,  3.18127408,  3.42183634,  3.64458198,
           3.84775894,  4.03083297,  4.19451771,  4.34009121,  4.46913317,
           4.58344091,  4.68484494]])},
 3: {'LEFT': np.array([[6.04350288, 6.02153551, 5.99687051, 5.96867786, 5.93538464,
          5.89507273, 5.84566076, 5.78545112, 5.71278571, 5.62632335,
          5.52515579, 5.40880027],
         [5.44350288, 5.17461064, 4.91872592, 4.67816366, 4.45541802,
          4.25224106, 4.06916703, 3.90548229, 3.75990879, 3.63086683,
          3.51655909, 3.41515506]]),
  'RIGHT': np.array([[6.34350288, 6.07461064, 5.81872592, 5.57816366, 5.35541802,
          5.15224106, 4.96916703, 4.80548229, 4.65990879, 4.53086683,
          4.41655909, 4.31515506],
         [5.44350288, 5.42153551, 5.39687051, 5.36867786, 5.33538464,
          5.29507273, 5.24566076, 5.18545112, 5.11278571, 5.02632335,
          4.92515579, 4.80880027]])}}


# Start-End points of each gate pair
SE_point = {0: {'S': np.array([ 3.65649712, -2.75649712]),
  'E_LEFT': np.array([ 5.68484494, -3.39119973]),
  'E_RIGHT': np.array([ 4.29119973, -4.68484494])},
 1: {'S': np.array([13.45649712, -5.34350288]),
  'E_LEFT': np.array([14.09119973, -3.31515506]),
  'E_RIGHT': np.array([15.68484494, -4.70880027])},
 2: {'S': np.array([16.44350288,  3.05649712]),
  'E_LEFT': np.array([14.41515506,  3.69119973]),
  'E_RIGHT': np.array([15.70880027,  4.68484494])},
 3: {'S': np.array([6.04350288, 5.44350288]),
  'E_LEFT': np.array([5.40880027, 3.41515506]),
  'E_RIGHT': np.array([4.31515506, 4.70880027])}}

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

    x_line_L = np.linspace(E_L[0], S_next[0], 20)
    y_line_L = np.linspace(E_L[1], S_next[1], 20)
    lines[i]["LEFT"] = np.array(list(zip(x_line_L, y_line_L)))

    x_line_R = np.linspace(E_R[0], S_next[0], 20)
    y_line_R = np.linspace(E_R[1], S_next[1], 20)
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
        yaw_x, yaw_y = SE_point[name]["S"]
        yaw = self.target_yaw(yaw_x, yaw_y)
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
