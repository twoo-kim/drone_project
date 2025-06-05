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
         [-2.65649712, -2.67846449, -2.70312949, -2.73132214, -2.76461536, -2.80492727, -2.85433924, -2.91454888, -2.98721429, -3.07367665, -3.17484421, -3.29119973]]),
  'RIGHT': np.array([[ 3.65649712,  3.67846449,  3.70312949,  3.73132214,  3.76461536,
           3.80492727,  3.85433924,  3.91454888,  3.98721429,  4.07367665,
           4.17484421,  4.29119973],
         [-2.65649712, -2.92538936, -3.18127408, -3.42183634, -3.64458198,
          -3.84775894, -4.03083297, -4.19451771, -4.34009121, -4.46913317,
          -4.58344091, -4.68484494]])},
 1: {'LEFT': np.array([[13.65649712, 13.67846449, 13.70312949, 13.73132214, 13.76461536,
          13.80492727, 13.85433924, 13.91454888, 13.98721429, 14.07367665,
          14.17484421, 14.29119973],
         [-5.34350288, -5.07461064, -4.81872592, -4.57816366, -4.35541802,
          -4.15224106, -3.96916703, -3.80548229, -3.65990879, -3.53086683,
          -3.41655909, -3.31515506]]),
  'RIGHT': np.array([[13.65649712, 13.92538936, 14.18127408, 14.42183634, 14.64458198,
          14.84775894, 15.03083297, 15.19451771, 15.34009121, 15.46913317,
          15.58344091, 15.68484494],
         [-5.34350288, -5.32153551, -5.29687051, -5.26867786, -5.23538464,
          -5.19507273, -5.14566076, -5.08545112, -5.01278571, -4.92632335,
          -4.82515579, -4.70880027]])},
 2: {'LEFT': np.array([[16.34350288, 16.07461064, 15.81872592, 15.57816366, 15.35541802,
          15.15224106, 14.96916703, 14.80548229, 14.65990879, 14.53086683,
          14.41655909, 14.31515506],
         [ 2.65649712,  2.67846449,  2.70312949,  2.73132214,  2.76461536,
           2.80492727,  2.85433924,  2.91454888,  2.98721429,  3.07367665,
           3.17484421,  3.29119973]]),
  'RIGHT': np.array([[16.34350288, 16.32153551, 16.29687051, 16.26867786, 16.23538464,
          16.19507273, 16.14566076, 16.08545112, 16.01278571, 15.92632335,
          15.82515579, 15.70880027],
         [ 2.65649712,  2.92538936,  3.18127408,  3.42183634,  3.64458198,
           3.84775894,  4.03083297,  4.19451771,  4.34009121,  4.46913317,
           4.58344091,  4.68484494]])},
 3: {'LEFT': np.array([[6.34350288, 6.32153551, 6.29687051, 6.26867786, 6.23538464,
          6.19507273, 6.14566076, 6.08545112, 6.01278571, 5.92632335,
          5.82515579, 5.70880027],
         [5.34350288, 5.07461064, 4.81872592, 4.57816366, 4.35541802,
          4.15224106, 3.96916703, 3.80548229, 3.65990879, 3.53086683,
          3.41655909, 3.31515506]]),
  'RIGHT': np.array([[6.34350288, 6.07461064, 5.81872592, 5.57816366, 5.35541802,
          5.15224106, 4.96916703, 4.80548229, 4.65990879, 4.53086683,
          4.41655909, 4.31515506],
         [5.34350288, 5.32153551, 5.29687051, 5.26867786, 5.23538464,
          5.19507273, 5.14566076, 5.08545112, 5.01278571, 4.92632335,
          4.82515579, 4.70880027]])}}

# {
#     0: {'LEFT': np.array([[3.75649712, 4.01763272,  4.26693309,  4.50229484,  4.72182989, 4.92451166, 5.11038629, 5.27993045,  5.43375368,  5.57233515, 5.69618356,  5.80622531, 5.90332091],
#                           [-2.65649712, -2.71503543, -2.7731686 , -2.83171292, -2.89157896, -2.95339346, -3.01850518, -3.08895065, -3.16655655, -3.25270407, -3.34909285, -3.45778229, -3.57995522]]),
#         'RIGHT': np.array([[ 3.65649712,  3.78971077,  3.82394803,  3.86039018,  3.90029759, 3.94450368,  3.99452004,  4.05257818,  4.12065691,  4.20025019, 4.29320651,  4.40172691,  4.52703451],
#                            [-2.65649712, -2.94295739, -3.21615366, -3.47361758, -3.71311125, -3.93340143, -4.13437143, -4.31630292, -4.47965332, -4.62478904, -4.7520699 , -4.86228069, -4.95624163]])},
#     1: {'LEFT': np.array([[13.65649712, 13.61503543, 13.6731686 , 13.73171292, 13.79157896, 13.85339346, 13.91850518, 13.98895065, 14.06655655, 14.15270407, 14.24909285, 14.35778229, 14.47995522],
#                           [-5.34350288, -5.08236728, -4.83306691, -4.59770516, -4.37817011, -4.17548834, -3.98961371, -3.82006955, -3.66624632, -3.52766485, -3.40381644, -3.29377469, -3.19667909]]),
#         'RIGHT': np.array([[13.65649712, 13.94295739, 14.21615366, 14.47361758, 14.71311125, 14.93340143, 15.13437143, 15.31630292, 15.47965332, 15.62478904, 15.7520699 , 15.86228069, 15.95624163],
#                             [-5.34350288, -5.31028923, -5.27605197, -5.23960982, -5.19970241, -5.15549632, -5.10547996, -5.04742182, -4.97934309, -4.89974981, -4.80679349, -4.69827309, -4.57296549]])},
#     2: {'LEFT': np.array([[16.34350288, 16.08236728, 15.83306691, 15.59770516, 15.37817011, 15.17548834, 14.98961371, 14.82006955, 14.66624632, 14.52766485, 14.40381644, 14.29377469, 14.19667909],
#                           [ 2.55649712,  2.61503543,  2.6731686 ,  2.73171292,  2.79157896, 2.85339346,  2.91850518,  2.98895065,  3.06655655,  3.15270407, 3.24909285,  3.35778229,  3.47995522]]),
#         'RIGHT': np.array([[16.34350288, 16.31028923, 16.27605197, 16.23960982, 16.19970241, 16.15549632, 16.10547996, 16.04742182, 15.97934309, 15.89974981, 15.80679349, 15.69827309, 15.57296549],
#                            [ 2.55649712,  2.84295739,  3.11615366,  3.37361758,  3.61311125, 3.83340143,  4.03437143,  4.21630292,  4.37965332,  4.52478904, 4.6520699 ,  4.76228069,  4.85624163]])},
#     3: {'LEFT': np.array([[6.44350288, 6.38496457, 6.3268314 , 6.26828708, 6.20842104, 6.14660654, 6.08149482, 6.01104935, 5.93344345, 5.84729593, 5.75090715, 5.64221771, 5.52004478],
#                           [5.34350288, 5.08236728, 4.83306691, 4.59770516, 4.37817011, 4.17548834, 3.98961371, 3.82006955, 3.66624632, 3.52766485, 3.40381644, 3.29377469, 3.19667909]]),
#         'RIGHT': np.array([[6.44350288, 6.15704261, 5.88384634, 5.62638242, 5.38688875, 5.16659857, 4.96562857, 4.78369708, 4.62034668, 4.47521096, 4.3479301 , 4.23771931, 4.14375837],
#                            [5.34350288, 5.31028923, 5.27605197, 5.23960982, 5.19970241, 5.15549632, 5.10547996, 5.04742182, 4.97934309, 4.89974981, 4.80679349, 4.69827309, 4.57296549]])}
# }



# Start-End points of each gate pair
SE_point = {0: {'S': np.array([ 3.65649712, -2.65649712]),
  'E_LEFT': np.array([ 5.68484494, -3.29119973]),
  'E_RIGHT': np.array([ 4.29119973, -4.68484494])},
 1: {'S': np.array([13.65649712, -5.34350288]),
  'E_LEFT': np.array([14.29119973, -3.31515506]),
  'E_RIGHT': np.array([15.68484494, -4.70880027])},
 2: {'S': np.array([16.34350288,  2.65649712]),
  'E_LEFT': np.array([14.31515506,  3.29119973]),
  'E_RIGHT': np.array([15.70880027,  4.68484494])},
 3: {'S': np.array([6.34350288, 5.34350288]),
  'E_LEFT': np.array([5.70880027, 3.31515506]),
  'E_RIGHT': np.array([4.31515506, 4.70880027])}}

# {
#     0: {'S': np.array([3.65649712,-2.65649712]),
#         'E_LEFT': np.array([5.80332091,-3.57995522]),
#         'E_RIGHT': np.array([ 4.42703451,-4.95624163])},
#     1: {'S': np.array([13.65649712,-5.34350288]),
#         'E_LEFT': np.array([14.57995522,-3.19667909]),
#         'E_RIGHT': np.array([15.95624163,-4.57296549])},
#     2: {'S': np.array([16.34350288, 2.65649712]),
#         'E_LEFT': np.array([14.19667909, 3.57995522]),
#         'E_RIGHT': np.array([15.57296549, 4.95624163])},
#     3: {'S': np.array([6.34350288,5.34350288]),
#         'E_LEFT': np.array([5.42004478,3.19667909]),
#         'E_RIGHT': np.array([4.04375837,4.57296549])}
# }

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
