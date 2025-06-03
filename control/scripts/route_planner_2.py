#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {0: {'LEFT': np.array([[ 3.65649712,  3.91763272,  4.16693309,  4.40229484,  4.62182989,
           4.82451166,  5.01038629,  5.17993045,  5.33375368,  5.47233515,
           5.59618356,  5.70622531,  5.80332091],
         [-2.65649712, -2.71503543, -2.7731686 , -2.83171292, -2.89157896,
          -2.95339346, -3.01850518, -3.08895065, -3.16655655, -3.25270407,
          -3.34909285, -3.45778229, -3.57995522]]),
  'RIGHT': np.array([[ 3.65649712,  3.68971077,  3.72394803,  3.76039018,  3.80029759,
           3.84450368,  3.89452004,  3.95257818,  4.02065691,  4.10025019,
           4.19320651,  4.30172691,  4.42703451],
         [-2.65649712, -2.94295739, -3.21615366, -3.47361758, -3.71311125,
          -3.93340143, -4.13437143, -4.31630292, -4.47965332, -4.62478904,
          -4.7520699 , -4.86228069, -4.95624163]])},
 1: {'LEFT': np.array([[13.65649712, 13.71503543, 13.7731686 , 13.83171292, 13.89157896,
          13.95339346, 14.01850518, 14.08895065, 14.16655655, 14.25270407,
          14.34909285, 14.45778229, 14.57995522],
         [-5.34350288, -5.08236728, -4.83306691, -4.59770516, -4.37817011,
          -4.17548834, -3.98961371, -3.82006955, -3.66624632, -3.52766485,
          -3.40381644, -3.29377469, -3.19667909]]),
  'RIGHT': np.array([[13.65649712, 13.94295739, 14.21615366, 14.47361758, 14.71311125,
          14.93340143, 15.13437143, 15.31630292, 15.47965332, 15.62478904,
          15.7520699 , 15.86228069, 15.95624163],
         [-5.34350288, -5.31028923, -5.27605197, -5.23960982, -5.19970241,
          -5.15549632, -5.10547996, -5.04742182, -4.97934309, -4.89974981,
          -4.80679349, -4.69827309, -4.57296549]])},
 2: {'LEFT': np.array([[16.34350288, 16.08236728, 15.83306691, 15.59770516, 15.37817011,
          15.17548834, 14.98961371, 14.82006955, 14.66624632, 14.52766485,
          14.40381644, 14.29377469, 14.19667909],
         [ 2.65649712,  2.71503543,  2.7731686 ,  2.83171292,  2.89157896,
           2.95339346,  3.01850518,  3.08895065,  3.16655655,  3.25270407,
           3.34909285,  3.45778229,  3.57995522]]),
  'RIGHT': np.array([[16.34350288, 16.31028923, 16.27605197, 16.23960982, 16.19970241,
          16.15549632, 16.10547996, 16.04742182, 15.97934309, 15.89974981,
          15.80679349, 15.69827309, 15.57296549],
         [ 2.65649712,  2.94295739,  3.21615366,  3.47361758,  3.71311125,
           3.93340143,  4.13437143,  4.31630292,  4.47965332,  4.62478904,
           4.7520699 ,  4.86228069,  4.95624163]])},
 3: {'LEFT': np.array([[6.34350288, 6.28496457, 6.2268314 , 6.16828708, 6.10842104,
          6.04660654, 5.98149482, 5.91104935, 5.83344345, 5.74729593,
          5.65090715, 5.54221771, 5.42004478],
         [5.34350288, 5.08236728, 4.83306691, 4.59770516, 4.37817011,
          4.17548834, 3.98961371, 3.82006955, 3.66624632, 3.52766485,
          3.40381644, 3.29377469, 3.19667909]]),
  'RIGHT': np.array([[6.34350288, 6.05704261, 5.78384634, 5.52638242, 5.28688875,
          5.06659857, 4.86562857, 4.68369708, 4.52034668, 4.37521096,
          4.2479301 , 4.13771931, 4.04375837],
         [5.34350288, 5.31028923, 5.27605197, 5.23960982, 5.19970241,
          5.15549632, 5.10547996, 5.04742182, 4.97934309, 4.89974981,
          4.80679349, 4.69827309, 4.57296549]])}}


# Start-End points of each gate pair
SE_point = {0: {'S': np.array([[ 3.65649712],
         [-2.65649712]]),
  'E_LEFT': np.array([[ 5.80332091],
         [-3.57995522]]),
  'E_RIGHT': np.array([[ 4.42703451],
         [-4.95624163]])},
 1: {'S': np.array([[13.65649712],
         [-5.34350288]]),
  'E_LEFT': np.array([[14.57995522],
         [-3.19667909]]),
  'E_RIGHT': np.array([[15.95624163],
         [-4.57296549]])},
 2: {'S': np.array([[16.34350288],
         [ 2.65649712]]),
  'E_LEFT': np.array([[14.19667909],
         [ 3.57995522]]),
  'E_RIGHT': np.array([[15.57296549],
         [ 4.95624163]])},
 3: {'S': np.array([[6.34350288],
         [5.34350288]]),
  'E_LEFT': np.array([[5.42004478],
         [3.19667909]]),
  'E_RIGHT': np.array([[4.04375837],
         [4.57296549]])}}

# Initial Line origin(0,0) -> First start point
x_init = np.linspace(0, SE_point[0]['S'][0], 10)
y_init = np.linspace(0, SE_point[0]['S'][1], 10)
initial_line = np.array(list(zip(x_init, y_init)))

# Connect each start-end points of the gate pairs
lines = {0:{}, 1:{}, 2:{}, 3:{}}

for i in range(4):
    S_next = SE_point[(i+1)%len(SE_point)]["S"]
    E_L = SE_point[i]["E_LEFT"]
    E_R = SE_point[i]["E_RIGHT"]

    x_line_L = np.linspace(E_L[0][0], S_next[0][0], 20)
    y_line_L = np.linspace(E_L[1][0], S_next[1][0], 20)
    lines[i]["LEFT"] = np.array(list(zip(x_line_L, y_line_L)))

    x_line_R = np.linspace(E_R[0][0], S_next[0][0], 20)
    y_line_R = np.linspace(E_R[1][0], S_next[1][0], 20)
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
        
        # [left_yaw, right_yaw] for each E  #[-np.pi/2, 0, np.pi/2, np.pi]
        self.yaw_E = [[
            np.arctan2(lines[i]["LEFT"][-1][1]-lines[i]["LEFT"][0][1], lines[i]["LEFT"][-1][0]-lines[i]["LEFT"][0][0]), np.arctan2(lines[i]["RIGHT"][-1][1]-lines[i]["RIGHT"][0][1], lines[i]["RIGHT"][-1][0]-lines[i]["RIGHT"][0][0])
        ] for i in range(4)]
        
        self.yaw_initial = np.arctan2(SE_point[0]['S'][1][0], SE_point[0]['S'][0][0])
        
        self.Z_tag = rospy.get_param("~Z_tag", 0.70)  # alwasys keep z as the height of tags 
        
        # Threshold for distance check
        self.threshold_start = rospy.get_param("~threshold_start", 0.10)
        self.threshold_curve = rospy.get_param("~threshold_curve", 0.10)
        self.threshold_line = rospy.get_param("~threshold_line", 0.10)

        self.current_pose = None
        self.prev_pose = None
        self.direction = "None"
        
        # ROS interfaces /mavros/local_position/pose   /pose_topic
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)   # subsribe the EKF solution
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
        yaw = self.yaw_S[name]
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
        yaw = self.yaw_E[name]
        quat = quaternion_from_euler(0,0,yaw)
        
        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp
    
    def generate_SE(self, type,  name, direction):
        if type == "E":
            type = type + f"_{direction}"
        x = self.SE_point[name][type][0][0]
        y = self.SE_point[name][type][1][0]
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

                rospy.loginfo_throttle(1.0, f"[Planner] Sending pose : x={self.current_pose.pose.position.x:.2f} y={self.current_pose.pose.position.y:.2f} z={self.current_pose.pose.position.z:.2f}")
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
            return self.generate_SE("S",  name, direction)
        elif self.mode == "E":
            return self.generate_SE("E",  name, direction)
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
            if self.idx < len(self.initial_line)-1:
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
            #self.direction = "None" keep it to proceed into 'line'
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

