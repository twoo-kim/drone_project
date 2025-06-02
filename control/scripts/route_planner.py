#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {'0': {'LEFT': np.array([[ 4.15147186,  4.35135338,  4.54345339,  4.72739197,  4.90234435,
         5.06729484,  5.22125275,  5.36332096,  5.49258217,  5.60799728,
         5.70880209,  5.79453619,  5.86511875,  5.92061395,  5.96113889,
         5.98688576,  5.99803993,  5.99489628,  5.97782139,  5.94735127,
         5.90410307,  5.84852814],
       [-3.15147186, -3.08908791, -3.04006829, -3.00463675, -2.98325021,
        -2.97653715, -2.98509271, -3.0091107 , -3.04854901, -3.1033771 ,
        -3.17372128, -3.2596573 , -3.36116112, -3.47788545, -3.60915735,
        -3.75386754, -3.91085581, -4.07904001, -4.25751816, -4.44555438,
        -4.64264103, -4.84852814]]), 'RIGHT': np.array([[ 4.15147186,  4.08908791,  4.04006829,  4.00463675,  3.98325021,
         3.97653715,  3.98509271,  4.0091107 ,  4.04854901,  4.1033771 ,
         4.17372128,  4.2596573 ,  4.36116112,  4.47788545,  4.60915735,
         4.75386754,  4.91085581,  5.07904001,  5.25751816,  5.44555438,
         5.64264103,  5.84852814],
       [-3.15147186, -3.35135338, -3.54345339, -3.72739197, -3.90234435,
        -4.06729484, -4.22125275, -4.36332096, -4.49258217, -4.60799728,
        -4.70880209, -4.79453619, -4.86511875, -4.92061395, -4.96113889,
        -4.98688576, -4.99803993, -4.99489628, -4.97782139, -4.94735127,
        -4.90410307, -4.84852814]])}, '1': {'LEFT': np.array([[14.15147186, 14.08908791, 14.04006829, 14.00463675, 13.98325021,
        13.97653715, 13.98509271, 14.0091107 , 14.04854901, 14.1033771 ,
        14.17372128, 14.2596573 , 14.36116112, 14.47788545, 14.60915735,
        14.75386754, 14.91085581, 15.07904001, 15.25751816, 15.44555438,
        15.64264103, 15.84852814],
       [-4.84852814, -4.64864662, -4.45654661, -4.27260803, -4.09765565,
        -3.93270516, -3.77874725, -3.63667904, -3.50741783, -3.39200272,
        -3.29119791, -3.20546381, -3.13488125, -3.07938605, -3.03886111,
        -3.01311424, -3.00196007, -3.00510372, -3.02217861, -3.05264873,
        -3.09589693, -3.15147186]]), 'RIGHT': np.array([[14.15147186, 14.35135338, 14.54345339, 14.72739197, 14.90234435,
        15.06729484, 15.22125275, 15.36332096, 15.49258217, 15.60799728,
        15.70880209, 15.79453619, 15.86511875, 15.92061395, 15.96113889,
        15.98688576, 15.99803993, 15.99489628, 15.97782139, 15.94735127,
        15.90410307, 15.84852814],
       [-4.84852814, -4.91091209, -4.95993171, -4.99536325, -5.01674979,
        -5.02346285, -5.01490729, -4.9908893 , -4.95145099, -4.8966229 ,
        -4.82627872, -4.7403427 , -4.63883888, -4.52211455, -4.39084265,
        -4.24613246, -4.08914419, -3.92095999, -3.74248184, -3.55444562,
        -3.35735897, -3.15147186]])}, '2': {'LEFT': np.array([[15.84852814, 15.64864662, 15.45654661, 15.27260803, 15.09765565,
        14.93270516, 14.77874725, 14.63667904, 14.50741783, 14.39200272,
        14.29119791, 14.20546381, 14.13488125, 14.07938605, 14.03886111,
        14.01311424, 14.00196007, 14.00510372, 14.02217861, 14.05264873,
        14.09589693, 14.15147186],
       [ 3.15147186,  3.08908791,  3.04006829,  3.00463675,  2.98325021,
         2.97653715,  2.98509271,  3.0091107 ,  3.04854901,  3.1033771 ,
         3.17372128,  3.2596573 ,  3.36116112,  3.47788545,  3.60915735,
         3.75386754,  3.91085581,  4.07904001,  4.25751816,  4.44555438,
         4.64264103,  4.84852814]]), 'RIGHT': np.array([[15.84852814, 15.91091209, 15.95993171, 15.99536325, 16.01674979,
        16.02346285, 16.01490729, 15.9908893 , 15.95145099, 15.8966229 ,
        15.82627872, 15.7403427 , 15.63883888, 15.52211455, 15.39084265,
        15.24613246, 15.08914419, 14.92095999, 14.74248184, 14.55444562,
        14.35735897, 14.15147186],
       [ 3.15147186,  3.35135338,  3.54345339,  3.72739197,  3.90234435,
         4.06729484,  4.22125275,  4.36332096,  4.49258217,  4.60799728,
         4.70880209,  4.79453619,  4.86511875,  4.92061395,  4.96113889,
         4.98688576,  4.99803993,  4.99489628,  4.97782139,  4.94735127,
         4.90410307,  4.84852814]])}, '3': {'LEFT': np.array([[5.84852814, 5.91091209, 5.95993171, 5.99536325, 6.01674979,
        6.02346285, 6.01490729, 5.9908893 , 5.95145099, 5.8966229 ,
        5.82627872, 5.7403427 , 5.63883888, 5.52211455, 5.39084265,
        5.24613246, 5.08914419, 4.92095999, 4.74248184, 4.55444562,
        4.35735897, 4.15147186],
       [4.84852814, 4.64864662, 4.45654661, 4.27260803, 4.09765565,
        3.93270516, 3.77874725, 3.63667904, 3.50741783, 3.39200272,
        3.29119791, 3.20546381, 3.13488125, 3.07938605, 3.03886111,
        3.01311424, 3.00196007, 3.00510372, 3.02217861, 3.05264873,
        3.09589693, 3.15147186]]), 'RIGHT': np.array([[5.84852814, 5.64864662, 5.45654661, 5.27260803, 5.09765565,
        4.93270516, 4.77874725, 4.63667904, 4.50741783, 4.39200272,
        4.29119791, 4.20546381, 4.13488125, 4.07938605, 4.03886111,
        4.01311424, 4.00196007, 4.00510372, 4.02217861, 4.05264873,
        4.09589693, 4.15147186],
       [4.84852814, 4.91091209, 4.95993171, 4.99536325, 5.01674979,
        5.02346285, 5.01490729, 4.9908893 , 4.95145099, 4.8966229 ,
        4.82627872, 4.7403427 , 4.63883888, 4.52211455, 4.39084265,
        4.24613246, 4.08914419, 3.92095999, 3.74248184, 3.55444562,
        3.35735897, 3.15147186]])}}

# Start-End points of each gate pair
SE_point = {'0': {'S': np.array([[ 4.15147186],
       [-3.15147186]]), 'E': np.array([[ 5.84852814],
       [-4.84852814]])}, '1': {'S': np.array([[14.15147186],
       [-4.84852814]]), 'E': np.array([[15.84852814],
       [-3.15147186]])}, '2': {'S': np.array([[15.84852814],
       [ 3.15147186]]), 'E': np.array([[14.15147186],
       [ 4.84852814]])}, '3': {'S': np.array([[5.84852814],
       [4.84852814]]), 'E': np.array([[4.15147186],
       [3.15147186]])}}

# Initial Line origin(0,0) -> First start point
x_init = np.linspace(0, SE_point[0]['S'][0], 10)
y_init = np.linspace(0, SE_point[0]['S'][1], 10)
initial_line = np.array(list(zip(x_init, y_init)))

# Connect each start-end points of the gate pairs
lines = dict()
for i in range(4):
    x_values = np.linspace(SE_point[(3+i)%4]['E'][0], SE_point[i]['S'][0], 20)
    y_values = np.linspace(SE_point[(3+i)%4]['E'][1], SE_point[i]['S'][1], 20)
    lines[i] = np.array(list(zip(x_values, y_values)))

# Route Planner class
class RoutePlanner:
    def __init__(self, curves, SE_point, initial_line, lines):
        rospy.init_node("route_planner")

        self.curves = curves
        self.SE_point = SE_point
        self.initial_line = initial_line
        self.lines = lines

        self.Z_tag = rospy.get_param("~Z_tag", 0.70)  # alwasys keep z as the height of tags 
        
        # Threshold for distance check
        self.threshold_start = rospy.get_param("~threshold_start", 0.05)
        self.threshold_curve = rospy.get_param("~threshold_curve", 0.07)
        self.threshold_line = rospy.get_param("~threshold_line", 0.1)

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
   
    def generate_initial(self, idx):
        # Initial line to get the first gate pair from the origin
        x = self.initial_line[idx][0]
        y = self.initial_line[idx][1]
        z = self.Z_tag

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        return wp

    def generate_curve(self, name, idx, direction):
        # Curve for passing through a gate
        x = self.curves[name][direction][0][idx]
        y = self.curves[name][direction][1][idx]
        z = self.Z_tag

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        return wp

    def generate_line(self, name, idx):
        # Line for connects the gate pairs e.g. 0th -> 1st
        x = self.lines[name][idx][0]
        y = self.lines[name][idx][1]
        z = self.Z_tag

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        return wp
    
    def generate_SE(self, type,  name):
        x = self.SE_point[name][type][0]
        y = self.SE_point[name][type][1]
        z = self.Z_tag

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        return wp

    def run(self):
        rate = rospy.Rate(20)
        self.current_waypoint = None
        self.mode = "initialize"
        self.direction = "LEFT"
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
                        rospy.loginfo_throttle(1.0, "[Planner] Waiting for the direction ...")
                    else:
                        self.advance_route()
                        self.current_waypoint = self.select_waypoint(self.name, self.idx, self.direction)
                self.waypoint_pub.publish(self.current_waypoint)
            
            rate.sleep()

    def select_waypoint(self, name, idx, direction):
        if self.mode == "line":
            return self.generate_line(name, idx)
        elif self.mode == "S":
            return self.generate_SE("S",  name)
        elif self.mode == "E":
            return self.generate_SE("E",  name)
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
            self.direction = "None"
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

