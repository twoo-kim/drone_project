#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {'0': {'LEFT': np.array([[ 3.93933983,  4.18241126,  4.41042686,  4.62308121,  4.81999761,
         5.00102088,  5.16615376,  5.31574308,  5.45036475,  5.57074083,
         5.6774853 ,  5.77096321,  5.85140192,  5.9190712 ,  5.97439963,
         6.01791222,  6.05015084,  6.07162327,  6.0829173 ,  6.08455828,
         6.07704978,  6.06066017],
       [-2.93933983, -2.90731374, -2.88485908, -2.87252969, -2.87108497,
        -2.88119511, -2.90349177, -2.93879062, -2.9879222 , -3.05158914,
        -3.13021342, -3.22412629, -3.33357113, -3.4588643 , -3.6004129 ,
        -3.75865553, -3.9339283 , -4.12625652, -4.33553277, -4.56133344,
        -4.80319742, -5.06066017]]), 'RIGHT': np.array([[ 3.93933983,  3.90731374,  3.88485908,  3.87252969,  3.87108497,
         3.88119511,  3.90349177,  3.93879062,  3.9879222 ,  4.05158914,
         4.13021342,  4.22412629,  4.33357113,  4.4588643 ,  4.6004129 ,
         4.75865553,  4.9339283 ,  5.12625652,  5.33553277,  5.56133344,
         5.80319742,  6.06066017],
       [-2.93933983, -3.18241126, -3.41042686, -3.62308121, -3.81999761,
        -4.00102088, -4.16615376, -4.31574308, -4.45036475, -4.57074083,
        -4.6774853 , -4.77096321, -4.85140192, -4.9190712 , -4.97439963,
        -5.01791222, -5.05015084, -5.07162327, -5.0829173 , -5.08455828,
        -5.07704978, -5.06066017]])}, '1': {'LEFT': np.array([[13.93933983, 13.90731374, 13.88485908, 13.87252969, 13.87108497,
        13.88119511, 13.90349177, 13.93879062, 13.9879222 , 14.05158914,
        14.13021342, 14.22412629, 14.33357113, 14.4588643 , 14.6004129 ,
        14.75865553, 14.9339283 , 15.12625652, 15.33553277, 15.56133344,
        15.80319742, 16.06066017],
       [-5.06066017, -4.81758874, -4.58957314, -4.37691879, -4.18000239,
        -3.99897912, -3.83384624, -3.68425692, -3.54963525, -3.42925917,
        -3.3225147 , -3.22903679, -3.14859808, -3.0809288 , -3.02560037,
        -2.98208778, -2.94984916, -2.92837673, -2.9170827 , -2.91544172,
        -2.92295022, -2.93933983]]), 'RIGHT': np.array([[13.93933983, 14.18241126, 14.41042686, 14.62308121, 14.81999761,
        15.00102088, 15.16615376, 15.31574308, 15.45036475, 15.57074083,
        15.6774853 , 15.77096321, 15.85140192, 15.9190712 , 15.97439963,
        16.01791222, 16.05015084, 16.07162327, 16.0829173 , 16.08455828,
        16.07704978, 16.06066017],
       [-5.06066017, -5.09268626, -5.11514092, -5.12747031, -5.12891503,
        -5.11880489, -5.09650823, -5.06120938, -5.0120778 , -4.94841086,
        -4.86978658, -4.77587371, -4.66642887, -4.5411357 , -4.3995871 ,
        -4.24134447, -4.0660717 , -3.87374348, -3.66446723, -3.43866656,
        -3.19680258, -2.93933983]])}, '2': {'LEFT': np.array([[16.06066017, 15.81758874, 15.58957314, 15.37691879, 15.18000239,
        14.99897912, 14.83384624, 14.68425692, 14.54963525, 14.42925917,
        14.3225147 , 14.22903679, 14.14859808, 14.0809288 , 14.02560037,
        13.98208778, 13.94984916, 13.92837673, 13.9170827 , 13.91544172,
        13.92295022, 13.93933983],
       [ 2.93933983,  2.90731374,  2.88485908,  2.87252969,  2.87108497,
         2.88119511,  2.90349177,  2.93879062,  2.9879222 ,  3.05158914,
         3.13021342,  3.22412629,  3.33357113,  3.4588643 ,  3.6004129 ,
         3.75865553,  3.9339283 ,  4.12625652,  4.33553277,  4.56133344,
         4.80319742,  5.06066017]]), 'RIGHT': np.array([[16.06066017, 16.09268626, 16.11514092, 16.12747031, 16.12891503,
        16.11880489, 16.09650823, 16.06120938, 16.0120778 , 15.94841086,
        15.86978658, 15.77587371, 15.66642887, 15.5411357 , 15.3995871 ,
        15.24134447, 15.0660717 , 14.87374348, 14.66446723, 14.43866656,
        14.19680258, 13.93933983],
       [ 2.93933983,  3.18241126,  3.41042686,  3.62308121,  3.81999761,
         4.00102088,  4.16615376,  4.31574308,  4.45036475,  4.57074083,
         4.6774853 ,  4.77096321,  4.85140192,  4.9190712 ,  4.97439963,
         5.01791222,  5.05015084,  5.07162327,  5.0829173 ,  5.08455828,
         5.07704978,  5.06066017]])}, '3': {'LEFT': np.array([[6.06066017, 6.09268626, 6.11514092, 6.12747031, 6.12891503,
        6.11880489, 6.09650823, 6.06120938, 6.0120778 , 5.94841086,
        5.86978658, 5.77587371, 5.66642887, 5.5411357 , 5.3995871 ,
        5.24134447, 5.0660717 , 4.87374348, 4.66446723, 4.43866656,
        4.19680258, 3.93933983],
       [5.06066017, 4.81758874, 4.58957314, 4.37691879, 4.18000239,
        3.99897912, 3.83384624, 3.68425692, 3.54963525, 3.42925917,
        3.3225147 , 3.22903679, 3.14859808, 3.0809288 , 3.02560037,
        2.98208778, 2.94984916, 2.92837673, 2.9170827 , 2.91544172,
        2.92295022, 2.93933983]]), 'RIGHT': np.array([[6.06066017, 5.81758874, 5.58957314, 5.37691879, 5.18000239,
        4.99897912, 4.83384624, 4.68425692, 4.54963525, 4.42925917,
        4.3225147 , 4.22903679, 4.14859808, 4.0809288 , 4.02560037,
        3.98208778, 3.94984916, 3.92837673, 3.9170827 , 3.91544172,
        3.92295022, 3.93933983],
       [5.06066017, 5.09268626, 5.11514092, 5.12747031, 5.12891503,
        5.11880489, 5.09650823, 5.06120938, 5.0120778 , 4.94841086,
        4.86978658, 4.77587371, 4.66642887, 4.5411357 , 4.3995871 ,
        4.24134447, 4.0660717 , 3.87374348, 3.66446723, 3.43866656,
        3.19680258, 2.93933983]])}}

# Start-End points of each gate pair
SE_point = {'0': {'S': np.array([[ 3.93933983],
       [-2.93933983]]), 'E': np.array([[ 6.06066017],
       [-5.06066017]])}, '1': {'S': np.array([[13.93933983],
       [-5.06066017]]), 'E': np.array([[16.06066017],
       [-2.93933983]])}, '2': {'S': np.array([[16.06066017],
       [ 2.93933983]]), 'E':np.array([[13.93933983],
       [ 5.06066017]])}, '3': {'S': np.array([[6.06066017],
       [5.06066017]]), 'E': np.array([[3.93933983],
       [2.93933983]])}}

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

