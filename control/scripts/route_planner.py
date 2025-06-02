#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {
    0: {'LEFT': np.array([[4.46966991,  4.59222164,  4.709656  ,  4.82176985,  4.92816864, 5.02833142,  5.12171147,  5.20774562,  5.28584353,  5.35552972, 5.41645516,
                           5.46840258,  5.51124617,  5.54504948,  5.57007953, 5.58665941,  5.59515014,  5.59594837,  5.58950212,  5.57623101, 5.55644598,  5.53033009],
                          [-3.46966991, -3.44062077, -3.41861453, -3.40381648, -3.39651703, -3.39700449, -3.40558354, -3.4224959 , -3.44797832, -3.48226244,-3.52549671,
                           -3.57769041, -3.6387141 , -3.7083841 , -3.78642977, -3.87243317, -3.96591686, -4.06642086, -4.17351714, -4.28676435, -4.40577407, -4.53033009]]),
        'RIGHT': np.array([[4.46966991,  4.44062077,  4.41861453,  4.40381648,  4.39651703, 4.39700449,  4.40558354,  4.4224959 ,  4.44797832,  4.48226244, 4.52549671,
                            4.57769041,  4.6387141 ,  4.7083841 ,  4.78642977, 4.87243317,  4.96591686,  5.06642086,  5.17351714,  5.28676435, 5.40577407,  5.53033009],
                           [-3.46966991, -3.59222164, -3.709656  , -3.82176985, -3.92816864, -4.02833142, -4.12171147, -4.20774562, -4.28584353, -4.35552972, -4.41645516,
                            -4.46840258, -4.51124617, -4.54504948, -4.57007953, -4.58665941, -4.59515014, -4.59594837, -4.58950212, -4.57623101, -4.55644598, -4.53033009]])
    },
    1: {'LEFT': np.array([[14.46966991, 14.44062077, 14.41861453, 14.40381648, 14.39651703, 14.39700449, 14.40558354, 14.4224959 , 14.44797832, 14.48226244, 14.52549671,
                            14.57769041, 14.6387141 , 14.7083841 , 14.78642977, 14.87243317, 14.96591686, 15.06642086, 15.17351714, 15.28676435,15.40577407, 15.53033009],
                          [-4.53033009, -4.40777836, -4.290344  , -4.17823015, -4.07183136, -3.97166858, -3.87828853, -3.79225438, -3.71415647, -3.64447028, -3.58354484,
                           -3.53159742, -3.48875383, -3.45495052, -3.42992047,-3.41334059, -3.40484986, -3.40405163, -3.41049788, -3.42376899, -3.44355402, -3.46966991]]),
        'RIGHT': np.array([[14.46966991, 14.59222164, 14.709656  , 14.82176985, 14.92816864, 15.02833142, 15.12171147, 15.20774562, 15.28584353, 15.35552972, 15.41645516,
                            15.46840258, 15.51124617, 15.54504948, 15.57007953, 15.58665941, 15.59515014, 15.59594837, 15.58950212, 15.57623101, 15.55644598, 15.53033009],
                           [-4.53033009, -4.55937923, -4.58138547, -4.59618352, -4.60348297, -4.60299551, -4.59441646, -4.5775041 , -4.55202168, -4.51773756, -4.47450329,
                            -4.42230959, -4.3612859 , -4.2916159 , -4.21357023, -4.12756683, -4.03408314, -3.93357914, -3.82648286, -3.71323565, -3.59422593, -3.46966991]])
    },
    2: {'LEFT': np.array([[15.53033009, 15.40777836, 15.290344  , 15.17823015, 15.07183136, 14.97166858, 14.87828853, 14.79225438, 14.71415647, 14.64447028, 14.58354484,
                            14.53159742, 14.48875383, 14.45495052, 14.42992047, 14.41334059, 14.40484986, 14.40405163, 14.41049788, 14.42376899, 14.44355402, 14.46966991],
                          [3.46966991,  3.44062077,  3.41861453,  3.40381648,  3.39651703, 3.39700449,  3.40558354,  3.4224959 ,  3.44797832,  3.48226244, 3.52549671,
                           3.57769041,  3.6387141 ,  3.7083841 ,  3.78642977, 3.87243317,  3.96591686,  4.06642086,  4.17351714,  4.28676435, 4.40577407,  4.53033009]]),
        'RIGHT': np.array([[15.53033009, 15.55937923, 15.58138547, 15.59618352, 15.60348297, 15.60299551, 15.59441646, 15.5775041 , 15.55202168, 15.51773756, 15.47450329,
                            15.42230959, 15.3612859 , 15.2916159 , 15.21357023, 15.12756683, 15.03408314, 14.93357914, 14.82648286, 14.71323565,14.59422593, 14.46966991],
                           [3.46966991,  3.59222164,  3.709656  ,  3.82176985,  3.92816864, 4.02833142,  4.12171147,  4.20774562,  4.28584353,  4.35552972, 4.41645516,
                            4.46840258,  4.51124617,  4.54504948,  4.57007953, 4.58665941,  4.59515014,  4.59594837,  4.58950212,  4.57623101, 4.55644598,  4.53033009]])
    },
    3: {'LEFT': np.array([[5.53033009, 5.55937923, 5.58138547, 5.59618352, 5.60348297, 5.60299551, 5.59441646, 5.5775041 , 5.55202168, 5.51773756, 5.47450329,
                           5.42230959, 5.3612859 , 5.2916159 , 5.21357023, 5.12756683, 5.03408314, 4.93357914, 4.82648286, 4.71323565, 4.59422593, 4.46966991],
                          [4.53033009, 4.40777836, 4.290344  , 4.17823015, 4.07183136, 3.97166858, 3.87828853, 3.79225438, 3.71415647, 3.64447028, 3.58354484,
                           3.53159742, 3.48875383, 3.45495052, 3.42992047, 3.41334059, 3.40484986, 3.40405163, 3.41049788, 3.42376899, 3.44355402, 3.46966991]]),
        'RIGHT': np.array([[5.53033009, 5.40777836, 5.290344  , 5.17823015, 5.07183136, 4.97166858, 4.87828853, 4.79225438, 4.71415647, 4.64447028, 4.58354484,
                            4.53159742,4.48875383, 4.45495052, 4.42992047, 4.41334059, 4.40484986, 4.40405163, 4.41049788, 4.42376899, 4.44355402, 4.46966991],
                           [4.53033009, 4.55937923, 4.58138547, 4.59618352, 4.60348297, 4.60299551, 4.59441646, 4.5775041 , 4.55202168, 4.51773756, 4.47450329,
                            4.42230959,4.3612859 , 4.2916159 , 4.21357023,4.12756683, 4.03408314, 3.93357914, 3.82648286, 3.71323565,3.59422593, 3.46966991]])
    }
}

# Start-End points of each gate pair
SE_point = {
    0: {'S': np.array([4.46966991, -3.46966991]),
        'E': np.array([5.53033009, -4.53033009])},
    1: {'S': np.array([14.46966991, -4.53033009]),
        'E': np.array([15.53033009, -3.46966991])},
    2: {'S': np.array([15.53033009, 3.46966991]),
        'E': np.array([14.46966991, 4.53033009])}, 
    3: {'S': np.array([5.53033009, 4.53033009]),
        'E': np.array([4.46966991, 3.46966991])}
}

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

