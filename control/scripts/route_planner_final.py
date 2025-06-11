#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

# Optimized curve points [x, y] dictionary
curves = {0: {'LEFT': np.array([[3.1       , 3.21666667, 3.33333333, 3.45      , 3.56666667,
          3.68333333, 3.8       , 3.91666667, 4.03333333, 4.15      ,
          4.26666667, 4.38333333, 4.5       , 4.5       , 4.61111111,
          4.72222222, 4.83333333, 4.94444444, 5.05555556, 5.16666667,
          5.27777778, 5.38888889, 5.5       ],
         [0.        , 0.08333333, 0.16666667, 0.25      , 0.33333333,
          0.41666667, 0.5       , 0.58333333, 0.66666667, 0.75      ,
          0.83333333, 0.91666667, 1.        , 1.        , 1.        ,
          1.        , 1.        , 1.        , 1.        , 1.        ,
          1.        , 1.        , 1.        ]]),
  'RIGHT': np.array([[ 3.1       ,  3.21666667,  3.33333333,  3.45      ,  3.56666667,
           3.68333333,  3.8       ,  3.91666667,  4.03333333,  4.15      ,
           4.26666667,  4.38333333,  4.5       ,  4.5       ,  4.61111111,
           4.72222222,  4.83333333,  4.94444444,  5.05555556,  5.16666667,
           5.27777778,  5.38888889,  5.5       ],
         [ 0.        , -0.08333333, -0.16666667, -0.25      , -0.33333333,
          -0.41666667, -0.5       , -0.58333333, -0.66666667, -0.75      ,
          -0.83333333, -0.91666667, -1.        , -1.        , -1.        ,
          -1.        , -1.        , -1.        , -1.        , -1.        ,
          -1.        , -1.        , -1.        ]])},
 1: {'LEFT': np.array([[7.95      , 7.81949788, 7.68899577, 7.55849365, 7.42799153,
          7.29748942, 7.1669873 , 7.03648518, 6.90598306, 6.77548095,
          6.64497883, 6.51447671, 6.3839746 , 6.3839746 , 6.32841904,
          6.27286349, 6.21730793, 6.16175237, 6.10619682, 6.05064126,
          5.99508571, 5.93953015, 5.8839746 ],
         [1.81865173, 1.87802136, 1.93739099, 1.99676062, 2.05613025,
          2.11549989, 2.17486952, 2.23423915, 2.29360878, 2.35297841,
          2.41234804, 2.47171767, 2.5310873 , 2.5310873 , 2.62731234,
          2.72353739, 2.81976243, 2.91598748, 3.01221252, 3.10843757,
          3.20466261, 3.30088766, 3.3971127 ]]),
  'RIGHT': np.array([[7.95      , 7.96383545, 7.9776709 , 7.99150635, 8.0053418 ,
          8.01917725, 8.0330127 , 8.04684815, 8.0606836 , 8.07451905,
          8.0883545 , 8.10218995, 8.1160254 , 8.1160254 , 8.06046985,
          8.00491429, 7.94935874, 7.89380318, 7.83824763, 7.78269207,
          7.72713651, 7.67158096, 7.6160254 ],
         [1.81865173, 1.9613547 , 2.10405766, 2.24676062, 2.38946359,
          2.53216655, 2.67486952, 2.81757248, 2.96027544, 3.10297841,
          3.24568137, 3.38838433, 3.5310873 , 3.5310873 , 3.62731234,
          3.72353739, 3.81976243, 3.91598748, 4.01221252, 4.10843757,
          4.20466261, 4.30088766, 4.3971127 ]])},
 2: {'LEFT': np.array([[3.95      , 3.96383545, 3.9776709 , 3.99150635, 4.0053418 ,
          4.01917725, 4.0330127 , 4.04684815, 4.0606836 , 4.07451905,
          4.0883545 , 4.10218995, 4.1160254 , 4.1160254 , 4.06046985,
          4.00491429, 3.94935874, 3.89380318, 3.83824763, 3.78269207,
          3.72713651, 3.67158096, 3.6160254 ],
         [5.10954827, 4.9668453 , 4.82414234, 4.68143938, 4.53873641,
          4.39603345, 4.25333048, 4.11062752, 3.96792456, 3.82522159,
          3.68251863, 3.53981567, 3.3971127 , 3.3971127 , 3.30088766,
          3.20466261, 3.10843757, 3.01221252, 2.91598748, 2.81976243,
          2.72353739, 2.62731234, 2.5310873 ]]),
  'RIGHT': np.array([[3.95      , 3.81949788, 3.68899577, 3.55849365, 3.42799153,
          3.29748942, 3.1669873 , 3.03648518, 2.90598306, 2.77548095,
          2.64497883, 2.51447671, 2.3839746 , 2.3839746 , 2.32841904,
          2.27286349, 2.21730793, 2.16175237, 2.10619682, 2.05064126,
          1.99508571, 1.93953015, 1.8839746 ],
         [5.10954827, 5.05017864, 4.99080901, 4.93143938, 4.87206975,
          4.81270011, 4.75333048, 4.69396085, 4.63459122, 4.57522159,
          4.51585196, 4.45648233, 4.3971127 , 4.3971127 , 4.30088766,
          4.20466261, 4.10843757, 4.01221252, 3.91598748, 3.81976243,
          3.72353739, 3.62731234, 3.5310873 ]])}}


# Start-End points of each gate pair
SE_point = {0: {'S': np.array([3.1, 0. ]),
  'E_LEFT': np.array([5.5, 1. ]),
  'E_RIGHT': np.array([ 5.5, -1. ])},
 1: {'S': np.array([7.95      , 1.81865173]),
  'E_LEFT': np.array([5.8839746, 3.3971127]),
  'E_RIGHT': np.array([7.6160254, 4.3971127])},
 2: {'S': np.array([3.95      , 5.10954827]),
  'E_LEFT': np.array([3.6160254, 2.5310873]),
  'E_RIGHT': np.array([1.8839746, 3.5310873])}}

central_point = {0:np.hstack([5, 0]),
          1:np.hstack([7.0, 3.4641]),
          2:np.hstack([3.0, 3.4641]),
          }

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
    def __init__(self, curves, SE_point, initial_line, lines, central_point):
        rospy.init_node("route_planner")

        self.curves = curves
        self.SE_point = SE_point
        self.initial_line = initial_line
        self.lines = lines
        self.central_point = central_point
        
        self.yaw_S = [0, 2*np.pi/3, -2*np.pi/3]
        #self.yaw_E = [[
        #    np.arctan2(lines[i]["LEFT"][-1][1]-lines[i]["LEFT"][0][1], lines[i]["LEFT"][-1][0]-lines[i]["LEFT"][0][0]),\
        #    np.arctan2(lines[i]["RIGHT"][-1][1]-lines[i]["RIGHT"][0][1], lines[i]["RIGHT"][-1][0]-lines[i]["RIGHT"][0][0])]
        #    for i in range(len(SE_point))]
    
        self.yaw_initial = np.arctan2(SE_point[0]['S'][1], SE_point[0]['S'][0])
        self.Z_tag = rospy.get_param("~Z_tag", 0.75)  # alwasys keep z as the height of tags 
        
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
        self.startpass_pub = rospy.Publisher("/start_passed", Bool, queue_size = 1)
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
        yaw_x, yaw_y = self.central_point[name+1][0], self.central_point[name+1][1]
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
        if type == "E_LEFT" or "E_RIGHT":
            yaw_x, yaw_y = self.central_point[name+1][0], self.central_point[name+1][1]
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
            s_pass = Bool()
            s_pass.data = True
            self.startpass_pub.publish(s_pass)
        elif self.mode == "E":
            # Passed the gate, follow the line for the next gate pair
            self.idx = 0
            self.name = (self.name + 1) % 3
            self.mode = "line"
        elif self.mode == "curve":
            # Followed the curve, change to the end point state
            if self.idx < curves[self.name]["LEFT"].shape[1]-1:
                self.idx +=1
            else:
                self.idx = 0
                self.mode = "E"
                s_pass = Bool()
                s_pass.data = False
                self.startpass_pub.publish(s_pass)
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
