#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_quaternion

curves = {'0': {'LEFT': np.array([[ 4.46966991,  4.59222164,  4.709656  ,  4.82176985,  4.92816864,
         5.02833142,  5.12171147,  5.20774562,  5.28584353,  5.35552972,
         5.41645516,  5.46840258,  5.51124617,  5.54504948,  5.57007953,
         5.58665941,  5.59515014,  5.59594837,  5.58950212,  5.57623101,
         5.55644598,  5.53033009],
       [-3.46966991, -3.44062077, -3.41861453, -3.40381648, -3.39651703,
        -3.39700449, -3.40558354, -3.4224959 , -3.44797832, -3.48226244,
        -3.52549671, -3.57769041, -3.6387141 , -3.7083841 , -3.78642977,
        -3.87243317, -3.96591686, -4.06642086, -4.17351714, -4.28676435,
        -4.40577407, -4.53033009]]), 'RIGHT': np.array([[ 4.46966991,  4.44062077,  4.41861453,  4.40381648,  4.39651703,
         4.39700449,  4.40558354,  4.4224959 ,  4.44797832,  4.48226244,
         4.52549671,  4.57769041,  4.6387141 ,  4.7083841 ,  4.78642977,
         4.87243317,  4.96591686,  5.06642086,  5.17351714,  5.28676435,
         5.40577407,  5.53033009],
       [-3.46966991, -3.59222164, -3.709656  , -3.82176985, -3.92816864,
        -4.02833142, -4.12171147, -4.20774562, -4.28584353, -4.35552972,
        -4.41645516, -4.46840258, -4.51124617, -4.54504948, -4.57007953,
        -4.58665941, -4.59515014, -4.59594837, -4.58950212, -4.57623101,
        -4.55644598, -4.53033009]])}, '1': {'LEFT': np.array([[14.46966991, 14.44062077, 14.41861453, 14.40381648, 14.39651703,
        14.39700449, 14.40558354, 14.4224959 , 14.44797832, 14.48226244,
        14.52549671, 14.57769041, 14.6387141 , 14.7083841 , 14.78642977,
        14.87243317, 14.96591686, 15.06642086, 15.17351714, 15.28676435,
        15.40577407, 15.53033009],
       [-4.53033009, -4.40777836, -4.290344  , -4.17823015, -4.07183136,
        -3.97166858, -3.87828853, -3.79225438, -3.71415647, -3.64447028,
        -3.58354484, -3.53159742, -3.48875383, -3.45495052, -3.42992047,
        -3.41334059, -3.40484986, -3.40405163, -3.41049788, -3.42376899,
        -3.44355402, -3.46966991]]), 'RIGHT': np.array([[14.46966991, 14.59222164, 14.709656  , 14.82176985, 14.92816864,
        15.02833142, 15.12171147, 15.20774562, 15.28584353, 15.35552972,
        15.41645516, 15.46840258, 15.51124617, 15.54504948, 15.57007953,
        15.58665941, 15.59515014, 15.59594837, 15.58950212, 15.57623101,
        15.55644598, 15.53033009],
       [-4.53033009, -4.55937923, -4.58138547, -4.59618352, -4.60348297,
        -4.60299551, -4.59441646, -4.5775041 , -4.55202168, -4.51773756,
        -4.47450329, -4.42230959, -4.3612859 , -4.2916159 , -4.21357023,
        -4.12756683, -4.03408314, -3.93357914, -3.82648286, -3.71323565,
        -3.59422593, -3.46966991]])}, '2': {'LEFT': np.array([[15.53033009, 15.40777836, 15.290344  , 15.17823015, 15.07183136,
        14.97166858, 14.87828853, 14.79225438, 14.71415647, 14.64447028,
        14.58354484, 14.53159742, 14.48875383, 14.45495052, 14.42992047,
        14.41334059, 14.40484986, 14.40405163, 14.41049788, 14.42376899,
        14.44355402, 14.46966991],
       [ 3.46966991,  3.44062077,  3.41861453,  3.40381648,  3.39651703,
         3.39700449,  3.40558354,  3.4224959 ,  3.44797832,  3.48226244,
         3.52549671,  3.57769041,  3.6387141 ,  3.7083841 ,  3.78642977,
         3.87243317,  3.96591686,  4.06642086,  4.17351714,  4.28676435,
         4.40577407,  4.53033009]]), 'RIGHT': np.array([[15.53033009, 15.55937923, 15.58138547, 15.59618352, 15.60348297,
        15.60299551, 15.59441646, 15.5775041 , 15.55202168, 15.51773756,
        15.47450329, 15.42230959, 15.3612859 , 15.2916159 , 15.21357023,
        15.12756683, 15.03408314, 14.93357914, 14.82648286, 14.71323565,
        14.59422593, 14.46966991],
       [ 3.46966991,  3.59222164,  3.709656  ,  3.82176985,  3.92816864,
         4.02833142,  4.12171147,  4.20774562,  4.28584353,  4.35552972,
         4.41645516,  4.46840258,  4.51124617,  4.54504948,  4.57007953,
         4.58665941,  4.59515014,  4.59594837,  4.58950212,  4.57623101,
         4.55644598,  4.53033009]])}, '3': {'LEFT': np.array([[5.53033009, 5.55937923, 5.58138547, 5.59618352, 5.60348297,
        5.60299551, 5.59441646, 5.5775041 , 5.55202168, 5.51773756,
        5.47450329, 5.42230959, 5.3612859 , 5.2916159 , 5.21357023,
        5.12756683, 5.03408314, 4.93357914, 4.82648286, 4.71323565,
        4.59422593, 4.46966991],
       [4.53033009, 4.40777836, 4.290344  , 4.17823015, 4.07183136,
        3.97166858, 3.87828853, 3.79225438, 3.71415647, 3.64447028,
        3.58354484, 3.53159742, 3.48875383, 3.45495052, 3.42992047,
        3.41334059, 3.40484986, 3.40405163, 3.41049788, 3.42376899,
        3.44355402, 3.46966991]]), 'RIGHT': np.array([[5.53033009, 5.40777836, 5.290344  , 5.17823015, 5.07183136,
        4.97166858, 4.87828853, 4.79225438, 4.71415647, 4.64447028,
        4.58354484, 4.53159742, 4.48875383, 4.45495052, 4.42992047,
        4.41334059, 4.40484986, 4.40405163, 4.41049788, 4.42376899,
        4.44355402, 4.46966991],
       [4.53033009, 4.55937923, 4.58138547, 4.59618352, 4.60348297,
        4.60299551, 4.59441646, 4.5775041 , 4.55202168, 4.51773756,
        4.47450329, 4.42230959, 4.3612859 , 4.2916159 , 4.21357023,
        4.12756683, 4.03408314, 3.93357914, 3.82648286, 3.71323565,
        3.59422593, 3.46966991]])}}

SE_point = {
    0: {'S': np.array([[ 4.46966991],
       [-3.46966991]]), 'E': np.array([[ 5.53033009],
       [-4.53033009]])}, 
    1: {'S': np.array([[14.46966991],
       [-4.53033009]]), 'E': np.array([[15.53033009],
       [-3.46966991]])}, 
    2: {'S': np.array([[15.53033009],
       [ 3.46966991]]), 'E': np.array([[14.46966991],
       [ 4.53033009]])}, 
    3: {'S': np.array([[5.53033009],
       [4.53033009]]), 'E': np.array([[4.46966991],
       [3.46966991]])}}

initial_line = np.hstack([np.linspace(0, SE_point[0]['S'][0], 10), np.linspace(0, SE_point[0]['S'][1], 10)])

lines = {
    0 : np.hstack([np.linspace(SE_point[3]['E'][0], SE_point[0]['S'][0], 20), np.linspace(SE_point[3]['E'][1], SE_point[0]['S'][1], 20)]),
    1 : np.hstack([np.linspace(SE_point[0]['E'][0], SE_point[1]['S'][0], 20), np.linspace(SE_point[0]['E'][1], SE_point[1]['S'][1], 20)]),
    2 : np.hstack([np.linspace(SE_point[1]['E'][0], SE_point[2]['S'][0], 20), np.linspace(SE_point[1]['E'][1], SE_point[2]['S'][1], 20)]),
    3 : np.hstack([np.linspace(SE_point[2]['E'][0], SE_point[3]['S'][0], 20), np.linspace(SE_point[2]['E'][1], SE_point[3]['S'][1], 20)]),
}

class RoutePlanner:
    def __init__(self, curves, SE_point, initial_line, lines):
        rospy.init_node("route_planner")

        self.curves = curves
        self.SE_point = SE_point
        self.initial_line = initial_line
        self.lines = lines

        # Parameters
        self.yaw_S = [1*np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4] # add np.pi/2 for next S's yaw
        self.yaw_E = [2*np.pi/4, 4*np.pi/4, 6*np.pi/4, 8*np.pi/4] # add np.pi/2 for next E's yaw
        self.yaw_initial = np.arctan2(self.SE_point[0]['S'][0], -self.SE_point[0]['S'][1])[0]

        self.Z_tag = rospy.get_param("~Z_tag", 0.70)  # alwasys keep z as the height of tags 
        
        self.ths_S = rospy.get_param("~ths_S", 0.05)
        self.ths_curve = rospy.get_param("~ths_curve", 0.07)
        self.ths_line = rospy.get_param("~ths_curve", 0.1)

        self.ths_yaw = rospy.get_param("~ths_yaw", 0.09)

        self.current_pose = None
        self.prev_pose = None
        self.direction = "None"

        # ROS interfaces
        self.pose_sub = rospy.Subscriber("/pose", PoseStamped, self.pose_callback)   # subsribe the EKF solution
        self.direction_sub = rospy.Subscriber("/goal", String, self.pose_callback)
        self.waypoint_pub = rospy.Publisher("/lookahead_waypoint", PoseStamped, queue_size=1)
        
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

    def yaw_toward_center(self, x, y):
        dx = self.center.x - x
        dy = self.center.y - y
        return math.atan2(dy, dx)

    def yaw_error(self, orien_cur, orien_tar):
        q_cur = [orien_cur.x, orien_cur.y, orien_cur.z, orien_cur.w]
        q_tar = [orien_tar.x, orien_tar.y, orien_tar.z, orien_tar.w]

        _, _, cur_yaw = euler_from_quaternion(q_cur)
        _, _, tar_yaw = euler_from_quaternion(q_tar)

        # [-pi, pi] normalization
        error = np.arctan2(np.sin(tar_yaw - cur_yaw), np.cos(tar_yaw - cur_yaw))
        return np.abs(error)
    
    def generate_initial(self, idx):
        yaw = self.yaw_initial
        x = self.initial_line[idx][0]
        y = self.initial_line[idx][1]
        z = self.Z_tag
        quat = quaternion_from_euler(0, 0, yaw)

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp

    def generate_curve(self, name, idx, direction):
        yaw = self.yaw_S[name]
        x = self.curves[name][direction][0][idx]
        y = self.curves[name][direction][1][idx]
        z = self.Z_tag
        quat = quaternion_from_euler(0, 0, yaw)

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp

    def generate_line(self, name, idx):
        yaw = self.yaw_E[name]
        x = self.lines[name][idx][0]
        y = self.lines[name][idx][1]
        z = self.Z_tag
        quat = quaternion_from_euler(0, 0, yaw)

        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "odom"
        
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        wp.pose.orientation = Quaternion(*quat)
        return wp
    
    def generate_SE(self, type,  name):
        if type == "S":
            yaw = self.yaw_S[name]
        if type == "E":
            yaw = self.yaw_E[name]
        x = self.SE_point[name][type][0][0]
        y = self.SE_point[name][type][1][0]
        z = self.Z_tag
        quat = quaternion_from_euler(0, 0, yaw)

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
        self.direction = "LEFT"
        self.name = 0

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

                dist = self.compute_distance(self.current_pose.pose.position, self.current_waypoint.pose.position)
                yaw_err = self.yaw_error(self.current_pose.pose.orientation, self.current_waypoint.pose.orientation)
                
                rospy.loginfo_throttle(1.0, "[Planner] Sending pose : x=%.2f y=%.2f z=%.2f", self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z)
                rospy.loginfo_throttle(1.0,"[Planner] Sending waypoint : x=%.2f y=%.2f z=%.2f", self.current_waypoint.pose.position.x, self.current_waypoint.pose.position.y, self.current_waypoint.pose.position.z)
                rospy.loginfo_throttle(1.0,"[Planner] current distance : %.2f, yaw err: %.2f", dist, yaw_err)
                
                if self.mode == 'initialize' or 'line':
                    dist_thr = self.ths_line
                if self.mode == "curve":
                    dist_thr == self.ths_curve
                if self.mode == "S" or "E":
                    dist_thr == self.ths_S
                
                if dist < dist_thr and yaw_err < self.ths_yaw:
                    rospy.loginfo_throttle(1.0, "[Planner] Reached waypoint")
                    # waiting for the QR code answer on S
                    if self.mode == "S" and self.direction == "None":
                        rospy.loginfo_throttle(1.0, "[Planner] Waiting for the direction ...")
                    else:
                        self.advance_route()
                        self.current_waypoint = self.select_waypoint(self.name, self.idx)
                
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
        else :
            rospy.loginfo("invalid mode")
            return PoseStamped()

    def advance_route(self):
        if self.mode == "line":
            if self.idx < len(self.initial_line)-1:
                self.idx +=1
            else :
                self.idx = 0
                self.mode = "S"
        elif self.mode == "S":
            self.idx = 0
            self.name = self.name
            self.mode = "curve"
        elif self.mode == "E":
            self.idx = 0
            self.name = (self.name + 1) % 4
            self.mode = "line"
            self.direction = "None"
        elif self.mode == "curve":
            if self.idx < curves[self.name]["L"].shape[1]-1:
                self.idx +=1
            else :
                self.idx = 0
                self.mode = "E"
        elif self.mode == "initialize":
            if self.idx < len(self.initial_line)-1:
                self.idx +=1
            else :
                self.idx = 0
                self.mode = "S"
        else :
            rospy.loginfo("invalid mode")
            return PoseStamped()

if __name__ == '__main__':
    RoutePlanner(curves, SE_point, initial_line, lines)
    rospy.spin()

