#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import *

april_map_pose = [
[{'x': 5.0, 'y': -4.0, 'z': 0.75},
  {'qx': 0.0, 'qy': 0.0, 'qz': -0.3827, 'qw': 0.9239}],
 [{'x': 15.0, 'y': -4.0, 'z': 0.75},
  {'qx': 0.0, 'qy': 0.0, 'qz': 0.3827, 'qw': 0.9239}],
 [{'x': 15.0, 'y': 4.0, 'z': 0.75},
  {'qx': 0.0, 'qy': 0.0, 'qz': 0.3827, 'qw': 0.9239}],
 [{'x': 5.0, 'y': 4.0, 'z': 0.75},
  {'qx': 0.0, 'qy': 0.0, 'qz': -0.3827, 'qw': 0.9239}]
]
  
class comb_localization:
    def __init__(self):
        rospy.init_node('tag_pose_printer')
        
        #subscribe poses from april&orb3&mavros, publish combined pose
        self.april_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.april_callback)
        #self.orb_sub = rospy.Subscriber("/orb_slam3/camera_pose",PoseStamped, self.pose_callback)
        self.pub = rospy.Publisher("/comb_pose", PoseStamped, queue_size=1)
        
        self.april_pose = None
        self.master_id = None
    
    def april_callback(self, msg):
        for i, det in enumerate(msg.detections):
            self.master_id = min(det.id)
            print(f"Tag {self.master_id} position:")
            cam_pose = det.pose.pose.pose
            print(f"  x: {cam_pose.position.x}, y: {cam_pose.position.y}, z: {cam_pose.position.z}")
            print(f"  x: {cam_pose.orientation.x}, y: {cam_pose.orientation.y}, z: {cam_pose.orientation.z}, w: {cam_pose.orientation.w}")
            print()
            
        q_wc = [cam_pose.orientation.x, cam_pose.orientation.y, cam_pose.orientation.z, cam_pose.orientation.w]
        t_wc = [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]

        T_wc = quaternion_matrix(q_wc)
        T_wc[0:3, 3] = t_wc

        # Define T_bc (body → camera), so we will invert it to get T_cb
        R_bc = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, -1, 0]
        ])
        
        t_bc = np.array([0.05, 0, 0])
        T_bc = np.eye(4)
        T_bc[0:3, 0:3] = R_bc
        T_bc[0:3, 3] = t_bc

        #T_cb = np.linalg.inv(T_bc)
        
        T_bw = T_bc @ np.linalg.inv(T_wc)

        # Extract new position and orientation
        new_position = T_bw[0:3, 3]
        new_quat = quaternion_from_matrix(T_bw)
        
        # Return transformed pose
        self.april_pose = Pose()
        self.april_pose.position.x = new_position[0]
        self.april_pose.position.y = new_position[1]
        self.april_pose.position.z = new_position[2]
        self.april_pose.orientation.x = new_quat[0]
        self.april_pose.orientation.y = new_quat[1]
        self.april_pose.orientation.z = new_quat[2]
        self.april_pose.orientation.w = new_quat[3]
        
        print(f"  x: {self.april_pose.position.x}, y: {self.april_pose.position.y}, z: {self.april_pose.position.z}")
        print(f"  x: {self.april_pose.orientation.x}, y: {self.april_pose.orientation.y}, z: {self.april_pose.orientation.z}, z: {self.april_pose.orientation.w}")
        print()
    
    


if __name__ == "__main__":
    cl = comb_localization()
    rospy.spin()

