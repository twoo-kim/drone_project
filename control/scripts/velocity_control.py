#!/usr/bin/env python3
import numpy as np
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

current_pose = PoseStamped()
target_pose = PoseStamped()
current_yaw = 0.0

def pose_callback(msg):
    global current_pose
    current_pose = msg

def imu_callback(msg):
    global current_yaw
    q = msg.orientation
    quat = [q.x, q.y, q.z, q.w]
    _, _, yaw = euler_from_quaternion(quat)
    current_yaw = yaw
    #rospy.loginfo("[offboard] current yaw : z=%.2f", current_yaw)

def waypoint_callback(msg):
    global target_pose
    target_pose = msg

def get_yaw_from_orientation(orientation):
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def yaw_error(target_yaw, current_yaw):
    # Normalize to [-pi, pi]
    error = target_yaw - current_yaw
    return math.atan2(math.sin(error), math.cos(error))

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

if __name__ == "__main__":
    rospy.init_node("vel_control_node_py")
    
    # mavros real value := /mavros/local_position/pose
    rospy.Subscriber("/ekf_pose", PoseStamped, callback=pose_callback)
    rospy.Subscriber("/lookahead_waypoint", PoseStamped, callback=waypoint_callback)
    #rospy.Subscriber("/mavros/imu/data", Imu, callback=imu_callback)
    
    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rate = rospy.Rate(20)
    
    Z_tag = rospy.get_param("~Z_tag", 0.70)
    cmd_velocity = TwistStamped()

    rospy.loginfo("sending hovering topics to arm")
    rate = rospy.Rate(35)
    
    while current_pose.pose.position.z < Z_tag - 0.08:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = Z_tag
        pos_pub.publish(pose)
        rospy.loginfo("[offboard] current height : z=%.2f", current_pose.pose.position.z)
        rate.sleep()
    
    rospy.loginfo("ready to go")
    rospy.loginfo("take off")
    
    # applying PI control for stable adjustment of yaw & z
    z_error_integral = 0.0
    yaw_error_integral = 0.0
    previous_time = rospy.Time.now().to_sec()

    K_P = 0.3
    K_I = 0.1
    K_YAW_P = 1.0
    K_YAW_I = 0.2

    MAX_VEL_XY = 1.0   # m/s
    MAX_VEL_Z  = 0.5   # m/s
    MAX_Z_INT = 1.0    # m/s
    MAX_YAW_RATE = 0.7 # rad/s
    MAX_YAW_INT = 1.0 # for I in yaw control

    while not rospy.is_shutdown():
        # --- position error ---
        current_time = rospy.Time.now().to_sec()
        dt = current_time - previous_time
        previous_time = current_time

        error_x = target_pose.pose.position.x - current_pose.pose.position.x
        error_y = target_pose.pose.position.y - current_pose.pose.position.y
        error_z = target_pose.pose.position.z - current_pose.pose.position.z

        vx = K_P * error_x
        vy = K_P * error_y
        vz = K_P * error_z

        # --- yaw control (PI) ---
        if abs(error_z) > 0.02:
            z_error_integral += error_z * dt
        
        z_error_integral = clamp(z_error_integral, -MAX_Z_INT, MAX_Z_INT)
        wz = vz + K_I * z_error_integral

        # --- clamp velocity ---
        cmd_velocity.twist.linear.x = clamp(vx, -MAX_VEL_XY, MAX_VEL_XY)
        cmd_velocity.twist.linear.y = clamp(vy, -MAX_VEL_XY, MAX_VEL_XY)
        cmd_velocity.twist.linear.z = clamp(wz, -MAX_VEL_Z, MAX_VEL_Z)

        # --- yaw control ---
        current_yaw = get_yaw_from_orientation(current_pose.pose.orientation)
        target_yaw = get_yaw_from_orientation(target_pose.pose.orientation)
        error_yaw = yaw_error(target_yaw, current_yaw)
        
        if abs(error_yaw) > 0.02:
            yaw_error_integral += error_yaw * dt
            
        yaw_error_integral = clamp(yaw_error_integral, -MAX_YAW_INT, MAX_YAW_INT)
        rospy.loginfo("[offboard] current yaw error : %.2f, yaw : %.2f", error_yaw*180/np.pi, current_yaw*180/np.pi)

        #wz = K_YAW_P * error_yaw
        wz = K_YAW_P * error_yaw + K_YAW_I * yaw_error_integral
        
        cmd_velocity.twist.angular.z = clamp(wz, -MAX_YAW_RATE, MAX_YAW_RATE)

        local_vel_pub.publish(cmd_velocity)
        rate.sleep()