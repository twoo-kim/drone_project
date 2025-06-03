#!/usr/bin/env python3
import numpy as np
import rospy
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# MAVROS state
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

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

class OffboardController:
    def __init__(self):
        # State
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.current_yaw = 0.0
        self.Z_tag = rospy.get_param("~Z_tag", 0.70)
        self.cmd_velocity = TwistStamped()
        self.is_takeoff = False

        self.z_error_integral = 0.0
        self.yaw_error_integral = 0.0
        self.previous_time = rospy.Time.now().to_sec()

        # Publisher subscriber
        rospy.Subscriber("/pose_topic", PoseStamped, callback=self.pose_callback)
        rospy.Subscriber("/lookahead_waypoint", PoseStamped, callback=self.waypoint_callback)
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    def pose_callback(self, msg):
        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.current_pose = msg
        self.current_yaw = yaw

    def waypoint_callback(self, msg):
        self.target_pose = msg

    def takeOff(self):
        rate = rospy.Rate(35)
        while (self.current_pose.pose.position.z < self.Z_tag - 0.08):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = self.Z_tag
            self.pose_pub.publish(pose)
            rate.sleep()
        self.is_takeoff = True
        rospy.loginfo("[Controller] Take off")
     
    def control_loop(self):
        # PI controller parameters
        K_P = 0.8
        K_I = 0.1
        K_YAW_P = 0.8
        K_YAW_I = 0.1

        MAX_VEL_XY = 1.0   # m/s
        MAX_VEL_Z  = 0.5   # m/s
        MAX_Z_INT = 1.0    # m/s
        MAX_YAW_RATE = 0.7 # rad/s
        MAX_YAW_INT = 1.0 # for I in yaw control

        # Error check
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        error_x = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        error_y = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        error_z = self.target_pose.pose.position.z - self.current_pose.pose.position.z

        vx = K_P * error_x
        vy = K_P * error_y
        vz = K_P * error_z

        # --- yaw control (PI) ---
        # if abs(error_z) > 0.02:
        #     self.z_error_integral += error_z * dt
        
        # self.z_error_integral = clamp(self.z_error_integral, -MAX_Z_INT, MAX_Z_INT)
        # vz = vz + K_I * self.z_error_integral

        # --- clamp velocity ---
        self.cmd_velocity.twist.linear.x = clamp(vx, -MAX_VEL_XY, MAX_VEL_XY)
        self.cmd_velocity.twist.linear.y = clamp(vy, -MAX_VEL_XY, MAX_VEL_XY)
        self.cmd_velocity.twist.linear.z = clamp(vz, -MAX_VEL_Z, MAX_VEL_Z)

        # --- yaw control ---
        current_yaw = get_yaw_from_orientation(self.current_pose.pose.orientation)
        target_yaw = get_yaw_from_orientation(self.target_pose.pose.orientation) #self.target_yaw()
        error_yaw = yaw_error(target_yaw, current_yaw)
        
        # if abs(error_yaw) > 0.025:
        #     self.yaw_error_integral += error_yaw * dt
            
        # self.yaw_error_integral = clamp(self.yaw_error_integral, -MAX_YAW_INT, MAX_YAW_INT)
        # rospy.loginfo_throttle(1, "[Controller] current yaw error : %.2f, yaw : %.2f", error_yaw*180/np.pi, current_yaw*180/np.pi)

        # wz = K_YAW_P * error_yaw
        wz = K_YAW_P * error_yaw #+ K_YAW_I * self.yaw_error_integral
        self.cmd_velocity.twist.angular.z = clamp(wz, -MAX_YAW_RATE, MAX_YAW_RATE)

        # Publish the velocity
        self.local_vel_pub.publish(self.cmd_velocity)


if __name__ == "__main__":
    rospy.init_node("vel_control_node_py")
    rate = rospy.Rate(20)

    # Arming the drone
    rospy.Subscriber("mavros/state", State, callback = state_cb)
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # Controller
    controller = OffboardController()
    
    # Send few setpoints
    for _ in range(50):
        if rospy.is_shutdown():
            break
        controller.local_vel_pub.publish(controller.cmd_velocity)

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()
    rospy.loginfo("[Controller] Controller initialized")

    # Main loop
    while not rospy.is_shutdown():
        # --- Arming the drone ---
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        
        # Take off
        if (current_state.armed and not controller.is_takeoff):
            controller.takeOff()

        # Run the control loop
        controller.control_loop()
        rate.sleep()
    
