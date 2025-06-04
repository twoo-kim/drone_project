#ifndef EKF_SO3_HPP
#define EKF_SO3_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"

#include "ekf.hpp"

/* FSM states */
// FULL: IMU+ORB+AprilTag, TAG: IMU+AprilTag, LOST: IMU only
enum class FusionMode {FULL, TAG, LOST};

/* EKF node class*/
class EKFNode {
public:
    EKFNode() {}

    void init(ros::NodeHandle& nh);

private:
    /* Callback functions */
    void imuCallback(const sensor_msgs::Imu& imu);
    void orbCallback(const geometry_msgs::PoseStamped& p_orb);
    void tagCallback(const geometry_msgs::PoseStamped& p_tag);
    void keypointCallback(const sensor_msgs::PointCloud2::ConstPtr& key_points);
    void publishPose(void);

    /* Utility functions */
    void updateMode(void);

    /* EKF estimation */
    EKF ekf_;
    double prev_time_;
    double orb_cov_, tag_cov_;

    /* FSM */
    FusionMode mode_;
    ros::Time last_lost_keypoint_time_;
    ros::Time last_valid_keypoint_time_;
    ros::Time last_valid_tag_time_;
    ros::Duration lost_time_;

    /* ORB relocalization */
    bool is_orb_lost_;
    struct EKFState last_ekf_state_;
    Sophus::SE3d T_map_orb_;

    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_, orb_sub_, tag_sub_, keypoint_sub_;
    ros::Publisher pose_pub_;
};

#endif