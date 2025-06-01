#ifndef EKF_SO3_HPP
#define EKF_SO3_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"

#include "ekf.hpp"

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
    
    /* EKF estimation */
    EKF ekf_;
    double prev_time_;
    double orb_cov_, tag_cov_, refactor_threshold_;
    
    bool is_orb_lost_;
    ros::Time last_valid_time_;
    ros::Duration lost_threshold_;

    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_, orb_sub_, tag_sub_, keypoint_sub_;
    ros::Publisher pose_pub_;
};

#endif