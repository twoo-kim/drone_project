#ifndef APRILTAG_TF_HPP
#define APRILTAG_TF_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <vector>
#include <utility>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseStamped.h"

class AprilTagTF {
public:
    AprilTagTF() {}
    void init(ros::NodeHandle& nh);

private:
    /* Callback function */
    void aprilCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_array);

    // Gate pairs' ground truth w.r.t. the map frame
    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> gate_pairs_;

    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber tag_sub_;
    ros::Publisher pose_pub_;
};

#endif