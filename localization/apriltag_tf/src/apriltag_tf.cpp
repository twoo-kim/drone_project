#include "apriltag_tf.hpp"
#include <sophus/se3.hpp>

void AprilTagTF::init(ros::NodeHandle& nh) {
    nh_ = nh;

    /* Store ground truth value of apriltags */
    gate_pairs_ = {
        {{6.1844, -2.8156, 0.75}, {0.9239, 0, 0, -0.3827}}  // id 0; pair0
        {{13.8156, -2.8156, 0.75}, {0.9239, 0, 0, 0.3827}}  // id 4; pair1
        {{13.8156, 2.8156, 0.75}, {0.9239, 0, 0, 0.3827}}   // id 8; pair2
        {{3.8156, 2.8156, 0.75}, {0.9239, 0, 0, -0.3827}}   // id 12; pair3
    };

    // Initialize subscribers
    tag_sub_ = nh_.subscribe("/tag_detections", 10, &EKFNode::aprilCallback, this);

    // Initialize the publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/cam_pose", 1);
}

/* AprilTag pose transformation callback function */
void aprilCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_array) {
    // Lie algebra for average; suppose estimations are close enough
    Eigen::Matrix<double, 6, 1> lie = Eigen::Matrix<double, 6, 1>::Zero();
    double n = 0.0;

    /* Look over the detections array */
    for (const auto& detection : tag_array->detections) {
        // ID; we set the minimum id tag as the bundle frame
        int id = detection.id[0];
        Sophus::SE3d T_wt(gate_pairs_[id/4][1], gate_pairs_[id/4][0]);

        // Pose; pose of the tag with respect to the camera link
        const geometry_msgs::Pose pose = detection.pose.pose.pose;
        Eigen::Vector3d p_ct(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond q_ct(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Sophus::SE3d T_cam_tag(q_ct, p_ct);

        // Camera pose in the world frame; T_wc = T_wt * T_tc (= T_ct^-1)
        Sophus::SE3d T_wc;
        T_wc = T_wt * T_cam_tag.inverse();
        lie += T_wc.log();
        n++;
    }
    // Get the pose
    geometry_msgs::PoseStamped cam_pose;
    cam_pose.header.frame_id = "map"
    cam_pose.header.stamp = ros::Time::now();
    
    Sophus::SE3d avg_pose = Sophus::SE3d::exp(lie/n);
    Eigen::Vector3d avg_p = avg_pose.translation();
    Eigen::Quaterniond avg_q = avg_pose.unit_quaternion();
    
    cam_pose.pose.position.x = avg_p.x();
    cam_pose.pose.position.x = avg_p.y();
    cam_pose.pose.position.x = avg_p.z();
    cam_pose.pose.orientation.w = avg_q.w();
    cam_pose.pose.orientation.x = avg_q.x();
    cam_pose.pose.orientation.y = avg_q.y();
    cam_pose.pose.orientation.z = avg_q.z();

    pose_pub_.publish(cam_pose);
}