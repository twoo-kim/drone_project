#include "apriltag_tf.hpp"

void AprilTagTF::init(ros::NodeHandle& nh) {
    nh_ = nh;

    /* Store ground truth value of apriltags */
    gate_pairs_ = {
        {{6.1314, -2.7626, 0.75}, {0.270598, 0.270598, -0.6532815, -0.6532815}}, // id 0; pair0
        {{13.7626, -2.8686, 0.75}, {0.6532815, 0.6532815, -0.270598, -0.270598}}, // id 4; pair1
        {{13.8686, 2.7626, 0.75}, {0.6532815, 0.6532815, 0.270598, 0.270598}},  // id 8; pair2
        {{6.2374, 2.8686, 0.75}, {0.270598, 0.270598, 0.6532815, 0.6532815}}   // id 12; pair3
    };

    // Initialize subscribers
    tag_sub_ = nh_.subscribe("/tag_detections", 10, &AprilTagTF::aprilCallback, this);

    // Initialize the publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/cam_pose", 1);
}

/* AprilTag pose transformation callback function */
void AprilTagTF::aprilCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag_array) {
    // Minimum distance and number of detections
    double min_dist = std::numeric_limits<double>::infinity();
    int n = 0;
    // Result
    Eigen::Vector3d p_wc = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_wc = Eigen::Quaterniond::Identity();

    /* Look over the detections array */
    for (const auto& detection : tag_array->detections) {
        // ID; we set the minimum id tag as the bundle frame (already sorted)
        int id = detection.id[0];
        Eigen::Vector3d p_wt = gate_pairs_[id/4].first;
        Eigen::Quaterniond q_wt = gate_pairs_[id/4].second;
        Sophus::SE3d T_wt(q_wt, p_wt);

        // Pose; pose of the tag with respect to the camera link
        const geometry_msgs::Pose pose = detection.pose.pose.pose;
        Eigen::Vector3d p_ct(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond q_ct(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Sophus::SE3d T_ct(q_ct, p_ct);

        // Check if the detected gate is the closest gate, else skip
        double current_dist = p_ct.norm();
        if (current_dist > min_dist) {
            continue;
        }
        min_dist = current_dist;

        Eigen::Matrix3d R;
        R <<  0,  0, 1,
             -1,  0, 0,
              0, -1, 0;
        
        Sophus::SE3d T_conv(R, Eigen::Vector3d::Zero());
        
        // Find the camera pose in world frame
        Sophus::SE3d T_wc = T_wt * T_ct.inverse() * T_conv.inverse();
        p_wc = T_wc.translation();
        q_wc = T_wc.so3().unit_quaternion();
        n++;
    }

    // Assert if there's no pose
    if (n <= 0) {
        ROS_WARN("No pose received!");
        return;
    }

    // Get the pose
    geometry_msgs::PoseStamped cam_pose;
    cam_pose.header.frame_id = "map";
    cam_pose.header.stamp = ros::Time::now();
    
    cam_pose.pose.position.x = p_wc.x();
    cam_pose.pose.position.y = p_wc.y();
    cam_pose.pose.position.z = p_wc.z();
    cam_pose.pose.orientation.w = q_wc.w();
    cam_pose.pose.orientation.x = q_wc.x();
    cam_pose.pose.orientation.y = q_wc.y();
    cam_pose.pose.orientation.z = q_wc.z();

    pose_pub_.publish(cam_pose);
}