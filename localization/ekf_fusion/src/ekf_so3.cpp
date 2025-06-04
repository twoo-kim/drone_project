#include "ekf_so3.hpp"

void EKFNode::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    // Covariance for sensor data
    double orb_cov, tag_cov;
    nh_.param("ekf/orb_covariance", orb_cov, 0.1);
    nh_.param("ekf/tag_covariance", tag_cov, 0.1);
    orb_cov_ = orb_cov;
    tag_cov_ = tag_cov;

    // Initialize state
    prev_time_ = 0.0;
    is_orb_lost_ = false;

    // FSM
    mode_ = FusionMode::FULL;
    last_lost_keypoint_time_ = ros::Time(0);
    last_valid_keypoint_time_ = ros::Time(0);
    lost_time_ = ros::Duration(0.3);

    // Initialize EKF module
    ekf_.init(nh_);
    T_map_orb_ = Sophus::SE3d();
    last_ekf_state_ = ekf_.getState();
    

    // Initialize subscribers
    imu_sub_ = nh_.subscribe("/imu_topic", 100, &EKFNode::imuCallback, this);
    orb_sub_ = nh_.subscribe("/orb_pose", 10, &EKFNode::orbCallback, this);
    tag_sub_ = nh_.subscribe("/tag_pose", 10, &EKFNode::tagCallback, this);
    keypoint_sub_ = nh_.subscribe("/keypoints", 1, &EKFNode::keypointCallback, this);
    
    // Initialize the publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ekf_pose", 1);
}

void EKFNode::publishPose(void) {
    struct EKFState state = ekf_.getState();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.position.x = state.p.x();
    pose_msg.pose.position.y = state.p.y();
    pose_msg.pose.position.z = state.p.z();

    Eigen::Quaterniond q = state.R.unit_quaternion();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    pose_pub_.publish(pose_msg);
}

void EKFNode::imuCallback(const sensor_msgs::Imu& imu) {
    // Pass the very first call
    double now = imu.header.stamp.toSec();
    if (prev_time_ < 1e-8) {
        prev_time_ = now;
        return;
    }
    // Find dt
    double dt = now - prev_time_;
    prev_time_ = now;

    // Get linear acclearation and angular velocity
    Eigen::Vector3d acc(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    Eigen::Vector3d ang(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);

    // Predict state
    ekf_.predict(acc, ang, dt);
    publishPose();
}

void EKFNode::orbCallback(const geometry_msgs::PoseStamped& p_orb) {
    // Store valid ORB pose time
    ros::Time orb_time = p_orb.header.stamp;
    bool is_orb_lost = (orb_time - last_valid_keypoint_time_) > lost_time_;
    
    // Fuse ORB pose when mode is FULL
    if (mode_ != FusionMode::FULL) {
        return;
    }
    
    // Get pose and orientation
    Eigen::Vector3d p_meas(p_orb.pose.position.x, p_orb.pose.position.y, p_orb.pose.position.z);
    Eigen::Quaterniond q(p_orb.pose.orientation.w, p_orb.pose.orientation.x,
                         p_orb.pose.orientation.y, p_orb.pose.orientation.z);
    Sophus::SO3d R_meas(q);

    // Check if orb has lost its tracking before
    if (is_orb_lost) {
        // Compute offset from new ORB frame to the global frame
        Sophus::SE3d T_orb_new(R_meas, p_meas);
        Sophus::SE3d T_map_old(last_ekf_state_.R, last_ekf_state_.p);
        T_map_orb_ = T_map_old * T_orb_new.inverse();

        // Check error with the current value
        struct EKFState state = ekf_.getState();
        double pos_err = (p_meas - state.p).norm();
        // Adjust covariance
        double orb_std = std::max(0.05, pos_err);
        orb_cov_ = orb_std*orb_std;
        return;
    }

    // Apply offset
    Sophus::SE3d T_orb(R_meas, p_meas);
    Sophus::SE3d T_map = T_map_orb_ * T_orb;
    p_meas = T_map.translation();
    R_meas = T_map.so3();

    // Covariance matrix for ORB SLAM
    Eigen::Matrix<double, 6, 6> R_cov = Eigen::Matrix<double, 6, 6>::Identity();
    R_cov *= orb_cov_;

    ekf_.updatePose(p_meas, R_meas, R_cov);
}

void EKFNode::tagCallback(const geometry_msgs::PoseStamped& p_tag) {
    last_valid_tag_ = p_tag.header.stamp;

    // Get pose and orientation
    Eigen::Vector3d p_meas(p_tag.pose.position.x, p_tag.pose.position.y, p_tag.pose.position.z);
    Eigen::Quaterniond q(p_tag.pose.orientation.w, p_tag.pose.orientation.x,
                         p_tag.pose.orientation.y, p_tag.pose.orientation.z);
    Sophus::SO3d R_meas(q);

    // Covariance matrix for AprilTag
    Eigen::Matrix<double, 6, 6> R_cov = Eigen::Matrix<double, 6, 6>::Identity();
    // z-value is easy to be changed, make covariance larger than x, y
    R_cov *= tag_cov_;
    R_cov(2,2) *= 3.0;

    ekf_.updatePose(p_meas, R_meas, R_cov);
}

void EKFNode::keypointCallback(const sensor_msgs::PointCloud2::ConstPtr& key_points) {
    ros::Time now = ros::Time::now();

    // Check if keypoin has lost (width: number of tracked keypoints)
    if (key_points->width == 0) {
        last_ekf_state_ = ekf_.getState();
        last_lost_keypoint_time_ = now;
    } else {
        last_valid_keypoint_time_ = now;
    }
}

// Function for updating FSM
void EKFNode::updateMode(void) {
    bool orb_ok = (ros::Time::now() - last_valid_keypoint_time_) < lost_time_;
    bool tag_ok = (ros::Time::now() - last_valid_tag_time_) < lost_time_;

    switch (mode_) {
        case FusionMode::FULL:
        {
            // Check if sensor data are lost
            if (!orb_ok) {
                mode_ = tag_ok ? FusionMode::TAG : FusionMode::LOST;
            }
            // Enlarge process noise 5 times bigger
            if (mode_ == FusionMode::LOST) {
                ekf_.setProcessNoise(5.0);
            }
            break;
        }
        case FusionMode::TAG:
        {
            if (orb_ok) {
                mode_ = FusionMode::FULL;
            } else if (!tag_ok) {
                // Enlarge process noise 5 times bigger
                mode_ = FusionMode::LOST;
                ekf_.setProcessNoise(5.0);
            }
            break;
        }
        case FusionMode::LOST:
        {
            if (orb_ok) {
                mode_ = FusionMode::FULL;
                ekf_.setProcessNoise(0.2);
            } else if (tag_ok) {
                mode_ = FusionMode::TAG;
                ekf_.setProcessNoise(0.2);
            }
            break;
        }

    }
}