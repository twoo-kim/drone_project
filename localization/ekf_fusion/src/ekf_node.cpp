#include "ekf_node.hpp"

void EKFNode::init(ros::NodeHandle& nh) {
    nh_ = nh;
    // Initialize state
    prev_time = 0;

    // Initialize EKF module
    ekf_.init();

    // Initialize subscribers
    imu_sub_ = nh_.subscribe("/imu_topic", 100, &EKFNode::imuCallback, this);
    orb_sub_ = nh_.subscribe("/orb_pose", 10, &EKFNode::orbCallback, this);
    tag_sub_ = nh_.subscribe("/tag_pose", 10, &EKFNode::tagCallback, this);

    // Initialize the publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ekf_pose", 1);
}

void publishPose(void) {
    struct EKFState state = ekf_.getState();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map"
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.position.x = state.p.x()
    pose_msg.pose.position.y = state.p.y()
    pose_msg.pose.position.z = state.p.z()

    Eigen::Quaterniond q = state.R.unit_quaternion();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    pose_pub.publish(pose_msg);
}

void imuCallback(const sensor_msgs::Imu& imu) {
    // Find dt
    double now = imu.header.stamp.toSec();
    double dt = now - prev_time_;
    prev_time_ = now;

    // Predict state
    ekf_.predict(imu.linear_acceleration, imu.angular_velocity, dt);
    publishPose();
}

void orbCallback(const geometry_msgs::PoseStamped& p_orb) {
    // Get pose and orientation
    Eigen::Vector3d p_meas(p_orb.pose.position.x, p_orb.pose.position.y, p_orb.pose.position.z)
    Eigen::Quaterniond q(p_orb.pose.orientation.w, p_orb.pose.orientation.x
                         p_orb.pose.orientation.y, p_orb.pose.orientation.z)
    Sophus::SO3d R_meas(q);
    
    // Covariance matrix for ORB SLAM
    Eigen::Matrix<double 6, 6> R_cov = Eigen::Matrix<double 6, 6>::Identity();
    R_cov *= 0.15;

    updatePose(p_meas, R_meas, R_cov);
}

void tagCallback(const geometry_msgs::PoseStamped& p_tag) {
    // Get pose and orientation
    Eigen::Vector3d p_meas(p_tag.pose.position.x, p_tag.pose.position.y, p_tag.pose.position.z)
    Eigen::Quaterniond q(p_tag.pose.orientation.w, p_tag.pose.orientation.x
                         p_tag.pose.orientation.y, p_tag.pose.orientation.z)
    Sophus::SO3d R_meas(q);
    
    // Covariance matrix for AprilTag
    Eigen::Matrix<double 6, 6> R_cov = Eigen::Matrix<double 6, 6>::Identity();
    R_cov *= 0.01;

    updatePose(p_meas, R_meas, R_cov);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh("~");
    
    EKFNode ekf_node;
    
    ekf_node.init(nh);
    ros::spin();
    
    return 0;
}