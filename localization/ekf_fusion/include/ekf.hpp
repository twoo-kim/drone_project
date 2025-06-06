#ifndef EKF_HPP
#define EKF_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sophus/so3.hpp>

/* State of the EKF model */
struct EKFState {
    Eigen::Vector3d p;      // Position
    Eigen::Vector3d v;      // Velocity
    Sophus::SO3d R;         // Orientation
    Eigen::Vector3d b_acc;  // Acceleration bias
    Eigen::Vector3d b_ang;  // Angular velocity bias
};

class EKF {
public:
    EKF() {};

    // Initialize EKF model
    void init(ros::NodeHandle& nh);

    // Functions for estimation
    void predict(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, double dt);
    void updatePose(const Eigen::Vector3d& p_meas, const Sophus::SO3d& R_meas, const Eigen::Matrix<double, 6, 6>& R_cov);
    void setProcessNoise(double times);

    // Get state
    struct EKFState getState(void);

private:
    // State vector and Covariance for estimation model
    struct EKFState state_;
    Eigen::Matrix<double, 15, 15> P_;
    double acc_cov_, ang_cov_, acc_bias_cov_, ang_bias_cov_;
    
    // Gravity
    const Eigen::Vector3d gravity_ = Eigen::Vector3d(0,0,-9.81);

    /* ROS */
    ros::NodeHandle nh_;
};

#endif