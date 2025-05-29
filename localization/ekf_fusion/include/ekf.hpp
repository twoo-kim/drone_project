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
};

class EKF {
public:
    EKF() {};

    // Initialize EKF model
    void init(ros::NodeHandle& nh);

    // Functions for estimation
    void predict(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, double dt);
    void updatePose(const Eigen::Vector3d& p_meas, const Sophus::SO3d& R_meas, const Eigen::Matrix<double, 6, 6>& R_cov);

    // Get state
    struct EKFState getState(void);

private:
    // Update state
    void updateState(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, double dt);

    // State vector and Covariance for estimation model
    struct EKFState state_;
    Eigen::Matrix<double, 9, 9> P_;
    double process_cov_;
    
    // Gravity
    const Eigen::Vector3d gravity_ = Eigen::Vector3d(0,0,-9.81);

    /* ROS */
    ros::NodeHandle nh_;
};

#endif