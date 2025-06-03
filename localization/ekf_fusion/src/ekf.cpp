#include "ekf.hpp"

void EKF::init(ros::NodeHandle& nh) {
    nh_ = nh;

    // Covariance for model data
    double acc, ang, acc_bias, ang_bias;
    nh_.param("ekf/acceleartion_covariance", acc, 0.1);
    nh_.param("ekf/angular_velocity_covariance", ang, 0.1);
    nh_.param("ekf/acc_bias_covariance", acc_bias, 0.1);
    nh_.param("ekf/ang_bias_covariance", ang_bias, 0.1);
    acc_cov_ = acc;
    ang_cov_ = ang;
    acc_bias_cov_ = acc_bias;
    ang_bias_cov_ = ang_bias;

    // Initialize state
    state_.p.setZero();
    state_.v.setZero();
    state_.R = Sophus::SO3d();
    state_.b_acc.setZero();
    state_.b_ang.setZero();

    // Initialize covariance
    P_.setIdentity();
    P_ *= 0.01;
    P_.block<3,3>(9,9) *= 0.1;  // accel bias
    P_.block<3,3>(12,12) *= 0.1; // gyro bia
}

/* Update orientation */
void EKF::updateState(const Eigen::Vector3d& acc, const Eigen::Vector3d& ang, double dt) {
    // Bias corrected inputs
    Eigen::Vector3d acc_corrected = acc - state_.b_acc;
    Eigen::Vector3d ang_corrected = ang - state_.b_ang;

    // Update orientation
    // Note that R(w+dw) = R(w)*exp(dw) where dw = angular change = ang * dt in so(3)
    Sophus::SO3d dR = Sophus::SO3d::exp(ang_corrected * dt);
    state_.R = state_.R * dR;

    // Acceleartion in world frame
    Eigen::Vector3d acc_w = state_.R * acc_corrected + gravity_;

    // Update position and velocity
    state_.p += state_.v*dt + 0.5*acc_w*dt*dt;
    state_.v += acc_w*dt;
}


/* Predict the new state */
void EKF::predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& ang, double dt) {
    /* 1. Update states */
    updateState(acc, ang, dt);

    /* 2. Jacobian for local linearization

             [dp/dp dp/dv dp/dR]
        F =  [dv/dp dv/dv dv/dR]  numerator(k+1), denominator(k)
             [dR/dp dR/dv dR/dR]

    */
    Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();

    // dp/dv = I*dt
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;

    // dp/dR = 0.5 * dRa/dR * dt^2
    F.block<3,3>(0,6) = -0.5 * state_.R.matrix() * Sophus::SO3d::hat(acc - state_.b_acc) *dt*dt;

    // dp/db_acc = -0.5 * dR/db_acc * dt^2
    F.block<3,3>(0,9) = -0.5 * state_.R.matrix() * dt*dt;

    // dv/dR = dRa/dR * dt
    F.block<3,3>(3,6) = -state_.R.matrix() * Sophus::SO3d::hat(acc - state_.b_acc) *dt;

    // dv/db_acc = -R*dt
    F.block<3,3>(3,9) = -state_.R.matrix() * dt;

    // dR/dR = exp(dw)
    F.block<3,3>(6,6) = Sophus::SO3d::exp(ang * dt).matrix();

    // dR/db_ang ~ -R*dt (suppose small angle)
    F.block<3,3>(6,12) = -state_.R.matrix() * dt;

    /* 3. Process noise */
    Eigen::Matrix<double,15,15> Q = Eigen::Matrix<double,15,15>::Zero();
    Q.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * acc_cov_;
    Q.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * ang_cov_;
    Q.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * acc_bias_cov_;
    Q.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * ang_bias_cov_;

    /* 4. Predicted Covariance */
    // P_k = FP_k-1 F^T + Q
    P_ = F*P_*F.transpose() + Q;
}

/* Estimate the position using Kalman Filter inputs are measured value and measurement covariance */
void EKF::updatePose(const Eigen::Vector3d& p_meas, const Sophus::SO3d& R_meas, const Eigen::Matrix<double, 6, 6>& R_cov) {
    /* 1. Error */
    // orientation error = R_meas - R = Log(R^-1 * R_meas)
    Eigen::Vector3d pos_err = p_meas - state_.p;
    Eigen::Vector3d ori_err = (state_.R.inverse() * R_meas).log(); 

    Eigen::Matrix<double,6,1> y;
    y << pos_err, ori_err;

    /* 2. Measurement Jacobian H; output = [p R] */
    Eigen::Matrix<double,6,15> H = Eigen::Matrix<double,6,15>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3,6) = Eigen::Matrix3d::Identity();

    /* 3. Kalman filter */
    Eigen::Matrix<double,6,6> S = H*P_*H.transpose() + R_cov;
    Eigen::Matrix<double,15,6> K = P_*H.transpose()*S.inverse();
    // dx = x - x_meas
    Eigen::Matrix<double,15,1> dx = K*y;

    /* 4. Apply estimation */
    state_.p += dx.segment<3>(0);
    state_.v += dx.segment<3>(3);
    state_.R = state_.R * Sophus::SO3d::exp(dx.segment<3>(6));

    /* 5. Update Covariance */
    Eigen::Matrix<double,15,15> I = Eigen::Matrix<double,15,15>::Identity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + (K * R_cov * K.transpose());
}

struct EKFState EKF::getState(void) {
    return state_;
}