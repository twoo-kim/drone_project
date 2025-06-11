#include "ekf.hpp"

// Mahalanobis distance threshold
#define MAHAL_THRESHOLD 16.81

void EKF::init(ros::NodeHandle& nh) {
    nh_ = nh;

    // Covariance for model data
    double acc, ang, acc_bias, ang_bias;
    nh_.param("ekf/acceleartion_std", acc, 0.1);
    nh_.param("ekf/angular_velocity_std", ang, 0.1);
    nh_.param("ekf/acc_bias_std", acc_bias, 0.1);
    nh_.param("ekf/ang_bias_std", ang_bias, 0.1);
    acc_cov_ = acc*acc;
    ang_cov_ = ang*ang;
    acc_bias_cov_ = acc_bias*acc_bias;
    ang_bias_cov_ = ang_bias*ang_bias;

    // Initialize state
    state_.p.setZero();
    state_.v.setZero();
    state_.R = Sophus::SO3d();
    state_.b_acc.setZero();
    state_.b_ang.setZero();

    // Initialize covariance
    P_.setIdentity();
    P_ *= 0.0001;
}

// Hat function
static Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d M;
    M <<    0, -w.z(),   w.y(),
         w.z(),      0, -w.x(),
        -w.y(),  w.x(),      0;
    return M;
}

/* Predict the new state */
void EKF::predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& ang, double dt) {
    /* 1. Update nominal states */
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

    /* 2. Jacobian for error state */
    Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Zero();

    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity();                // dp/dv = I
    F.block<3,3>(3,6) = -state_.R.matrix() * hat(acc_corrected);    // dv/dR = -R * hat(acc)
    F.block<3,3>(3,9) = -state_.R.matrix();                         // dv/db_acc = -R
    F.block<3,3>(6,6) = -hat(ang_corrected);                        // dR/dR = -hat
    F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity();              // dR/db_ang = -I

    /* 3. Jacobian for noise */
    Eigen::Matrix<double,15,12> Fi = Eigen::Matrix<double,15,12>::Zero();
    Fi.block<3,3>(3,0) = -state_.R.matrix();
    Fi.block<3,3>(6,3) = -Eigen::Matrix3d::Identity();
    Fi.block<3,3>(9,6) = Eigen::Matrix3d::Identity();
    Fi.block<3,3>(12,9) = Eigen::Matrix3d::Identity();

    /* 4. Process noise */
    Eigen::Matrix<double,12,12> Qi = Eigen::Matrix<double,12,12>::Zero();
    Qi.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * acc_cov_;
    Qi.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * ang_cov_;
    Qi.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * acc_bias_cov_;
    Qi.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * ang_bias_cov_;

    /* 5. Discrete system Jacobian; Fx = I + F*dt and Q = Fi*Qi*Fi^T */
    Eigen::Matrix<double,15,15> Fx = Eigen::Matrix<double,15,15>::Identity() + F * dt;
    Eigen::Matrix<double,15,15>  Q = Fi * Qi * Fi.transpose() * dt;

    /* 6. Predicted Covariance */
    // P_k = FP_k-1 F^T + Q
    P_ = Fx*P_*Fx.transpose() + Q;
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

    /* 3. Kalman gain */
    Eigen::Matrix<double,6,6> S = H*P_*H.transpose() + R_cov;
    Eigen::Matrix<double,15,6> K = P_*H.transpose()*S.inverse();
    // Outlier check; Mahalanobis distance with 95% confidence chi^2 = 12.59
    double mahalanobis_dist = y.transpose() * S.inverse() * y;
    if (mahalanobis_dist > MAHAL_THRESHOLD) {
        return;
    }

    // dx = x - x_meas
    Eigen::Matrix<double,15,1> dx = K*y;

    /* 4. Apply estimation */
    state_.p += dx.segment<3>(0);
    state_.v += dx.segment<3>(3);
    state_.R = state_.R * Sophus::SO3d::exp(dx.segment<3>(6));
    state_.b_acc += dx.segment<3>(9);
    state_.b_ang += dx.segment<3>(12);

    /* 5. Update Covariance */
    Eigen::Matrix<double,15,15> I = Eigen::Matrix<double,15,15>::Identity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + (K * R_cov * K.transpose());

    /* 6. Reset */
    Eigen::Matrix<double,15,15> G = Eigen::Matrix<double,15,15>::Identity();
    G.block<3,3>(6,6) = Eigen::Matrix3d::Identity() - 0.5*hat(dx.segment<3>(6));
    P_ = G * P_ * G.transpose();
}

struct EKFState EKF::getState(void) {
    return state_;
}

void EKF::setProcessNoise(double times) {
    acc_cov_ *= times;
    ang_cov_ *= times;
}