#include "ekf_fusion.hpp"

void EKF::init(ros::NodeHandle& nh) {
    nh_ = nh;
    // Initialize state
    state_.p.setZero();
    state_.v.setZero();
    state_.q.setIdentity();

    // Initialize covariance
    P_.setIdentity();
    P_ *= 0.01;

    // Initialize the publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ekf_pose", 1);
}

/* Get partial derivative of rotation matrix with quaternion */
static Eigen::Matrix3d dR_dqi(const Eigen::Quaterniond &q, int idx) {
    const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Eigen::Matrix3d M;
    switch (idx) {
        case 0: // dR/dw
            M <<  0,   -2*z,  2*y,
                  2*z,   0,  -2*x,
                 -2*y,  2*x,   0;
            break;
        case 1: // dR/dx
            M <<   0,   2*y,   2*z,
                  2*y, -4*x,  -2*w,
                  2*z,  2*w, -4*x;
            break;
        case 2: // dR/dy
            M << -4*y,  2*x,   2*w,
                   2*x,   0,   2*z,
                  -2*w,  2*z, -4*y;
            break;
        case 3: // dR/dz
            M << -4*z,  -2*w,  2*x,
                   2*w, -4*z,  2*y,
                   2*x,   2*y,   0;
            break;
        default:
            M.setZero();
    }
    return M;
}


/* Update orientation */
void EKF::updateState(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, double dt) {
    // Orientation using quaternion
    Eigen::Vector3d wb = ang;
    double norm = wb.norm();
    if (norm > 1e-6) {
        // Find angle, axis and corresponding quaternion
        double theta = norm * dt;
        Eigen::Vector3d axis = wb / norm;
        Eigen::Quaterniond dq(Eigen::AngleAxisd(theta, axis));
        // q_wc = q_wb * q_bc
        state_.q = (state_.q * dq).normalized();
    }

    // Acceleartion in world frame
    Eigen::Matrix3d R_wb = state_.q.toRotationMatrix();
    Eigen::Vector3d acc_w = R_wb * lin + gravity_;

    // Update position and velocity
    state_.p += state_.v*dt + 0.5*acc_w*dt*dt;
    state_.v += acc_w*dt;
}


/* Predict the new state */
void EKF::predict(const Eigen::Vector3d& lin, const Eigen::Vector3d& ang, double dt) {
    /* 1. Update states */
    updateState(lin, ang, dt);

    /* 2. Jacobian for local linearization

             [dp/dp dp/dv dp/dq]
        F =  [dv/dp dv/dv dv/dq]  numerator(k+1), denominator(k)
             [dq/dp dq/dv dq/dq]

    */
    Eigen::Matrix<double,10,10> F = Eigen::Matrix<double,10,10>::Identity();

    // dp/dv = I*dt
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;

    // dp/dq = 0.5 * dR/dq * a_k * dt^2
    for (int i=0; i<4; i++) {
        Eigen::Vector3d col = 0.5*(dR_dqi(state_.q, i) * lin)*dt*dt;
        F.block<3,1>(0, 6+i) = col;
    }

    // dv/dq = dR/dq * a_k * dt
    for (int i=0; i<4; i++) {
        Eigen::Vector3d col = (dR_dqi(state_.q, i) * lin)*dt;
        F.block<3,1>(3, 6+i) = col;
    }

    // Leave dq/dq = I since dt << 1
    /* 3. Process noise */
    Eigen::Matrix<double,10,10> Q = Eigen::Matrix<double,10,10>::Identity();
    Q.block<3,3>(0,0).setZero();    // Position noise will be obtained from velocity, orientation noise
    Q *= 0.01;

    /* 4. Predicted Covariance */
    // P_k = FP_k-1 F^T + Q
    P_ = F*P_*F.transpose() + Q;
}

/* Estimate the position using Kalman Filter inputs are measured value and measurement covariance */
void updatePose(const Eigen::Vector3d& p_meas, const Eigen::Quaterniond& q_meas, const Eigen::Matrix<double 6, 6>& R_meas) {
    /* 1. Error */
    Eigen::Vector3d pos_err = p_meas - state_.p;
    Eigen::Quaterniod q_err = q_meas * state_.q.conjugate(); // Find dq
    Eigen::Vector3d rot_err = 2.0 * q_err.vec();    // usin(theta/2) ~ utheta/2 with small theta

    Eigen::Matrix<double,6,1> y;
    y << pos_err, rot_err;

    /* 2. Measurement Jacobian H (6 x 10 / output x input)
        pos: I
        rot: 2I (from quaternion angle)
    */
    Eigen::Matrix<double,6,10> H = Eigen::Matrix<double,6,10>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(3,7) = 2.0 * Eigen::Matrix3d::Identity();

    /* 3. Kalman filter */
    Eigen::Matrix<double,6,6> S = H*P_*H.transpose() + R_meas;
    Eigen::Matrix<double,10,6> K = P_*H.transpose()*S.invers();
    // dx = x - x_meas
    Eigen::Matrix<double,10,1> dx = K*y;

    /* 4. Apply estimation */
    state_.p += dx.segment<3>(0);
    state_.v += dx.segment<3>(3);

    // Adjust quaternion with estimation value q ~ (1, 0.5u) with theta << 1
    Eigen::Vector4d dq_vec = dx.segment<4>(6);
    Eigen::Quaterniond dq(1.0, 0.5*dq_vec(1), 0.5*dq_vec(2), 0.5*dq_vec(3));
    dq.normalize();
    state_.q = (state_.q * dq).normalized();

    /* 5. Update Covariance */
    Eigen::Matrix<double,10,10> I = Eigen::Matrix<double,10,10>::Identity();
    P_ = (I - K * H) * P_;
}
