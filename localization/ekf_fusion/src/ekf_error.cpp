#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

class EKFError {
public:
    EKFError(ros::NodeHandle& nh) {
        pose_sub_ = nh.subscribe("/mavros/local_position/pose", 10, &EKFError::poseCallback, this);
        pred_sub_ = nh.subscribe("/ekf_node/ekf_pose", 10, &EKFError::predCallback, this);
    }

private:
    /* Compute error */
    double clamp(double value, double min, double max) {
        return std::max(min, std::min(value, max));
    }

    Eigen::Quaterniond normalizeQSign(const Eigen::Quaterniond& q) {
        if (q.w() < 0.0) {
            return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
        }
        return q;
    }

    void poseError(geometry_msgs::Pose& pred, geometry_msgs::Pose& truth) {
        Eigen::Vector3d p_pred(pred.position.x, pred.position.y, pred.position.z);
        Eigen::Vector3d p_truth(truth.position.x, truth.position.y, truth.position.z);
        Eigen::Quaterniond q_pred(pred.orientation.w, pred.orientation.x, pred.orientation.y, pred.orientation.z);
        Eigen::Quaterniond q_truth(truth.orientation.w, truth.orientation.x, truth.orientation.y, truth.orientation.z);

        // Error
        pose_err[0] = (p_pred - p_truth).norm();
        Eigen::Quaterniond q_err = normalizeQSign(q_pred) * normalizeQSign(q_truth).inverse();
        pose_err[1] = 2.0 * std::acos(clamp(q_err.w(), -1.0, 1.0));
    }

    /* Callback functions */
    void poseCallback(const geometry_msgs::PoseStamped& pose) {
        true_pose = pose.pose;
    }

    void predCallback(const geometry_msgs::PoseStamped& pose) {
        pred_pose = pose.pose;
        poseError(pred_pose, true_pose);
        ROS_INFO_THROTTLE(1.0,"Position error: %f, Orientation error: %f", pose_err[0], pose_err[1]);
    }

    /* Pose */
    geometry_msgs::Pose pred_pose, true_pose;
    Eigen::Vector2d pose_err;

    /* ROS */
    ros::Subscriber pred_sub_, pose_sub_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_error");
    ros::NodeHandle nh("~");
    
    EKFError error = EKFError(nh);
    ros::spin();
    
    return 0;
}