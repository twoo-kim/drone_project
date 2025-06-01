#include "ekf_so3.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh("~");
    
    EKFNode ekf_node;
    
    ekf_node.init(nh);
    ros::spin();
    
    return 0;
}