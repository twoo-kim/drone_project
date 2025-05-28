#include <ros/ros.h>
#include "apriltag_tf.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_tf_node");
    ros::NodeHandle nh("~");
    
    AprilTagTF apriltag_tf_node;
    
    apriltag_tf_node.init(nh);
    ros::spin();
    
    return 0;
}