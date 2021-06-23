#include "human_robot_interaction.hpp"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "human_robot_interaction";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    RobotControl::HumanRobotInteraction node(nh, nh_private);
    ROS_INFO("Initialized human-robot interaction node.");
    ros::AsyncSpinner aspinner(4);
    aspinner.start();
    ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
    sleep(3.0);
    node.mainLoop();
    ros::waitForShutdown();
}