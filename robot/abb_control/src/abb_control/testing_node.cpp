#include "testing.hpp"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "human_robot_testing";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    RobotControlTesting::HumanRobotTesting node(nh, nh_private);
    ROS_INFO("Initialized human-robot testing node.");
    ros::AsyncSpinner aspinner(4);
    aspinner.start();
    sleep(3.0);
    node.mainLoop();
    ros::waitForShutdown();
}