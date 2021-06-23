#ifndef HUMAN_ROBOT_INTERACTION_HPP
#define HUMAN_ROBOT_INTERACTION_HPP

#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <abb_control/TrackedPersons.h>
#include <abb_control/TrackedPerson.h>
#include <geometry_msgs/Point.h>


namespace RobotControlTesting {
	
	class HumanRobotTesting {
		public:
			HumanRobotTesting(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

			~HumanRobotTesting();
			
			void init();
			
			void newTrackedReceived(const abb_control::TrackedPersons::ConstPtr& trackedPersons);
			
			void robotControlStrategy(float trackPosition_x, float trackPosition_y, float trackSpeed_x, float trackSpeed_y);

			void stopTimer(const ros::TimerEvent& event);

			void speedTimer(const ros::TimerEvent& event);

			void mainLoop();

			void setCalibRobotPosition(std::string fileName);

			void setPose(geometry_msgs::Pose &pose, float pX, float pY, float pZ, 
						 				float oX, float oY, float oZ, float oW);

			int comparePosition(const geometry_msgs::Pose pose_A, const geometry_msgs::Pose pose_B, float diff=0.0002);

		private:
			// public ros node handle
			ros::NodeHandle nh;
			// private ros node handle
			ros::NodeHandle pnh;
			// names
			std::string input_topic;
			// timer
			ros::Timer stop_timer;
			ros::Timer speed_timer;
			// subscriber
			ros::Subscriber tracksSubscriber;
			// publisher

			//history
			typedef std::vector<std::pair<int,int>> pairVector;
			pairVector trackHistory;
			int maxTrackAge;

			// track-to-robot distance/speed
			bool stopMovement;
			bool resumePlaning;
			bool replanMovement;
			float velocityFactor;
			// robot position
			std::string tcrFileName;
			float robotPosition_x;
			float robotPosition_y;
			// robot radius
			float robotRadius;

			std::ofstream myfile;
			
	}; // class HumanRobotInteraction

} // namespace RobotControl
#endif