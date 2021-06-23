#include "testing.hpp"

namespace RobotControlTesting {
	// constructor()
	HumanRobotTesting::HumanRobotTesting(const ros::NodeHandle &node_handle, 
												 const ros::NodeHandle &private_node_handle):
	nh(node_handle),
	pnh(private_node_handle),
	input_topic("/spencer/perception/tracked_persons_orientation_fixed"){
		this->init();
	}


	// destructor()
	HumanRobotTesting::~HumanRobotTesting() {
		myfile.close();
	}
	

	// init()
	void HumanRobotTesting::init() {
		stop_timer = pnh.createTimer(ros::Duration(4.0), &HumanRobotTesting::stopTimer, this);
		speed_timer = pnh.createTimer(ros::Duration(2.0), &HumanRobotTesting::speedTimer, this);
		tracksSubscriber = pnh.subscribe(input_topic, 3, &HumanRobotTesting::newTrackedReceived, this);
		
		// default robot position, mainly for testing
		robotPosition_x = 4.06;
		robotPosition_y = 2.22;
		robotRadius = 1.81;
		stopMovement = false;
		velocityFactor = 1.0;

		maxTrackAge = 3;

		
      	//myfile.open("/home/dario/tmp/test_box.csv");
	} // init()
	

	// newTrackedReceived()
	void HumanRobotTesting::newTrackedReceived(const abb_control::TrackedPersons::ConstPtr& trackedPersons) {
		float trackPosition_x, trackPosition_y = 0;
		float trackSpeed_x, trackSpeed_y = 0;
		pairVector::iterator pairPosition; 

		//myfile << trackedPersons->header.stamp;
		//myfile << ",";
		int n = 0;
		
		for (int i = 0; i < trackedPersons->tracks.size(); i++) {
			const abb_control::TrackedPerson& track = trackedPersons->tracks[i];
			int trackID = track.track_id;
			bool trackAlive = true; 
			

			pairPosition = std::find_if(trackHistory.begin(), trackHistory.end(),
			 			[&trackID](std::pair<int, int>const& element){
							return element.first == trackID;
						});
			if (pairPosition == trackHistory.end()) {
				if (track.is_matched) {
					trackHistory.push_back(std::make_pair(trackID,0));
				}
				else trackAlive = false;
			}
			else { 
				if (track.is_matched) {
					pairPosition->second = 0;
				}
				else {
					++(pairPosition->second);
				}
				if (pairPosition->second > maxTrackAge) {
					trackHistory.erase(pairPosition);
					trackAlive = false;
				}	
			}
			
			// if the track is currently matched by a detector
			if (trackAlive){
				trackPosition_x = track.pose.pose.position.x;
				trackPosition_y = track.pose.pose.position.y;
				trackSpeed_x = track.twist.twist.linear.x;
				trackSpeed_y = track.twist.twist.linear.y;

				n++;

				this->robotControlStrategy(trackPosition_x, trackPosition_y, trackSpeed_x, trackSpeed_y);
			}	
		}
		
		/* std::string msg = "";
		for(auto i : trackHistory)
			msg += std::to_string(i.first) + "--" + std::to_string(i.second) + " // ";
		ROS_WARN("#######  %s", msg.c_str()); */
		//myfile << "\n";
	} // newTrackedReceived()

	// robotControlStrategy()
	void HumanRobotTesting::robotControlStrategy(float trackPosition_x, float trackPosition_y, float trackSpeed_x, float trackSpeed_y){
		geometry_msgs::Pose pose_buff;
		float trackSpeed, trackDistance = 0;
		float alpha, beta = 0;
		float gamma;
		float newVelocityFactor = 0;
		float robotEnvelope = robotRadius;
		stopMovement = false;


		if (trackSpeed_x != 0 || trackSpeed_y != 0){
			trackSpeed = sqrtf(pow(trackSpeed_x,2) + pow(trackSpeed_y,2));
			//ROS_WARN("############### track spped: %.2f", trackSpeed);
		}

		if (trackSpeed >= 0.1 && trackSpeed < 1.0){
			robotEnvelope = 1.1*robotRadius;
		}
		else if (trackSpeed >= 1.0){
			robotEnvelope = 1.5*robotRadius;
		}

		trackDistance = sqrtf(pow(robotPosition_x - trackPosition_x,2) 
								+ pow(robotPosition_y - trackPosition_y,2));

		// stop conditions
		if (trackDistance <= robotEnvelope){
			stopMovement = true;
		}
		else { 
			if (trackSpeed >= 2){
				stopMovement = true;
			}
			else {
				alpha = atan2f(trackSpeed_y, trackSpeed_x);
				beta = atan2f((robotPosition_y-trackPosition_y),
							(robotPosition_x-trackPosition_x));
				gamma = asinf(robotEnvelope/trackDistance);

				if (fabsf(alpha-beta) <= gamma){
					stopMovement = true;
					ROS_WARN("############### alpha-beta, gamma : %.2f, %.2f", (alpha-beta),gamma);
				}
			}		
		}

		if (stopMovement){
			if (!stop_timer.hasStarted()){
				//move_group.stop();
				resumePlaning = false;
				//ROS_WARN("Stop condition met, robot motion disabled!");
			}
			//ROS_WARN("####################### envelope - distance = %.2f", (robotEnvelope-trackDistance));
		
			//myfile << "1";

			stop_timer.stop();
			stop_timer.start();
			return;
		}

		else if (!stop_timer.hasStarted()){
			if (trackSpeed < 0.1){
				newVelocityFactor = 0.05;
			}
			else {
				newVelocityFactor = 0.075; 
			}
			if (newVelocityFactor != velocityFactor){
				// TODO:: create new timer to reset velocity constraint
				if (!speed_timer.hasStarted()){
					//ROS_WARN("Max velocity changed, replaning movement!");
				}
				velocityFactor = newVelocityFactor;
				//move_group.stop();

				speed_timer.stop();
				speed_timer.start();
			}
		}
		//myfile << "0";
	} // robotControlStrategy()

/**
	// robotControlStrategy()
	void HumanRobotInteraction::robotControlStrategy(float trackPosition_x, float trackPosition_y, float trackSpeed_x, float trackSpeed_y){
		geometry_msgs::Pose pose_buff;
		float trackVRadial, trackVTangential = 0;
		float trackSpeed, trackDistance = 0;
		float alpha, beta = 0;
		float maxVelocityFactor = 0;
		float robotEnvelope = robotRadius;
		stopMovement = false;


		if (trackSpeed_x != 0 || trackSpeed_y != 0){
			trackSpeed = sqrtf(pow(trackSpeed_x,2) + pow(trackSpeed_y,2));
			
		}
		if (trackSpeed >= 0.1 && trackSpeed < 1.0){
			robotEnvelope = 1.1*robotRadius;
		}
		else if (trackSpeed >= 1.0){
			robotEnvelope = 1.5*robotRadius;
		}

		trackDistance = sqrtf(pow(robotPosition_x - trackPosition_x,2) 
								+ pow(robotPosition_y - trackPosition_y,2));

		// stop conditions
		if (trackDistance <= robotEnvelope){
			stopMovement = true;
		}
		else { 
			if (trackSpeed >= 2){
				stopMovement = true;
			}
			else {
				alpha = atan2f(trackSpeed_y, trackSpeed_x);
				beta = atan2f((robotPosition_y-trackPosition_y),
							(robotPosition_x-trackPosition_x));

				trackVRadial = trackSpeed*cosf(alpha - beta);
				ROS_WARN("################################## %.2f %.2f", alpha, beta);
				ROS_WARN("################################## velocity = %.2f", trackVRadial);

				if (trackVRadial > 0){
					stopMovement = false;
				}
			}		
		}

		if (stopMovement){
			move_group.stop();
			resumePlaning = false;
			ROS_WARN("Robot envelope breached, motion disabled!");
			//ROS_WARN("####################### envelope - distance = %.2f", (robotEnvelope-trackDistance));
		
			periodic_timer.stop();
			periodic_timer.start();
			return;
		}
		else if (!periodic_timer.hasStarted()){
			if (trackSpeed < 0.1){
				maxVelocityFactor = 0.5;
			}
			else {
				trackVTangential = trackSpeed*sinf(alpha - beta);
				maxVelocityFactor = 0.5*fabsf(trackVTangential); 
			}
			// TODO:: create new timer to reset velocity constraint
			ROS_WARN("Max velocity reduced, replaning movement!");
			move_group.setMaxVelocityScalingFactor(maxVelocityFactor);
			move_group.stop();
			replanMovement = true;
		}

	} // robotControlStrategy()
**/

	// timerCallback()
	void HumanRobotTesting::stopTimer(const ros::TimerEvent &event) {
		stop_timer.stop();
		velocityFactor = 0.05;
		resumePlaning = true;

		speed_timer.stop();
		speed_timer.start();
	}  //timerCallback()


	// timerCallback()
	void HumanRobotTesting::speedTimer(const ros::TimerEvent &event) {
		speed_timer.stop();
		if(!stop_timer.hasStarted()){
			velocityFactor = 1.0;
			resumePlaning = true;
			ROS_WARN("Resuming motion with normal speed...");
		}
	}  //timerCallback()
	

	// mainLoop()
	void HumanRobotTesting::mainLoop(){
		
		
	} // mainLoop()


	void HumanRobotTesting::setCalibRobotPosition(std::string fileName){
		std::ifstream inStream(fileName);
		float TCR[4][4];

		if(inStream.is_open()){
			for(int i = 0; i < 4; i++)
				for(int j = 0; j < 4; j++)
					inStream >> TCR[i][j];	
		}
		else{
			ROS_WARN("Could not open file: %s", fileName.c_str());
		}
		inStream.close();

		robotPosition_x = TCR[0][3];
		robotPosition_y = TCR[1][3];
	}


	void HumanRobotTesting::setPose(geometry_msgs::Pose &pose, float pX, float pY, float pZ, 
						 				float oX, float oY, float oZ, float oW){
		pose.position.x = pX;
		pose.position.y = pY;
		pose.position.z = pZ;
		pose.orientation.x = oX;
		pose.orientation.y = oY;
		pose.orientation.z = oZ;
		pose.orientation.w = oW;								 	
	}


	int HumanRobotTesting::comparePosition(const geometry_msgs::Pose pose_A, const geometry_msgs::Pose pose_B, float diff){
		// diff default is 0.0002
		if (fabsf(pose_A.position.x - pose_B.position.x) > diff ||
		    fabsf(pose_A.position.y - pose_B.position.y) > diff ||
		    fabsf(pose_A.position.z - pose_B.position.z) > diff){
			   return 0;
		   }
		else {
			return 1;
		}
	}

	
}	// namespace RobotControl