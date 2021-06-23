#include "human_robot_interaction.hpp"

namespace RobotControl {
	// constructor()
	HumanRobotInteraction::HumanRobotInteraction(const ros::NodeHandle &node_handle, 
												 const ros::NodeHandle &private_node_handle):
	nh(node_handle),
	pnh(private_node_handle),
	input_topic("/spencer/perception/tracked_persons_orientation_fixed"), 
	PLANING_GROUP("manipulator"),
	move_group(moveit::planning_interface::MoveGroupInterface(PLANING_GROUP)){
		this->init();
	}


	// destructor()
	HumanRobotInteraction::~HumanRobotInteraction() {
	}
	

	// init()
	void HumanRobotInteraction::init() {
		stop_timer = pnh.createTimer(ros::Duration(4.0), &HumanRobotInteraction::stopTimer, this);
		speed_timer = pnh.createTimer(ros::Duration(2.0), &HumanRobotInteraction::speedTimer, this);
		tracksSubscriber = pnh.subscribe(input_topic, 3, &HumanRobotInteraction::newTrackedReceived, this);
		
		// default robot position, mainly for testing
		robotPosition_x = 2.95;
		robotPosition_y = 2.0;
		robotRadius = 1.81;
		stopMovement = false;
		velocityFactor = 1.0;

		// real robot position
		tcrFileName = "/home/dario/Code/calib_ws/src/abbcmd/TCR.txt";
		//setCalibRobotPosition(tcrFileName);
		
		// two robot poses
		setPose(first_pose, 0.94768, 0.03007, 1.7031, 0.0, 0.99831, 0.0, 0.05813);
		setPose(second_pose, 0.82601, -0.3745, 1.624, 0.0, 0.99789, 0.0, -0.064918);
		
		move_group.setPlanningTime(10.0);
		move_group.allowReplanning(true);
		//move_group.setPlannerId();
	} // init()
	

	// newTrackedReceived()
	void HumanRobotInteraction::newTrackedReceived(const abb_control::TrackedPersons::ConstPtr& trackedPersons) {
		float trackPosition_x, trackPosition_y = 0;
		float trackSpeed_x, trackSpeed_y = 0;

		for (int i = 0; i < trackedPersons->tracks.size(); i++) {
			const abb_control::TrackedPerson& track = trackedPersons->tracks[i];
			
			// if the track is currently matched by a detector
			if (track.is_matched){
				trackPosition_x = track.pose.pose.position.x;
				trackPosition_y = track.pose.pose.position.y;
				trackSpeed_x = track.twist.twist.linear.x;
				trackSpeed_y = track.twist.twist.linear.y;

				this->robotControlStrategy(trackPosition_x, trackPosition_y, trackSpeed_x, trackSpeed_y);
			}
		}			
	} // newTrackedReceived()

	// robotControlStrategy()
	void HumanRobotInteraction::robotControlStrategy(float trackPosition_x, float trackPosition_y, float trackSpeed_x, float trackSpeed_y){
		geometry_msgs::Pose pose_buff;
		float trackSpeed, trackDistance = 0;
		float alpha, beta = 0;
		float gamma;
		float newVelocityFactor = 0;
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
				gamma = asinf(robotEnvelope/trackDistance);

				if (fabsf(alpha-beta) <= gamma){
					stopMovement = true;
				}
			}		
		}

		if (stopMovement){
			if (!stop_timer.hasStarted()){
				move_group.stop();
				resumePlaning = false;
				ROS_WARN("Stop condition met, robot motion disabled!");
			}
		
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
				if (!speed_timer.hasStarted()){
					ROS_WARN("Max velocity changed, replaning movement!");
				}
				velocityFactor = newVelocityFactor;
				move_group.stop();

				speed_timer.stop();
				speed_timer.start();
			}
		}
	} // robotControlStrategy()


	// timerCallback()
	void HumanRobotInteraction::stopTimer(const ros::TimerEvent &event) {
		stop_timer.stop();
		velocityFactor = 0.05;
		resumePlaning = true;

		speed_timer.stop();
		speed_timer.start();
	}  //timerCallback()


	// timerCallback()
	void HumanRobotInteraction::speedTimer(const ros::TimerEvent &event) {
		speed_timer.stop();
		if(!stop_timer.hasStarted()){
			velocityFactor = 1.0;
			resumePlaning = true;
			ROS_WARN("Resuming motion with normal speed...");
		}
	}  //timerCallback()
	

	// mainLoop()
	void HumanRobotInteraction::mainLoop(){
		ROS_INFO("Starting main loop.");
		move_group.clearPoseTargets();
		poseFlag = 0;
		
		while (ros::ok()){
			if (resumePlaning){

				current_pose = move_group.getCurrentPose().pose;
				move_group.setPoseTarget(current_pose);


				if (poseFlag==0){
					if (comparePosition(current_pose, first_pose) == 0) {
						move_group.setPoseTarget(first_pose);
					}
					else { 
						poseFlag = 1;
						continue;
					}
				}
				
				else if (poseFlag==1){
					if (comparePosition(current_pose, second_pose) == 0) {
						move_group.setPoseTarget(second_pose);
					}
					else {
						poseFlag = 0;
						continue;
					}
				}
				move_group.setMaxVelocityScalingFactor(velocityFactor);
				move_group.plan(move_plan);
				move_group.execute(move_plan);
			}
		}
	} // mainLoop()


	void HumanRobotInteraction::setCalibRobotPosition(std::string fileName){
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


	void HumanRobotInteraction::setPose(geometry_msgs::Pose &pose, float pX, float pY, float pZ, 
						 				float oX, float oY, float oZ, float oW){
		pose.position.x = pX;
		pose.position.y = pY;
		pose.position.z = pZ;
		pose.orientation.x = oX;
		pose.orientation.y = oY;
		pose.orientation.z = oZ;
		pose.orientation.w = oW;								 	
	}


	int HumanRobotInteraction::comparePosition(const geometry_msgs::Pose pose_A, const geometry_msgs::Pose pose_B, float diff){
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