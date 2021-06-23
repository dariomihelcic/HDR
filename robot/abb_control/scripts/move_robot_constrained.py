#!/usr/bin/env python
import rospy as rp
from abb_control.abbCtrl import abbRobot
from std_msgs.msg import Float32MultiArray

pt_1=[[0.8, -0.5, 1.62]]
pt_2=[[1,0.5,1.62]]

global maxVelocity

def movmentConstraint(stopSing):
    global maxVelocity
    if stopSing.data[2] < 3.0:
        maxVelocity = 0.5
    elif stopSing.data[2] < 2.0:
        maxVelocity = 0.001
    else:
        maxVelocity = 1
    
def compare_points(pntA, pntB, diff=0.05):
	for i in range(3):
		if pntA[i] - pntB[i] > diff:
			return 0
	return 1



if __name__ == '__main__':
    rp.init_node('abbMove_Main')
    robot=abbRobot()
    
    stopSignTopic = "/spencer/perception/stop_sign"
    rp.Subscriber(stopSignTopic, Float32MultiArray, movmentConstraint, queue_size=10)

    global maxVelocity
    maxVelocity = 1.0   
    robot.cartesian2Point(pt_1,[0,0,0],resolution=0.02, maxVelocityFactor = maxVelocity)
    while not rp.is_shutdown(): 
        current_point=robot.getEEPoint()[0]
        	
        if compare_points(current_point, pt_1[0]):		
            robot.cartesian2Point(pt_2,[0,0,0],resolution=0.02, maxVelocityFactor = maxVelocity)
        elif compare_points(current_point, pt_2[0]):
            robot.cartesian2Point(pt_1,[0,0,0],resolution=0.02, maxVelocityFactor = maxVelocity)
        rp.sleep(2)
