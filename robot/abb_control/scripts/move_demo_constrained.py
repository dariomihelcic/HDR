#!/usr/bin/env python
from abb_control.abbCtrl import abbRobot
import rospy as rp

def compare_points(pntA, pntB, diff=0.05):
	for i in range(3):
		if pntA[i] - pntB[i] > diff:
			return 0
	return 1
 
rp.init_node('abbMove_Main')
robot=abbRobot()

pt_1=[[0.8, -0.5, 1.62]]
pt_2=[[1,0.5,1.62]]

robot.cartesian2Point(pt_1,[0,0,0],resolution=0.02)
while not rp.is_shutdown():
	current_point=robot.getEEPoint()[0]
	
	if compare_points(current_point, pt_1[0]):
		rp.sleep(0.5)		
		robot.cartesian2Point(pt_2,[0,0,0],resolution=0.02)
	elif compare_points(current_point, pt_2[0]):
		rp.sleep(0.5)
		robot.cartesian2Point(pt_1,[0,0,0],resolution=0.02)
	rp.sleep(1.4)
