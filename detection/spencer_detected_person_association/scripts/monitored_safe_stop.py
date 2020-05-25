#!/usr/bin/env python

import rospy
import rospy, tf
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16

#Will be published on the parameters server/topic in the future
robot= {'x': 2, 'y': -2, 'reach': 1.8} 

def newDetectedPersonsAvailable(detectedPersons):
    try:
        tfListener.waitForTransform(detectedPersons.header.frame_id, "odom", detectedPersons.header.stamp, rospy.Duration(0.05))
    except tf.Exception:
        return

    for detectedPerson in detectedPersons.detections:
        poseStamped = PoseStamped()
        poseStamped.pose = detectedPerson.pose.pose
        poseStamped.header = detectedPersons.header

        transformedPoseStamped = tfListener.transformPose("odom", poseStamped)

        pos = transformedPoseStamped.pose.position
        stop_sign = 1 if ((robot['x']-pos.x)**2 + (robot['y']-pos.y)**2)**(1.0/2) <= (robot['reach'] + pos.z) else 0
        
        stopSignPublisher.publish(stop_sign)

    global firstMessageOK
    if not firstMessageOK:
        firstMessageOK = True
        rospy.loginfo("First detections have been received, transformed!")
        rospy.loginfo("{}\t{}\t{}\t{}\n{}\n" .format(pos.x, pos.y, pos.z, detectedPersons.header.stamp.to_sec(), stop_sign))


if __name__ == '__main__':
    rospy.init_node("monitored_safe_stop")

    global csvFile
    csvFile = open('safe_stop.txt','w')

    global tfListener
    tfListener = tf.TransformListener()

    global firstMessageOK
    firstMessageOK = False

    detectedPersonsTopic = "spencer/perception/detected_persons"
    detectedPersonsSubscriber = rospy.Subscriber(detectedPersonsTopic, DetectedPersons, newDetectedPersonsAvailable, queue_size=500)
    
    global stopSignPublisher
    stopSignTopic = "/spencer/perception/stop_sign"
    stopSignPublisher = rospy.Publisher(stopSignTopic, Int16, queue_size=3)

    rospy.loginfo("Publishing stop sign on topic %s!" % (stopSignTopic))
    rospy.spin()

    csvFile.close()

