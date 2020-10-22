#!/usr/bin/env python

import rospy, tf
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

#Will be published on the parameters server/topic in the future
#robot= {'x': 2, 'y': -2, 'reach': 1.8} 
#IR focal length of Kinect 1 camera ~[6.1, 6.5] mm
robot= {'x': 2.95, 'y': 1.60, 'reach': 2}
time_delay = 0
stop_sign = 0

start_time = 0

TP = 0
TN = 0
FP = 0
FN = 0
NF = 0

def checkInterval(time_dec):
    if time_dec>8.665740 and time_dec<11.295466:
        return 1
    elif time_dec>21.702444 and time_dec<25.322700:
        return 1
    elif time_dec>31.879765 and time_dec<35.482993:
        return 1
    elif time_dec>43.329398 and time_dec<46.200230:
        return 1
    elif time_dec>44.434521 and time_dec<47.531761:
        return 1
    elif time_dec>46.200230 and time_dec<50.457970:
        return 1
    else: return 0

def newDetectedPersonsAvailable(detectedPersons):
    try:
        tfListener.waitForTransform(detectedPersons.header.frame_id, "odom",
				 detectedPersons.header.stamp, rospy.Duration(0.05))
    except tf.Exception:
        return
    '''
    distance = 100
    global start_time
    global stop_sign
    global last_detection_time
    global TP, TN, FP, FN, NF
    for detectedPerson in detectedPersons.detections:
        poseStamped = PoseStamped()
        poseStamped.pose = detectedPerson.pose.pose
        poseStamped.header = detectedPersons.header

        transformedPoseStamped = tfListener.transformPose("odom", poseStamped)
        pos = transformedPoseStamped.pose.position
        
        distance = ((robot['x']-pos.x)**2 + (robot['y']-pos.y)**2)**(1.0/2)
        #<= (robot[rech] + pos.z)
        if distance <= (robot['reach']):
            stop_sign = 1
            last_detection_time = rospy.get_rostime()
            break
        else: 
            stop_sign = 0
     
    current_time = rospy.get_rostime()
    if stop_sign and (current_time.secs - last_detection_time.secs > time_delay):
        stop_sign = 0
    
    if start_time == 0:
        start_time = rospy.get_rostime()
    
    time_dec = rospy.get_time() - 1591688411.701
    
    stop_msg = Float32MultiArray()
    stop_msg.data = []
    stop_msg.data = [stop_sign, time_dec,distance]   
    stopSignPublisher.publish(stop_msg)
    
    if stop_sign:
        if checkInterval(time_dec):
            TP += 1
        else: FP +=1
    elif not stop_sign:
        if checkInterval(time_dec):
            FN +=1
        else: TN +=1
    NF +=1
    '''
    time_dec = rospy.get_time()  - 1591000000
    num  = 0    
    if len(detectedPersons.detections) != 0:
        num = len(detectedPersons.detections)
    else: num = 0
    
    csvFile.write("{},{}\n".format(num,time_dec))
    
    global firstMessageOK
    if not firstMessageOK:
        firstMessageOK = True
        rospy.loginfo("First detections have been received, transformed!")
        #rospy.loginfo("{}\t{}\t{}\t{}\n{}\n" .format(pos.x, pos.y, pos.z, detectedPersons.header.stamp.to_sec(), stop_sign))


if __name__ == '__main__':
    rospy.init_node("monitored_safe_stop")

    global csvFile
    csvFile = open('safe_stop.txt','w')
    csvFile.write("TP,TN,FP,FN,time,frame\n")

    global tfListener
    tfListener = tf.TransformListener()

    global firstMessageOK
    firstMessageOK = False

    detectedPersonsTopic = "spencer/perception/detected_persons"
    detectedPersonsSubscriber = rospy.Subscriber(detectedPersonsTopic, DetectedPersons, newDetectedPersonsAvailable, queue_size=500)
    
    global stopSignPublisher
    stopSignTopic = "/spencer/perception/stop_sign"
    stopSignPublisher = rospy.Publisher(stopSignTopic, Float32MultiArray, queue_size=3)

    rospy.loginfo("Publishing stop sign on topic %s!" % (stopSignTopic))
    rospy.spin()

    csvFile.close()

