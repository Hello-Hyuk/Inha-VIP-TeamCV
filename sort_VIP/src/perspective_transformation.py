#! /usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import ObjectHypothesisWithPose

rospy.init_node('test_subscriber', anonymous=False)

def callback1(msg1): # bounding box
    try:
        print("Bbox x: ", msg1.detections[int(PPP)].bbox.center.x)
        print("Bbox y: ", msg1.detections[int(PPP)].bbox.center.y)
    except:
        print('No Person Detected')
    
def callback2(msg2): # confidence
    try:
        if len(msg2.data)!=0:
            print("confs: ", msg2.data[int(PPP)])
    except:
        print('No Person Detected')

def callback3(msg3): # class id
    global PPP # for only person
    PPP = 'fuck' # for only person
    
    if len(msg3.data)!=0:
        for i in range(len(msg3.data)):
            if msg3.data[i]==0: # person class is number 0
                PPP = i
                print("clss: ", msg3.data[i])
    #os.system('cls' if os.name =='nt' else 'clear')
    

def listener():
    rospy.Subscriber('/clss', Float32MultiArray, callback3) # class id , detecting only person
    rospy.Subscriber('/confs', Float32MultiArray, callback2) # confidence
    rospy.Subscriber('/bbox', Detection2DArray, callback1) # bouding box center x,y
    
if __name__ == '__main__':
    listener()
    rospy.spin()