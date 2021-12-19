import rospy
import cv2 as cv
import numpy as np

from std_msgs.msg import String
from BoundingBox2D.msg import BoundingBox2D
from Detector2D.msg import Detector2D
from Detector2DArray.msg import Detector2DArray
from ObjectiveHypothesis.msg import ObjectiveHypothesis

bbox = [1, 2, 3, 4] # [x, y, w, h] 
width = 1920
height = 1080

def talker(distance):
    pub=rospy.Publisher('destance_msg',String,queue_size=100)
    rate=rospy.Rate(10)
    pub.publish(distance)

def callback():
    hi=1

def listener():
    rospy.Subscriber('detector', Detector2DArray, callback)
    rospy.spin()

def find_distance(bbox):
    cx = bbox[0] + (bbox[2]/2)
    cy = bbox[1] + (bbox[3]/2)
    c_position = (cx,cy, 1)
    
    # pts1, pts2 is ROI
    # pts1은 임의로 지정한 4개의 콘 영역이다
    # [457, 279], [612, 282], [147, 720], [955, 720]
    pts1 = np.float32([[812, 419], [1088, 423], [216, 1080], [1698, 1080]])
    pts2 = np.float32([[0, 0], [width, 0], [0,height], [width, height]])
    # ROI 사이의 homography 
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    
    # bbox 중점 시점변환
    trans_c_position = np.dot(matrix, c_position)
    trans_c_position = trans_c_position / trans_c_position[2]
    # bbox중점에서의 영상 끝의 거리 계산
    distance_car = 1080 - trans_c_position[1]
    return distance_car


rospy.init_node('perspective_node',anonymous=True)
distance = find_distance(bbox)
talker(distance)
listener()

