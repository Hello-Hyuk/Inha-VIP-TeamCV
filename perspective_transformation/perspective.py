import cv2 as cv
import numpy as np

bbox = [1, 2, 3, 4] # [x, y, w, h]

width = 1920
height = 1080

def find_distance(bbox):

    cx = bbox[0] + (bbox[2]/2)
    cy = bbox[1] + (bbox[3]/2)
    c_position = (cx,cy, 1)
    
    # pts1, pts2 is ROI
    # pts1은 임의로 지정한 4개의 콘 영역이다
    pts1 = np.float32([[457, 279], [612, 282], [147, 720], [955, 720]]) # 이거 1080때 인지 기억이 안납니다!! 1080 720 때면 전환해야합니다
    pts2 = np.float32([[0, 0], [width, 0], [0,height], [width, height]])
    # ROI 사이의 homography 
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    
    # bbox 중점 시점변환
    trans_c_position = np.dot(matrix, c_position)
    trans_c_position = trans_c_position / trans_c_position[2]
    # bbox중점에서의 영상 끝의 거리 계산
    distance_car = 1080 - trans_c_position[1]
    return distance_car

distance = find_distance(bbox)
print("distance",distance)