import cv2 as cv
import numpy as np

bbox = [0.4958, 0.7775, 0.10166, 0.11]

width = 600
height = 400

def find_distance(bbox, frame):
    #normalized 된 bbox 좌표를 원본 좌표로 변환하여 박스 중앙값 구하기

    cx = width * bbox[0]
    cy = height * bbox[1] 
    c_position = (cx,cy, 1)
    print(c_position)
    
    # pts1, pts2 is ROI
    # pts1은 임의로 지정한 4개의 콘 영역이다
    pts1 = np.float32([[214, 260], [459, 279], [7, 375], [539, 375]])
    pts2 = np.float32([[0, 0], [width, 0], [0,height], [width, height]])
    # ROI 사이의 homography 
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    
    # 영상 시점변환
    frame = cv.warpPerspective(frame, matrix, (width, height))
    
    # bbox 중점 시점변환
    trans_c_position = np.dot(matrix, c_position)
    trans_c_position = trans_c_position / trans_c_position[2];
    # bbox중점에서의 영상 끝의 거리 계산
    distance_car = 400 - trans_c_position[1]
    return distance_car

img = cv.imread('test1.jpg')
img = cv.resize(img,(600,400))
cv.imshow('car', img)
distance = find_distance(bbox, img)
print("distance",distance)