import cv2 as cv
import numpy as np
# find coordinate
def onMouse(event, x, y, flags, param) :
    if event == cv.EVENT_LBUTTONDOWN :
        print('왼쪽 마우스 클릭 했을 때 좌표 : ', x, y)


bbox = [0.4958, 0.7775, 0.10166, 0.11]
x_size = 600
y_size = 400
# const ROI좌표 
#nx, ny, nw, nh
#ROI 정보 


def find_box_center(bbox, img):
    width = img.cols
    height = img.rows
    cx = width * bbox[0]
    cy = height * bbox[1] 
    c_position = (cx,cy)
    return c_position

def bird_eye_view(bbox, frame):
    width = 600
    height = 400
    cx = width * bbox[0]
    cy = height * bbox[1] 
    c_position = (cx,cy, 1)
    print(c_position)
    # pts1, pts2 is ROI
    pts1 = np.float32([[214, 260], [459, 279], [7, 375], [539, 375]])
    pts2 = np.float32([[0, 0], [x_size, 0], [0, y_size], [x_size, y_size]])
    #print(car)
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    #print(matrix)
    frame = cv.warpPerspective(frame, matrix, (x_size, y_size))
    frame2 = np.dot(matrix, c_position)
    frame2 = frame2 / frame2[2];
    print(frame2)
    distance_car = 400 - frame2[1]
    print(distance_car)
    return frame


    

img = cv.imread('test1.jpg')

img = cv.resize(img,(600,400))
cv.imshow('car', img)
cv.setMouseCallback('car', onMouse)
img = bird_eye_view(bbox, img)
cv.imshow('car2', img)
cv.setMouseCallback('car2', onMouse)
cv.waitKey()

#img = scharr_filter(img)
#kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
#median_img=cv2.medianBlur(img,5)
#color_filtered_img = cv2.dilate(median_img, kernel)
#cv2.imshow('3-3. Thickened image', color_filtered_img)
#cv2.imshow("image", img)
#cv2.waitKey()