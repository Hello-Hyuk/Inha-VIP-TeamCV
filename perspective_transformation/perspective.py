import cv2
import numpy as np

x_size = 600
y_size = 400

def onMouse(event, x, y, flags, param) :
    if event == cv2.EVENT_LBUTTONDOWN :
        print('왼쪽 마우스 클릭 했을 때 좌표 : ', x, y)


def bird_eye_view(frame):
    # pts1, pts2 is ROI
    pts1 = np.float32([[214, 260], [459, 279], [7, 375], [539, 375]])
    pts2 = np.float32([[0, 0], [x_size, 0], [0, y_size], [x_size, y_size]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    frame = cv2.warpPerspective(frame, matrix, (x_size, y_size))
    return frame

def scharr_filter(frame):
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # gray scale

    #sobel x,y filtering for gradient detection ====
    img_scharr_x = cv2.Scharr(img_gray, cv2.CV_64F, 1, 0)
    img_scharr_x = cv2.convertScaleAbs(img_scharr_x)
    img_scharr_x2 = cv2.Scharr(img_gray, cv2.CV_64F, 0, 1)
    img_scharr_x2 = cv2.convertScaleAbs(img_scharr_x2)
    #img_scharr_y = cv2.Scharr(img_gray, cv2.CV_64F, 0, 1)
    #img_scharr_y = cv2.convertScaleAbs(img_scharr_y)
    #sobel x,y = sobel x + sobel y
    img_scharr = cv2.addWeighted(img_scharr_x, 1, img_scharr_x2, 1, 0)
    #img_scharr = cv2.addWeighted(img_scharr, 1, img_scharr_y, 1, 0)
    # detecting white mark and yellow mark
    # first,
    _, white_line = cv2.threshold(img_scharr, 150, 255, cv2.THRESH_BINARY)
    return white_line

img = cv2.imread('test1.jpg')

img = cv2.resize(img,(600,400))
cv2.imshow('car', img)
cv2.setMouseCallback('car', onMouse)
cv2.waitKey()

img = bird_eye_view(img)
img = scharr_filter(img)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
median_img=cv2.medianBlur(img, 5)
color_filtered_img = cv2.dilate(median_img, kernel)
cv2.imshow('3-3. Thickened image', color_filtered_img)
cv2.imshow("image", img)
cv2.waitKey()