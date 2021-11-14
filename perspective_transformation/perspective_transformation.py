import cv2 as cv
import numpy as np
x_size = 600
y_size = 600

# find coordinate
def onMouse(event, x, y, flags, param) :
    if event == cv.EVENT_LBUTTONDOWN :
        print('왼쪽 마우스 클릭 했을 때 좌표 : ', x, y)

# onMouse 를 통해 얻은 좌표
#(70,316)
#(131,265)
#(320,274)
#(379,328)

# perspective transfer
def bird_eye_view(frame):
    # pts1, pts2 is ROI
    pts1 = np.float32([[131, 265], [320, 274], [70, 316], [379, 328]])
    #pts1 = np.float32([[230, 150], [370, 150], [20, 280], [580, 280]])
    pts2 = np.float32([[0, 0], [x_size, 0], [0, y_size], [x_size, y_size]])
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    matrix_inv = cv.getPerspectiveTransform(pts2, pts1)
    frame = cv.warpPerspective(frame, matrix, (x_size, y_size))
    return frame

# scharr_filter
def scharr_filter(frame):
    img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # gray scale

    #sobel x,y filtering for gradient detection ====
    img_scharr_x = cv.Scharr(img_gray, cv.CV_64F, 1, 0)
    img_scharr_x = cv.convertScaleAbs(img_scharr_x)
    img_scharr_x2 = cv.Scharr(img_gray, cv.CV_64F, 0, 1)
    img_scharr_x2 = cv.convertScaleAbs(img_scharr_x2)
    #img_scharr_y = cv.Scharr(img_gray, cv.CV_64F, 0, 1)
    #img_scharr_y = cv.convertScaleAbs(img_scharr_y)
    #sobel x,y = sobel x + sobel y
    img_scharr = cv.addWeighted(img_scharr_x, 1, img_scharr_x2, 1, 0)
    #img_scharr = cv.addWeighted(img_scharr, 1, img_scharr_y, 1, 0)
    # detecting white mark and yellow mark
    # first,
    _, white_line = cv.threshold(img_scharr, 150, 255, cv.THRESH_BINARY)
    return white_line

img = cv.imread('test.jpg')
dst = cv.resize(img,(600,400),interpolation=cv.INTER_AREA)
bevimg = bird_eye_view(dst)
sfimg = scharr_filter(bevimg)
median_img=cv.medianBlur(sfimg, 5)

#thickend image
kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
color_filtered_img = cv.dilate(median_img, kernel)

#find coordinate
#마우스 좌클릭을 통해 좌표 확인
cv.imshow('image', img)
cv.setMouseCallback('image', onMouse)
cv.waitKey()

#show img

cv.imshow('perspective tranformation', bevimg)
#cv.setMouseCallback('image', onMouse)
cv.waitKey() 

cv.imshow('scharr filtered image', sfimg)
cv.waitKey()

cv.imshow('median image', median_img)
cv.waitKey()

cv.imshow('3-3. Thickened image', color_filtered_img)
cv.waitKey()
