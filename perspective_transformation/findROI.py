import cv2 as cv
import numpy as np

width = 395
height = 500
car2cone = 300

# find coordinate
def onMouse(event, x, y, flags, param) :
    if event == cv.EVENT_LBUTTONDOWN :
        print('왼쪽 마우스 클릭 했을 때 좌표 : ', x, y)

cap = cv.VideoCapture(1)               # 1번 카메라 장치 연결 ---①
if cap.isOpened():                      # 캡쳐 객체 연결 확인
    while True:
        ret, img = cap.read()           # 다음 프레임 읽기
        
        if ret:
            hi=cv.imshow('camera', cv.resize(img,(1080,720)))   # 다음 프레임 이미지 표시
            cv.setMouseCallback('camera', onMouse)
            if cv.waitKey(1) != -1:    # 1ms 동안 키 입력 대기 ---②
                break                   # 아무 키라도 입력이 있으면 중지
        else:
            print('no frame')
            break
else:
    print("can't open camera.")
cap.release()                           # 자원 반납
cv.destroyAllWindows()