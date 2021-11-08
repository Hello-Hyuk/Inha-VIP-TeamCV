import numpy as np
import matplotlib.pyplot as plt
import sympy as sym

dt = np.arange(0, 20, 2) #t(2) - t(1)을 2s 로 두어 0초부터 20초까지 진행되는 행렬 만듬

#계산식 
#P = error * Kp
#I += error*Ki*dt
#D = [error(2) - error(1)] / dt *Kd
#주어지는 변수 dt 
Kp = 6.0 #비례 계수
Ki = 0.4 #적분 계수
Kd = 2.0 #미분 계수

P = list()          #P = error * Kp
I = [0]             #I += error*Ki*dt
D = [0]             #D = [error(2)-error(1)] / dt*Kd
PID = list()        #P+I+D 저장할 리스트 
velocity = list()   #PID*alpha를 통해 속력으로 환산한 값을 저장 

#alpha = 3
GOAL = 20                           #commend variable(목표값)
error = list()                      #error
value = [0,3,7,13,20,19,21,23,20,20]#측정값

for i in range(0, len(value)):
    error.append(GOAL - value[i])

for i in range(0, len(error)):

    P.append(error[i]*Kp) 
    if i == 0:
        D[i] = 0
        I[i] = 0
    else:
        I.append((error[i] * Ki)*dt[i] + I[i-1])
        D.append(((error[i] - error[i-1]) / dt[i]) * Kd)
    
    PID.append(P[i]+I[i]+D[i])

    velocity.append(PID[i])

plt.figure()
plt.subplot(4,1,1)
plt.plot(dt, P)
plt.xlabel('dt')
plt.ylabel('P')

plt.subplot(4,1,2)
plt.plot(dt, I)
plt.xlabel('dt')
plt.ylabel('I')

plt.subplot(4,1,3)
plt.plot(dt, D)
plt.xlabel('dt')
plt.ylabel('D')

plt.subplot(4,1,4)
plt.plot(dt, PID)
plt.xlabel('dt')
plt.ylabel('PID')

plt.show()
'''

plt.figure()
plt.plot(dt, velocity)  # x=t, y=velocity 로 하는 비선형 그래프를 그림.
plt.show()
'''