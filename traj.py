#import matplotlib.pyplot as plt
from numpy import *
from robot import kinematics
import sys
import threading as t
x1,y1,z1 = 11,0,2
x2,y2,z2 = 8,4,1
# eqn => y = mx + c
#        c = y-mx
# circle => dx*2 + dy*2 + dz*2 = r*2

def fx(xlist,m,c):
    xcod,ycod,zcod = [],[],[]
    for i in range(len(xlist)):
        ycod.append(m*xlist[i] + c)
        xcod.append(xlist[i])
        zcod.append(2)       
    return xcod,ycod,zcod

#segment the start to end point into 15 points
xlist = linspace(x1,x2,15)
m  = (y2-y1)/(x2-x1)      
c = y2 - m*x2

xlist,ylist,zlist = fx(xlist,m,c)

my_robot = kinematics(2,2,6,3) #Arguments are link1,link2,link3,link4
H0_6 = [ #Orientation of the end effector
    [0,1,0,0],
    [0,0,1,0],
    [1,0,0,0],
    [0,0,0,1]
]

def getAngles(xlist):
    t1,t2,t3,t4,t5 = [],[],[],[],[]
    for i in range(len(xlist)):
        x = xlist[i]
        y = ylist[i]
        z = zlist[i]
        th1,th2,th3 = my_robot.ikine(x,y,z) #Returns theta1,theta2,theta3
        H0_4 = my_robot.pos(th1,th2,th3)
        H4_6 = dot(linalg.inv(H0_4),H0_6)
        th4 = round(rad2deg(arcsin(H4_6[0,2])),2)
        th5 = round(rad2deg(arcsin(H4_6[2,0])),2)
        t1.append(th1),t2.append(th2),t3.append(th3)
        t4.append(th4),t5.append(th5)
    return t1,t2,t3,t4,t5

t1,t2,t3,t4,t5 = getAngles(xlist)
print("Th1:",t1),print("Th2:",t2),print("Th3:",t3),print("Th4:",t4),print("Th5:",t5),

sys.exit()
