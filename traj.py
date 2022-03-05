#import matplotlib.pyplot as plt
from numpy import *
from robot import kinematics
import sys
import threading as t
from time import sleep

#defining the HOME position of the robot
HOME_POS = [11,0,2]
TARGET = [10,0,2]

H0_6 = [ #Orientation of the end effector
    [0,1,0,0],
    [0,0,1,0],
    [1,0,0,0],
    [0,0,0,1]
]

class Move(t.Thread):
    #Initialize the threading process
    def __init__(self,name,theta):
        t.Thread.__init__(self)
        self.name = name
        self.theta = theta
    
    #What happens when the thread is called
    def run(self):
        t_lock.acquire()
        for angle in self.theta:
            print("{} {}".format(self.name,angle))
            sleep(.1)
        t_lock.release()
        print("")
class trajectory_planner():
    #Initializing the start and end position
    def __init__(self,x1,y1,z1,x2,y2,z2):
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
                
    def fx(self): 
        #This function creates a linear relation using linear 
        #equation to map the start point to the end point
        xlist = linspace(self.x1,self.x2,5)
        m  = (self.y2-self.y1)/(self.x2-self.x1)      
        c = self.y2 - m*self.x2
        xcod,ycod,zcod = [],[],[]
        for i in range(len(xlist)):
            ycod.append(m*xlist[i] + c)
            xcod.append(xlist[i])
            zcod.append(2)       
        return xcod,ycod,zcod

    def getAngles(self,xlist,ylist,zlist):
        #Returns the joint angles
        t1,t2,t3,t4,t5 = [],[],[],[],[]
        for i in range(len(xlist)):
            x = xlist[i]
            y = ylist[i]
            z = zlist[i]
            th1,th2,th3 = my_robot.ikine(x,y,z) #Returns theta1,theta2,theta3
            H0_4 = my_robot.pos(th1,th2,th3) #Returns the last two angles theta1 and theta2
            H4_6 = dot(linalg.inv(H0_4),H0_6)
            th4 = round(rad2deg(arcsin(H4_6[0,2])),2)
            th5 = round(rad2deg(arcsin(H4_6[2,0])),2)
            t1.append(th1),t2.append(th2),t3.append(th3)
            t4.append(th4),t5.append(th5)
        return t1,t2,t3,t4,t5

    def gripper(self, process,flag=0): #0 > gripper, 1 > Grab
        for p in process:
            p.join() #Wait for all the threads to finish
        sleep(2)
        if flag:
            print("Grab")
        else:
            print("Drop")

my_robot = kinematics(a1=2,a2=2,a3=6,a4=3) #Arguments are link1,link2,link3,link4
def goTo(HOME_POS,TARGET):
    main = trajectory_planner(HOME_POS[0],HOME_POS[1],HOME_POS[2],
                      TARGET[0],TARGET[1],TARGET[2]) #arguments are the intial position and the endeffector position

    #Generate trajectory path
    xlist,ylist,zlist = main.fx()

    #Generated angles from the trajectory
    t1,t2,t3,t4,t5 = main.getAngles(xlist,ylist,zlist)
    #Lock all the threads in join movement occur synchronously
    global t_lock
    t_lock = t.Lock()
    process = []
    joint1 = Move('First',t1)
    joint2 = Move('Second',t2)
    joint3 = Move('Third',t3)
    joint4 = Move('Fourth',t4)
    joint5 = Move('Fifth',t5)

    #Starting the multiprocesses
    joint1.start(),joint2.start(),joint3.start(),joint4.start(),joint5.start() 
    process.append(joint1),process.append(joint2),process.append(joint3),process.append(joint4),process.append(joint5)
    flag = False
    #Check whether the processes are finished then grab or release object
    main.gripper(process,flag)

if __name__ == '__main__':
    goTo(HOME_POS,TARGET)
    sys.exit()
