from numpy import *
from robot import kinematics
import sys
import threading as t
import time

class Move(t.Thread):
    #Initialize the threading process
    def __init__(self,name,qo,qf,tf):
        t.Thread.__init__(self)
        self.name = name
        self.qo = qo
        self.qf = qf
        self.tf = tf

    #What happens when the thread is called
    def run(self):
        #t_lock.acquire()
        qdd_p = 4*(self.qf-self.qo)/pow(self.tf,2)
        x = round((pow(qdd_p,2)*pow(self.tf,2)),3)
        y = round((4*qdd_p*(self.qf-self.qo)),3)
        num = sqrt(x-y)
        tb = (0.5*self.tf) - (num)/(2*qdd_p)
        qb_p = self.qo + (0.5*qdd_p*pow(tb,2))

        sec = linspace(0,self.tf,100)
        delay = self.tf/(len(sec))*0.95

        start = time.time()
        #calculating each joint's velocity and acceleration at each instance of time
        for t in sec:
            if t < tb and t >= 0:
                q = self.qo + 0.5*qdd_p*pow(t,2)
                vel = qdd_p*t
                acc = qdd_p
            if t < (self.tf-tb) and t >= (tb):
                q = qb_p + qdd_p*tb*(t-tb)
                vel = t
                acc = 0
            if t <= self.tf and t >= (self.tf-tb):
                q = self.qf - 0.5*qdd_p*pow((t-self.tf),2)
                vel = qdd_p*(t-self.tf)
                acc = qdd_p
            q = round(q,2)

            time.sleep(delay)
        stop = time.time()
        exe_t = stop-start

#Generates a path to be followed by the robot
class trajectory_planner():
    #Initializing the start and end position
    def __init__(self,x1,y1,z1,x2,y2,z2):
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
    #Equation that returns the respective angles of each joint
    def fx(self,qo,qf,tf):
        #This function creates a linear relation using linear 
        #equation to map the start point to the end point      
        qdd,qb = [],[]
        th1,th2,th3,th4,th5 = [],[],[],[],[]
        for i in range(len(qo)):
            qdd_p = 4*(qf[i]-qo[i])/pow(tf,2)
            x = pow(qdd_p,2)*pow(tf,2)
            y = 4*qdd_p*(qf[i]-qo[i])
            num = sqrt((x-y))
            tb = (0.5*tf) - num/(2*qdd_p)
            qb_p = qo[i] + (0.5*qdd_p*pow(tb,2))
            qdd.append(qdd_p),qb.append(qb_p)

        sec = linspace(0,tf,10)       
        for t in sec:
            for j in arange(len(qo)):
                if t < tb and t >= 0:
                    q = qo[j] + 0.5*qdd[j]*pow(t,2)
                if t < (tf-tb) and t >= (tb):
                    q = qb[j] + qdd[j]*tb*(t-tb)
                if t <= tf and t >= (tf-tb):
                    q = qf[j] - 0.5*qdd[j]*pow((t-tf),2)
                q = round(q,1)
                if j == 0:
                    th1.append(q)
                if j == 1:
                    th2.append(q)
                if j == 2:
                    th3.append(q)
                if j == 3:
                    th4.append(q)
                if j == 4:
                    th5.append(q)
        print(th1),print(th2),print(th3),print(th4),print(th5)
        return th1,th2,th3,th4,th5

    #Get the first angle and the last angle for the joints
    def getAngles(self,H0_6):
        qo,qf = [],[]
        #Returns the joint angles
        th1,th2,th3 = my_robot.ikine(self.x1,self.y1,self.z1) #Returns theta1,theta2,theta3
        H0_4 = my_robot.pos(th1,th2,th3) #Returns the last two angles theta1 and theta2
        H4_6 = dot(linalg.inv(H0_4),H0_6)
        th4 = round(rad2deg(arcsin(H4_6[0,2])),2)
        th5 = round(rad2deg(arcsin(H4_6[2,0])),2)
        qo.append(th1),qo.append(th2),qo.append(th3),qo.append(th4),qo.append(th5)
        
        th1,th2,th3 = my_robot.ikine(self.x2,self.y2,self.z2) #Returns theta1,theta2,theta3
        H0_4 = my_robot.pos(th1,th2,th3) #Returns the last two angles theta1 and theta2
        H4_6 = dot(linalg.inv(H0_4),H0_6)
        th4 = round(rad2deg(arcsin(H4_6[0,2])),2)
        th5 = round(rad2deg(arcsin(H4_6[2,0])),2)
        qf.append(th1),qf.append(th2),qf.append(th3),qf.append(th4),qf.append(th5)
        return qo,qf

def moveJoint(qo,qf,action):
    process = []
    if qf[0]-qo[0] != 0:
        joint1 = Move('First',qo[0],qf[0],tf)
        joint1.start(),process.append(joint1)
        print("Joint1 active")
    if qf[1]-qo[1] != 0:
        joint2 = Move('Second',qo[1],qf[1],tf)
        joint2.start(),process.append(joint2)
        print("Joint2 active")
    if qf[2]-qo[2] != 0:
        joint3 = Move('Third',qo[2],qf[2],tf)
        joint3.start(),process.append(joint3)
        print("joint3 active")
    if qf[3]-qo[3] != 0:
        joint4 = Move('Fourth',qo[3],qf[3],tf)
        joint4.start(),process.append(joint4)
        print("joint4 active")
    if qf[4]-qo[4] != 0:
        joint5 = Move('Fifth',qo[4],qf[4],tf)
        joint5.start(),process.append(joint5)
        print("Joint5 active")
    check_target(process,action)   

def check_target(process,action): #0 > gripper, 1 > Grab
    for p in process:
        p.join() #Wait for all the threads to finish
    print("****Target reached****")
    print(action)
 
my_robot = kinematics(a1=2,a2=2,a3=6,a4=3) #Arguments are link1,link2,link3,link4
def goTo(HOME_POS,TARGET,tf,H0_6,action):
    main = trajectory_planner(HOME_POS[0],HOME_POS[1],HOME_POS[2],
                      TARGET[0],TARGET[1],TARGET[2]) #arguments are the intial position and the endeffector position

    qo,qf = main.getAngles(H0_6)
    print(qo)
    print(qf) 
    moveJoint(qo,qf,action)

if __name__ == '__main__':
    
    HOME_POS = [11,0,2]
    TARGET = [9,0,0]
    tf = 5
    action = 'grab' #'open'
    H0_6 = [ #Orientation of the end effector
        [0,1,0,0],
        [0,0,1,0],
        [1,0,0,0],
        [0,0,0,1]
    ]

    goTo(HOME_POS,
        TARGET,
        tf,
        H0_6,
        action,
        )
