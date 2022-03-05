from numpy import * #Import numerical python numpy (Math Library)
class kinematics():
    def __init__(self,a1,a2,a3,a4):
        self.a1 = a1 #Link 1 length
        self.a2 = a2 #Link 2 Length
        self.a3 = a3 #Link 3 Length
        self.a4 = a4 #Link 4 Length
    # ikine >>> Inverse Kinematics | Takes the desired cordinates, 
    # and returns the desired first angles (theta1,theta2,theta3)
    def ikine(self,x,y,z):
        r3 = sqrt(x*x + y*y)
        r2 = z - self.a1
        r1 = sqrt(r2*r2 + (r3-self.a2)*(r3-self.a2)) 
        phi2 = arctan2(r2,(r3-self.a2))
        phi1 = arccos((self.a4*self.a4 - self.a3*self.a3 - r1*r1)/(-2*self.a3*r1))
        phi3 = arccos((r1*r1 - self.a3*self.a3 - self.a4*self.a4)/(-2*self.a3*self.a4))
        theta1 = round(rad2deg(arctan2(y,x)),2)
        theta2 = round(rad2deg(phi2 - phi1),2)
        theta3 = round(rad2deg(deg2rad(180) - phi3),2)
        return theta1,theta2,theta3
    
    #DHP >>> Denavit Herternberg Parameters | Takes the DH parameter
    # of the robot links and returns an homogenous transformation matrix (HTM)
    def DHP(self,th,alph,d,r):
        th = deg2rad(th)
        alph = deg2rad(alph)
        HTM = [
            [cos(th), -sin(th)*cos(alph), sin(th)*sin(alph), r*cos(th)],
            [sin(th), cos(th)*cos(alph), -cos(th)*sin(alph), r*sin(th)],
            [0, sin(alph), cos(alph), d],
            [0, 0, 0, 1]
        ]
        return matrix(HTM)
    
    #Takes the first three link angles and returns the homogenous tranformation matrix
    def pos(self,th1,th2,th3):
        H0_1 = self.DHP(th1,0,self.a1,0)
        H1_2 = self.DHP(0,90,0,self.a2)
        H2_3 = self.DHP(th2,0,0,self.a3)
        H3_4 = self.DHP(th3+90,90,0,self.a4)
        
        H0_2 = dot(H0_1,H1_2)
        H0_3 = dot(H0_2,H2_3)
        H0_4 = dot(H0_3,H3_4)
        return H0_4

    #Takes the last two link angles and returns the homogenous tranformation matrix
    #Orients the end effector
    def ori(self,th4,th5):
        H4_5 = self.DHP(th4,90,0,0)
        H5_6 = self.DHP(th5,0,0,0)
        H4_6 = dot(H4_5,H5_6)
        return H4_6
    
if __name__ == '__main__':
    try:
        my_robot = kinematics(1,1,3,2) #Arguments are link1,link2,link3,link4
        x,y,z = float(input("X: ")),float(input("Y: ")),float(input("Z: "))
        th1,th2,th3 = my_robot.ikine(x,y,z) #Returns theta1,theta2,theta3
        
        H0_6 = [ #Orientation of the end effector
            [0,1,0,0],
            [0,0,1,0],
            [1,0,0,0],
            [0,0,0,1]
        ]
        
        H0_4 = my_robot.pos(th1,th2,th3) #Gets the Homogenous matrix from 0 to 4
        H4_6 = dot(linalg.inv(H0_4),H0_6) #Gets the Homogenous matrix from 4 to 6
        th4 = round(rad2deg(arcsin(H4_6[0,2])),2) #Theta4
        th5 = round(rad2deg(arcsin(H4_6[2,0])),2) #Theta5
        print("Th1={} Th2={} Th3={} Th4={} Th5={} ".format(th1,th2,th3,th4,th5))
    except Exception as e: 
        print("ERROR ==> {}".format(e))

    #Send Command to servos no angles are none
    
