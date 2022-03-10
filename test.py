'''
Not that this code has been developed for testing purpose only.
The structure of the robot is defined in the trajectory library
in kinematics section
'''

from trajectory import goTo

#Defining the start and the end position of the robot arm
HOME_POS = [11,0,2]
TARGET = [3,7,0]
t = 5
H0_6 = [ #Orientation of the end effector
    [0,1,0,0],
    [0,0,1,0],
    [1,0,0,0],
    [0,0,0,1]
]
#Calling the function to execute the motion
goTo(HOME_POS,
     TARGET,
     t,
     H0_6)
