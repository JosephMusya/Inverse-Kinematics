from trajectory import goTo
import random as rd
while True:
    #Generate random  positions
    x = rd.randint(4,7) 
    y = rd.randint(-1,1)
    z = rd.randint(8,9)
    goTo(HOME_POS=[11,0,8],TARGET=[x,y,z])
    print("_____________________________")