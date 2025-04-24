from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars

#import needed instructions
import numpy as np
import time
import threading
import random as rand

#import states

from Libraries.saruco import SAruCo
from Libraries.RobotMover import robotmover

from orighelpers.uservice import service
from orighelpers.sedge import edge
from orighelpers.sir import ir
from orighelpers.spose import pose
from Libraries.saruco import imageAnalysis, search_id_list, search_id
from States.DriveToCube import DriveToCube

def stopturn():
   
    service.send("robobot/cmd/T0/servo","2 0 0")
    
def turnout(out = True):
    if out:
        turnspeed = 500
    else:
        turnspeed = -500
    service.send("robobot/cmd/T0/servo", f"2 {turnspeed} 0")
    threading.Timer(1, stopturn).start()

class PullLuggage(State):
    def __init__(self,systemvars:systemVars):
        
        trans1 = Transition(DriveToCube, lambda: systemvars.var1 == 1 or self.systemvars.var2 > 10, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1]
        super().__init__(transitions, systemvars)
        
        self.distvalues = [0]*10
        
        self.stable = False
    def execute(self) -> Transition:
        print("Executing state Pullluggage")
        time.sleep(1)
        state = 0
        c = 0
        service.send("robobot/cmd/T0/servo","1 420 0")
        starttime = time.time_ns()/1000000
        lastfound = 0
        turnout(False)
        mindist = 2


        robotmover.driveDist(0.4, speed=0.2)
        robotmover.timer.join()

        while True:
            time.sleep(0.01)
            if c > 1000000:
                c = 0

            self.systemvars.var2 = time.time()/1000000 - starttime


            if state == 0:


                if not edge.lineValid:
                    print("line not valid")
                    state = 0.2

                    continue
                else:
                    state = 0.1

            if state == 0.1:
                if (time.time_ns() - lastfound)/1000000 > 1000:
                    print("starting move agin-----")

                    edge.setslow()
                    edge.lineControl(0.12, 0.0)

                foundcube, _ , xyz , rotations=  search_id_list([20,53,5])

                if foundcube: # if the train is exactly in front of us, stop driving to avoind wrong distance

                    if abs(xyz[0][1]) < 0.2:
                        print("found cube and setting time-------------------")
                        edge.lineControl(0.0,0.0)
                        robotmover.stopmove()
                        lastfound = time.time_ns()

                if ir.getdist()[1] < 0.5 : # if the distance is less than 0.5m and the cube has not been found for 2 seconds


                    edge.lineControl(0.0,0.0)
                    while abs(ir.getdist()[1] - 0.5) > 0.01:
                        print(ir.getdist()[1])
                        if ir.getdist()[1] - 0.5 < 0.2:
                            robotmover.driveDist(ir.getdist()[1] - 0.5,speed=0.08)
                            robotmover.timer.join()
                        else:
                            state = 0.1


                    state = 1
                    edge.lineControl(0.0,0.0)
                    robotmover.stopmove()
                    robotmover.turn(np.pi/7,0,1)
                    robotmover.timer.join()
                    time.sleep(0.3)



            elif state == 0.2:
                robotmover.turn(np.pi/2,0,1)
                robotmover.timer.join()
                robotmover.driveDist(ir.getdist()[1] - 0.3,speed=0.15)
                robotmover.timer.join()
                service.send("robobot/cmd/ti/rc","-0.12 0.0 ")
                while not edge.lineValid:
                    time.sleep(0.05)
                    pass

                robotmover.stopmove()
                robotmover.turn(-np.pi/2,0,1)
                robotmover.timer.join()
                state = 0.1


            elif state == 1:
                print("state1")

                lastfound = time.time_ns()
                while True:
                    foundcube, _,_,_ =  search_id_list([20,53,5])

                    #print(foundcube)
                    calced_time = (time.time_ns() - lastfound)/1000000
                    print(f"calc time: {calced_time}")
                    if foundcube:
                        lastfound = time.time_ns()

#                    elif (time.time_ns() - lastfound)/1000000 > 2000:
                    elif calced_time > 2000:
                        break

                robotmover.turn(np.pi/2-np.pi/7,0.5,1)

                robotmover.timer.join()
                robotmover.driveDist(0.1)
                robotmover.timer.join()
                turnout()
                startstate2time = time.time_ns()/1000000

                pose.tripBreset()
                state = 2

            elif state == 2:
                print("state 2")

                

                
#                if (time.time_ns()/1000000 - startstate2time) > 1200:
                print(f"pose.tripb: {pose.tripB}")
                if pose.tripB < -1.5:
                    ir.wallControl(0.0,0.105)
                    robotmover.stopmove()
                else:
                    ir.wallControl(0.2,0.105)


                if (ir.getdist()[0] < 0.09 and (time.time_ns()/1000000 - startstate2time) > 2500):

                    print("stopping")
                    
                    ir.wallControl(0,0)
                    robotmover.stopmove()
                    time.sleep(5)

                    
                    
                    turnout(False)
                    state = 3

            elif state == 3:
                pose.tripBreset()

                robotmover.turn(rads=(np.pi/2), speed=0.2, turnrate=0.5)
                robotmover.timer.join()
                robotmover.turn(rads=(np.pi/2), speed=0.2, turnrate=1)
                robotmover.timer.join()
                

                robotmover.turn(rads=np.pi*1/3, speed=0 , turnrate=1)
                robotmover.timer.join()



                state = 4

            elif state == 4:
                self.systemvars.var1 = 1
                print(f"mindist value: {mindist}------------------")

            trans_out = self.checkIfDone()
            if trans_out != None:
                return trans_out
