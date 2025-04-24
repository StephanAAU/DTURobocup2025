#import needed instructions
import asyncio
import numpy as np
import time
import random as rand
from collections import deque

from Libraries.RobotMover import robotmover

from orighelpers.sir import ir
from orighelpers.spose import pose
from orighelpers.sedge import edge
from orighelpers.uservice import service

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars

#import states
from States.Goalstate import Goalstate
from States.PullLuggage import PullLuggage


class AxePasser(State):
   
    def __init__(self,systemvars:systemVars):
        self.through = False

        trans1 = Transition(PullLuggage, lambda: self.through, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1]
        super().__init__(transitions, systemvars)

    

    def execute(self) -> Transition:
        print("Executing Axestate")
        c = 0
        distvalues = [1.5]*500

        edge.lineControl(0, 0)  # stop
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
        
        state = 1 #check when to drive
        lastreadT = time.time_ns()
        stopped = False
        service.send("robobot/cmd/T0/servo","1 -700 0")

        robotmover.turn((14*np.pi/32), speed=0, turnrate=2) #turn to the axe
        robotmover.timer.join()

        pose.tripBreset()
        edge.setslow()
        edge.lineControl(0.10, 0.0)

        print("driving 20cm")
        while (pose.tripB < 0.2):
            time.sleep(0.03)
        
        edge.lineControl(0.0, 0.0)
        service.send("robobot/cmd/ti/rc", "0.0 0.0")

        print("going into axe loop")
        time.sleep(0.2)
        notgone = True
        ready = False
        blocked = False
        startblocktime = float('inf')
        while True:
            time.sleep(0.03)
            if c > 100000000:
                c = 0
            

            if state == 1: #check when to drive
                
                if (time.time_ns() - lastreadT )/1e6 >= 20:
                   
                    c += 1
                    distvalues[c % 500] = (ir.getdist()[1])
                    
                   
                    lastreadT = time.time_ns()

                    print("distvalues",distvalues[c % 500])

                if distvalues[c%500] > 0.4 and not stopped:
                    print("stop driving because we see through axe")
                    edge.lineControl(0.0,0.0)
                    service.send("robobot/cmd/ti/rc","0.0 0.0 ")

                elif distvalues[c % 500] > 0.15 and not stopped: # stop at 0.5 meters from the board
                    print("drive slow toward axe")
                    edge.setslow()
                    edge.lineControl(0.08,0.0)                   
                
                elif distvalues[c % 500] <= 0.15 and not stopped :
                    print("stop to get ready to yeet")
                    edge.lineControl(0.0,0.0)
                    service.send("robobot/cmd/ti/rc","0.0 0.0 ")
                    stopped = True
                    


                pastmeasurements = 4
                while state == 1 and stopped:

                    if (time.time_ns() - lastreadT )/1e6 >= 20:
                   
                        c += 1
                        distvalues[c % 500] = (ir.getdist()[1])
                   
                        lastreadT = time.time_ns()

                        print("distvalues",distvalues[c % 500])

                    print(f"while : {distvalues[c % 500]}, {distvalues[c- pastmeasurements % 500]}")

                    time.sleep(0.01)
                    if  c >= 2 and distvalues[c % 500] < 0.25 and distvalues[(c - pastmeasurements) % 500] > 0.4:
                        print(f"if 1 : {distvalues[c % 500]}, {distvalues[c % 500]}")
                        startblocktime = time.time_ns() / 1e6
                        blocked = True


                    elif c >= 2 and distvalues[c % 500] > 0.4 and distvalues[(c - pastmeasurements) % 500] < 0.25 and blocked: 
                        print(f"if 2 : {distvalues[c % 500]}, {distvalues[c % 500]}")
                        radsperSecond = 1.64/(time.time_ns()/ 1e6 - startblocktime)
                        startreadytogo = time.time_ns() / 1e6
                        ready = True
                    
                    if ready and time.time_ns()/1e6 - startreadytogo >=  1 / (radsperSecond  /   0.1)  :

                        print("swap state 2")
                        state = 2
                        

                    

                    
                ## if the axe was blocking 20 measurements = 400 ms ago but no longer is drive 
                #if stopped:
                #    print((time.time_ns() - startofstop)/1000000)
#
                ##print(c >= 25 , distvalues[c % 500] > 1 , distvalues[(c - 27) % 500] < 0.41 , stopped)
                ##if c >= 20 and distvalues[c % 500] > 1 and distvalues[(c - 30) % 500] < 0.41 and stopped:
                #print(distvalues[c % 500] ,distvalues[(c - 41) % 500])
#               # if c >= 30 and distvalues[c % 500] > 0.4 and distvalues[(c - 33) % 500] < 0.3 and stopped:
                #if c >= 57 and distvalues[c % 500] > 0.4 and distvalues[(c - 57) % 500] < 0.25 and stopped:

                       

            elif state == 2: #drive

                pose.tripBreset()
                
                robotmover.driveDist(0.8, speed=0.68)
                robotmover.timer.join()
                
                
                edge.lineControl(0.0,0.0)
                service.send("robobot/cmd/ti/rc","0.0 0.0 ")               
                time.sleep(1)
                self.through = True
                service.send("robobot/cmd/T0/servo","1 -700 0")
            
            
            trans_out = self.checkIfDone()
            if trans_out != None:
                return trans_out
        service.terminate()
