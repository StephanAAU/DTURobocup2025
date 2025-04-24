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
        
        trans1 = Transition(None, lambda: systemvars.var1 == 1, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1]
        super().__init__(transitions, systemvars)
        
        
        self.stable = False
    def execute(self) -> Transition:
        from States.LineFollow import LineFollow
        linefollow = LineFollow(self.systemvars)
        print("Executing state Pullluggage")

        state = 0
        interC = 0
        while True:
            if state == 0:
                for _ in range(5):
                    
                    found, ids,coords,rots = search_id_list([14,15])
                    if found:
                        break
                    time.sleep(0.2)

                if found:
                    print(ids[0])
                    print(rots)
                    if ids[0] == 14:
                        rotation = rots[0][1][0] - (np.pi*1/5)
                    if ids[0] == 15:
                        rotation =  (np.pi/2 - np.pi*1/5) - rots[0][1][0]

                    robotmover.turn(rotation, speed=0, turnrate=2)
                    robotmover.timer.join()

                    robotmover.driveDist(-2, speed=0.15)
                state = 1
            elif state == 1:
                #edge.printn()

                time.sleep(0.01)
                if (edge.lineValid):
                    # If the line is valid, stop moving
                    robotmover.stopmove()
                    

                    # Turn to the right again abit to get in position.
                    robotmover.turn(-np.pi/3.5, speed=0.1, turnrate=2)
                    robotmover.timer.join()
                    state = 2


            
            trans_out = self.checkIfDone()
            if trans_out != None:
                return trans_out
