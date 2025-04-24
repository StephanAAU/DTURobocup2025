from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars
#import needed instructions

from Libraries.RobotMover import robotmover,calculate_center
import time
import math
#import states
from orighelpers.uservice import service
import threading

from orighelpers.uservice import service
from orighelpers.sir import ir
import threading

from States.Goalstate import Goalstate
from Libraries.saruco import aruco, search_id_list, search_id

def stopturn():
    print("stopped")
    service.send("robobot/cmd/T0/servo","2 0 0")

def turnout(out = True):
    if out:
        turnspeed = 100
    else:
        turnspeed = -100
    service.send("robobot/cmd/T0/servo", f"2 {turnspeed} 0")
    threading.Timer(1, stopturn).start()

def stopturn():
    print("stopped")
    service.send("robobot/cmd/T0/servo","2 0 0")

def turnout(out = True):
    if out:
        turnspeed = 100
    else:
        turnspeed = -100
    service.send("robobot/cmd/T0/servo", f"2 {turnspeed} 0")
    threading.Timer(1, stopturn).start()

class teststate3(State):
    def __init__(self,systemvars:systemVars):
        trans1 = Transition(None, lambda: systemvars.timeSinceLastSwap > 2000, [])
        transitions = [trans1]
        super().__init__(transitions, systemvars)


    def execute(self) -> Transition:
        print("Executing state 2")

        service.send("robobot/cmd/T0/servo",  f"1 460 100")
        for i in range(5000):

            print(ir.getdist()[1])
            time.sleep(0.1)

        return None

