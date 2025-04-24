from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars
#import needed instructions

from Libraries.RobotMover import robotmover
import time

#import states
from orighelpers.uservice import service
from orighelpers.sedge import edge

from States.Goalstate import Goalstate


class teststate_linevalues(State):
    def __init__(self,systemvars:systemVars):
        trans1 = Transition(None, lambda: False == True, [])
        transitions = [trans1]

        super().__init__(transitions, systemvars)


    def execute(self) -> Transition:
        print("Executing state 2")
        
        while (True):
            edge.printn()
            print(f"{time.time()}: \t lineValid: {edge.lineValid} \t linecnt: {edge.lineValidCnt} \t crossThres: {edge.average}")
            time.sleep(0.1)

        return None
        
