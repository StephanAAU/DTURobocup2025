from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars
import random as rand
#import needed instructions


from orighelpers.uservice import service
from orighelpers.spose import pose

#import states
import time
from States.Goalstate import Goalstate
from Libraries.visionMovement import imageAnalysis, move_robot_toward_target, camera_to_robot_coordinates


class GotoTest(State):
    def __init__(self,systemvars:systemVars):
        self.test_done = False

        trans1 = Transition(Goalstate, lambda: self.test_done == True, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1]
        super().__init__(transitions, systemvars)



    def execute(self) -> Transition:
        print("Executing Goto Test State")
        service.send("robobot/cmd/T0/servo",  f"1 200 100") ## Lift arm over gnd

        pose.tripBreset()        
        while True:
            if (move_robot_toward_target(1.5, 0)):
                self.test_done = True

            #pose.printPose()
            time.sleep(0.1)

            trans_out = self.checkIfDone()
            if trans_out != None:
                return trans_out

