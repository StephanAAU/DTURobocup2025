from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars

import random as rand
import time
import numpy as np
#import needed instructions


from orighelpers.sedge import edge
from orighelpers.uservice import service
import cv2 as cv
from orighelpers.scam import cam
from Libraries.RobotMover import robotmover, calculate_center
from Libraries.saruco import imageAnalysis, search_id, search_id_list
#import states


class Goalstate(State):
    def __init__(self,systemvars:systemVars):

        trans = Transition(Goalstate, lambda: False, [])
        super().__init__([trans], systemvars)

    def execute(self) -> Transition:
        print("Executing Goalstate")
        state = 1
        while True:
            if (cam.useCam):
                imageAnalysis(False)
                key = cv.waitKey(10)  # ms
                if key > 0:  # e.g. Esc (key=27) pressed with focus on image
                    break
            else:
                imageAnalysis(False)

            #### State 1
            # Look for cube with a specific ID
            if (state == 1):
                print("goal state 1")
                cube_found, target_robot_x, target_robot_y = search_id(25) # final end goal cube
                print(f"cube found: {cube_found}, target_robot_x: {target_robot_x}, target_robot_y: {target_robot_y}")

                if cube_found == False:
                # Since cube has not been found, lets turn abit and look again
                    print("cube not found")
                    robotmover.turn(np.deg2rad(-45), speed=0,turnrate=1) 
                    robotmover.timer.join()
                    time.sleep(0.2)
                else:
                    state = 2
                    print("cube found")
            elif state == 2:
                # Move to the cube
                robotmover.move_to_target(target_robot_x, target_robot_y, distfrom=0.1)


                return None


        print("Goalstate")
        time.sleep(1)
        service.terminate()
