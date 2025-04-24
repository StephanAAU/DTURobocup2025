import numpy as np
import time

#import needed instructions
from orighelpers.spose import pose
from orighelpers.uservice import service

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars

from Libraries.saruco import imageAnalysis
from Libraries.saruco import aruco
from Libraries.saruco import search_id
#import states
from States.Goalstate import Goalstate
#from States.DriveToCube import search_id_list
from Libraries.RobotMover import robotmover
# Configuration parameters
   # seconds between QR scans

## Quick and dirty copy paste


class NavigationSystem(State):
    def __init__(self,systemvars:systemVars):
        self.cube_delivered = False

        trans1 = Transition(Goalstate, lambda: self.cube_delivered == True, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1]
        super().__init__(transitions, systemvars)
        
    def execute(self) -> Transition:
        print("Executing navigation")
        #turndegrees(-360)
        #time.sleep(3)
        
        infront_of_cube_13, robot_x, robot_y = search_id(13)
        
        if infront_of_cube_13:
            print("from 13") 
            
            robotmover.move_around_target(robot_x, robot_y, 0.6, np.pi/6)
        else:
            print("from 10")
            robotmover.move_around_target(robot_x, robot_y, 0.6, np.pi/2)
        
        print("Searching Destination", 17)
        cube_found, target_robot_x, target_robot_y = search_id(17)
        print(f"cube found: {cube_found}, target_robot_x: {target_robot_x}, target_robot_y: {target_robot_y}")
        if (cube_found == True):
            print("Found")
            robotmover.move_to_target(target_robot_x, target_robot_y, distfrom=.4, w=.5)

    
        trans_out = self.checkIfDone()
        if trans_out != None:
            return trans_out

