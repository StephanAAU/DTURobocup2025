import cv2 as cv
from datetime import *
import numpy as np
import time as t



# robot function
from Libraries.saruco import aruco

from orighelpers.spose import pose
from orighelpers.sir import ir
from orighelpers.srobot import robot
from orighelpers.scam import cam
from orighelpers.sedge import edge
from orighelpers.sgpio import gpio
from orighelpers.scam import cam
from orighelpers.uservice import service

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.instruction import instruction
from Statehelpers.systemVars import systemVars
from States.teststate2 import teststate2
from States.LineFollow import LineFollow
from States.PullLuggage import PullLuggage
from States.AxePasser import AxePasser
from States.SortingStationToBall import SortingStationToBall

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.instruction import instruction
from Statehelpers.systemVars import systemVars
from States.teststate2 import teststate2
from States.LineFollow import LineFollow
from States.PullLuggage import PullLuggage
from States.AxePasser import AxePasser
from States.Navigation import NavigationSystem
from States.SortingStationToBall import SortingStationToBall
from States.BallSorter import BallSorter
from States.DriveToCube import DriveToCube
from States.teststate_linevalues import teststate_linevalues
from States.teststate3 import teststate3
#setproctitle("mqtt-client")

CAMERA_OFFSET_X = 0.5  # Camera's x offset in robot coordinates (meters)
CAMERA_OFFSET_Y = 0.0  # Camera's y offset in robot coordinates (meters)
CAMERA_ORIENTATION = 0.0  # Camera's orientation relative to the robot (radians)

# Target position in camera coordinates
TARGET_CAMERA_X = 1.0  # Target x in camera coordinates (meters)
TARGET_CAMERA_Y = 1.0  # Target y in camera coordinates (meters)

# Robot movement parameters
MOVE_SPEED = 0.1  # Robot's forward speed (meters per step)
TURN_SPEED = 0.1  # Robot's turning speed (radians per step)
POSITION_TOLERANCE = 0.01  # Tolerance for reaching the target (meters)
ANGLE_TOLERANCE = 0.01  # Tolerance for facing the target (radians)



systemV = systemVars() # create system variables

          #initial state
state = NavigationSystem(systemV)       #initial state
state = DriveToCube(systemV)            #initial state
state = teststate_linevalues(systemV)
state = PullLuggage(systemV)            #initial state
#state = teststate3(systemV)
state = AxePasser(systemV)              #initial state

state = SortingStationToBall(systemV)   #initial state
state = BallSorter(systemV)             #initial state
state = teststate2(systemV)             #initial state
state = LineFollow(systemV)             #initial state
if __name__ == "__main__":
    service.setup('10.197.219.27')

## Sometimes tehre is a high spike of CPU consumption in the 
## beginning, so insert a small time wasting delay to allow the service stuff
## to stabilize
time_start = t.time()

print("Starting up... waiting for 2 seconds")
while t.time() - time_start < 2:
    pass

service.send("robobot/cmd/T0/servo","1 99999 0")
print("Starting!")

while True:
    transition = state.execute()
    print("transition", transition)
    print("got new transition")
    if transition == None:
        break 
    
    transition.execute_instructions(systemV)
    print(systemV)
    state = transition.nextstate(systemV) #could be memory leak
    print(state)

    t.sleep(0.1)

service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
service.terminate()

    
