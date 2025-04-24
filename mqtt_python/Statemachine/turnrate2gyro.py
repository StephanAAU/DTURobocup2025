from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.instruction import instruction
from Statehelpers.systemVars import systemVars

from orighelpers.uservice import UService
#import needed transition states
from States.AxePasser import AxePasser
from States.DriveToCube import DriveToCube
from States.LineFollow import LineFollow



import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
#from setproctitle import setproctitle
# robot function
from orighelpers.spose import pose
from orighelpers.sir import ir
from orighelpers.srobot import robot
from orighelpers.scam import cam
from orighelpers.sedge import edge
from orighelpers.sgpio import gpio
from orighelpers.scam import cam
from orighelpers.uservice import service
from Libraries.saruco import aruco
from orighelpers.ulog import flog
from orighelpers.simu import imu
from States.PullLuggage import PullLuggage
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



# state = LineFollow(systemV) #initial state

if __name__ == "__main__":
    service.setup('10.197.219.27')

    

    done = False
    w = 0
    lastw = 0
    starttime = t.time()
    while not done:
        t.sleep(0.001)
        flog.write(w)

        if w != lastw:
            service.send(service.topicCmd + "ti/rc", "0.4 {w}".format(w=w))

        if t.time()-starttime < 2:
            lastw = w
            w = 0.5
            
        elif t.time()-starttime < 4:
            lastw = w
            w = 1

        elif t.time()-starttime > 8:
            done = True
         
    service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
    service.terminate()

    
