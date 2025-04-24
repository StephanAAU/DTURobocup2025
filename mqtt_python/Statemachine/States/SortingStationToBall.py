import numpy as np
import time

from Libraries.RobotMover import robotmover

from orighelpers.sedge import edge
from orighelpers.sir import ir
from orighelpers.uservice import service

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars

from States.BallSorter import BallSorter
from Libraries.saruco import aruco, search_id_list, search_id
from States.Goalstate import Goalstate
from orighelpers.spose import pose

class SortingStationToBall(State):
    def __init__(self,systemvars:systemVars):
        self.in_position = False

        #trans1 = Transition(BallSorter, lambda: self.in_position, []) # initialize with next state, condition as a lambda function and instruction
        trans1 = Transition(BallSorter, lambda: self.in_position, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1]
        super().__init__(transitions, systemvars)

    def execute(self) -> Transition:
        print("Executing SortingStationToBall")
        from States.LineFollow import LineFollow
        linefollow = LineFollow(self.systemvars)

        state = 1
        while True:
            if (state == 1):
                print("state 1")
                # First move back abit
                robotmover.driveDist(-0.3, speed=0.2)
                robotmover.timer.join()
                service.send("robobot/cmd/T0/servo",  f"1 {-700} {200}")

                # Then turn 666 deg left to face the line
                for _ in range(5):

                    found, ids,coords,rots = search_id_list([16,17])
                    if found:
                        print("found id to turn")
                        break
                    time.sleep(0.2)
                pose.tripBreset()
                if found:
                    print(ids[0])
                    print(rots)
                    if (rots[0][2][0] > 2):
                        rots[0][1][0] = -rots[0][1][0]
                    if ids[0] == 16:
                        #rotation = rots[0][1][0] + (np.pi*1/4 - np.pi*1/10)
                        rotation = -rots[0][1][0] + (np.pi*1/4 - np.pi*1/12)
                    if ids[0] == 17:
                        rotation =  (np.pi*3/4 - np.pi*1/12) - rots[0][1][0]
                else:
                    print("notfound")
                    # If we dont find the marker, just turn 90 deg
                    rotation = np.pi/2

                print(rotation)
                robotmover.turn(rotation, speed=0, turnrate=1)
                robotmover.timer.join()

                if pose.tripBh > rotation + 0.1 or pose.tripBh < rotation-0.1:
                    robotmover.turn(pose.tripBh - rotation)
                    robotmover.timer.join()

                state = 2
                
            elif (state == 2):
               
                # Now move forward to the line while looking for it
                robotmover.driveDist(3, speed=0.15)

                state = 3
                time.sleep(2.5)
                print("done")
          
            elif (state == 3):
                #edge.printn()
                time.sleep(0.01)
                if (edge.lineValid):
                    # If the line is valid, stop moving
                    robotmover.stopmove()
                    

                    # Turn to the right again abit to get in position.
                    robotmover.turn(-np.pi/2.5, speed=0.1, turnrate=2)
                    robotmover.timer.join()

                    state  = 4

### From this point it might be better to have in the line follow statemachine but atm its here.
            elif (state == 4):
                # Now follow line untill crossing.
                
                #edge.lineControl(0.2, 0)
                linefollow.ref_pos = 0
                linefollow.cross_threshold_pos = 0.5

                edge.setslow()
                linefollow.start_following(0.2)

                state = 5

            elif (state == 5):
                # Need the delay to allow the PID controller to work deterministically
                time.sleep(0.05)
                linefollow.check_for_crossing()

                if (linefollow.crossing_counter >= 2):
                    # If the line is crossed, stop following
                    linefollow.stop_following()
                    robotmover.stopmove()

                    time.sleep(1)

                    state = 6

            elif (state == 6):
                service.send("robobot/cmd/T0/servo","1 -200 0")
                # Turn right to face the ball cup
                robotmover.turn(-np.pi/2, speed=0, turnrate=2)
                robotmover.timer.join()

                time.sleep(0.3)

                state = 7

            elif (state == 7):
                # Follow line very slowly untill the distance sensor triggers
                # Offset to the side to ensure contact
                #edge.lineControl(0.05, -0.8)
                #linefollow.ref_pos = -0.95
                #linefollow.start_following(0.2)
                linefollow.ref_pos = -1.05
                linefollow.start_following(0.3)

                state = 8

            elif (state == 8):
                time.sleep(0.05)

                if (ir.getdist()[1] < 0.1):
                    # If the distance sensor triggers, stop following
                    edge.lineControl(0, 0)
                    robotmover.stopmove()

                    time.sleep(0.3)

                    state = 9

            elif (state == 9):
                # Now move back and then position to the right of the cup
                robotmover.driveDist(-0.2, speed=0.2)
                robotmover.timer.join()
                time.sleep(0.3)
                service.send("robobot/cmd/T0/servo","1 420 0")
                #robotmover.move_around_target(x=0.3,y=0,radius=0.5,rads_driven=np.pi/2,endpose=-np.pi/2)
                robotmover.move_around_target(x=0.3,y=0,radius=0.35,rads_driven=-np.pi/2,endpose=np.pi/2)
#                robotmover.driveDist(0.4, speed=0.3)
                robotmover.driveDist(0.8, speed=0.3)
                robotmover.timer.join()
                robotmover.turn(-np.pi,turnrate=1)
                robotmover.timer.join()
#                robotmover.driveDist(-0.4,speed = 0.3)
#                robotmover.timer.join()
                state = 10

            elif (state == 10):
                print("state 10")
                # Now we are ready to detect
                self.in_position = True

            trans_out = self.checkIfDone()
            if trans_out != None:
                return trans_out
