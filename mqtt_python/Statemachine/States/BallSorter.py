import cv2 as cv
import numpy as np

from Statehelpers.Stateinterface    import State
from Statehelpers.Transition        import Transition
from Statehelpers.systemVars        import systemVars
#import needed instructions
from States.Goalstate       import Goalstate
from orighelpers.scam       import cam
from orighelpers.uservice   import service
from orighelpers.spose      import pose

from Libraries.RobotMover       import robotmover, calculate_center
from Libraries.ball_detection   import ball_detector
from Libraries.saruco           import aruco, search_id_list, search_id
from orighelpers.sedge          import edge
#from States.DriveToCube import search_id_list


import time

SERVO_SPEED     = 200
SERVO_POS_GND   = 450
SERVO_POS_INSP  = 50
SERVO_POS_REST  = 150

class BallSorter(State):
    def __init__(self,systemvars:systemVars):
        self.internal_state = 1
        from States.LineFollow import LineFollow
        trans1 = Transition(LineFollow, lambda: self.internal_state == 8, [])
      #  trans1 = Transition(Goalstate, lambda: self.internal_state == 8, [])

        transitions = [trans1]
        super().__init__(transitions, systemvars)


    def execute(self) -> Transition:
        print("Executing Ball Detection state")
        return self.ball_sorter_statemachine()


    def ball_sorter_statemachine(self):

        service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_REST} {SERVO_SPEED}")
        #service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_INSP} {SERVO_SPEED}")
        print("Starting ball sorter state machine")
        ball_detector.config_normal_color()
        while True:
            #print(f"curr state: {self.internal_state}")
            # Poll a frame and process,
            # show frame if not running code directly on the robot
            ball_detector.clear()
            if (cam.useCam):
                ball_detector.capture_frame_analyze()
                key = cv.waitKey(1)
                if key > 0:
                    break

            else:
                ball_detector.capture_frame_analyze()

            # Internal state machine
            if (self.internal_state == 1):
            ## State that looks for the ball


                if len(ball_detector.ball_color) > 0:

                    target_x = ball_detector.ball_alt_coordinates[0][0]
                    target_y = ball_detector.ball_alt_coordinates[0][1]

                    print(f"statemachine, move to: {target_x}, {target_y}")
                    robotmover.move_to_target(target_x, target_y, distfrom=0.45)

                    while (robotmover.moving == True):
                        time.sleep(0.1)

                    self.internal_state = 2

                else:
                    # Since cube has not been found, lets turn abit and look again
                    robotmover.turn(np.deg2rad(45), speed=0, turnrate=1)
                    robotmover.timer.join()
                    time.sleep(0.1)

            elif (self.internal_state == 2):
            ## State where we are closer to the ball and now try to grab it
                if len(ball_detector.ball_color) > 0:
                    service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_GND} {SERVO_SPEED}")
                    target_x = ball_detector.ball_alt_coordinates[0][0]
                    target_y = ball_detector.ball_alt_coordinates[0][1]

                    print(f"statemachine, move to: {target_x}, {target_y}")
                    #robotmover.move_to_target(target_x, target_y, distfrom=-0.05,v = 0.1)
                   
                    robotmover.move_to_target(target_x, target_y, distfrom=0 ,v = 0.1)

                    while (robotmover.moving == True):
                        time.sleep(0.1)

                    self.internal_state = 3

                else:
            ## If we cannot detect the ball drive backwards and go back
                    print("Ball not found, going back")
                    pose.tripBreset()
                    service.send("robobot/cmd/ti/rc","-0.3   0.0")
                    self.internal_state = 21


            elif (self.internal_state == 21):
                if (pose.tripB < -0.3):
                        service.send("robobot/cmd/ti/rc","0.0   0.0")
                        self.internal_state = 1


            elif (self.internal_state == 3):
                ## We assume we have grabbed the ball, first we lift it up
                service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_INSP} {SERVO_SPEED}")
                # Also we allow the area to be smaller
                ball_detector.config_grabber_color()
                time.sleep(1)
                self.internal_state = 31

            elif (self.internal_state == 31):
                ## Now we inspect and if its there go    to luggage area state
                # if now, set down the grabber and try again.

                if len(ball_detector.ball_color) > 0:
                    service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_REST} {SERVO_SPEED}")
                    time.sleep(0.5)
                    self.internal_state = 4
                    #self.internal_state = 32
                    print("ball in grabber")
                    robotmover.driveDist(-0.1)
                    robotmover.timer.join()
                else:
                    service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_GND} {SERVO_SPEED}")
                    time.sleep(0.5)
                    service.send("robobot/cmd/ti/rc","-0.3   0.0")
                    self.internal_state = 21
                    #self.internal_state = 32
                    print("ball not in grabber")
                    pose.tripBreset()

                ball_detector.config_normal_color()


            elif (self.internal_state == 32):
            ## temporary state
                print("temporary state")
                time.sleep(1)
                service.send("robobot/cmd/ti/rc","0   0.0")
                self.internal_state = 3

            elif (self.internal_state == 4):
                ## Now we look for the luggage area.
                luggage_ids = [10, 11, 12, 13, 14, 15, 16, 17]
                luggage_found = False
                luggage_found, _ids, _coords, rots = search_id_list(luggage_ids)

                # If we dont find the luggage area, we should probably turn and look again.
                if not luggage_found:
                    # Since cube has not been found, lets turn abit and look again
                    robotmover.turn(np.deg2rad(45), speed=0, turnrate=1)
                    robotmover.timer.join()

                elif luggage_found:
                    target_robot_x = _coords[0][0]
                    target_robot_y = _coords[0][1]

                    print(f"Found luggage area direct, {target_robot_x}, {target_robot_y}")
                    succes, center, extra_corner, extra_wall = calculate_center(_ids, _coords, rots)
                    if succes:
                        robotmover.move_to_target(center[0], center[1], distfrom=0.8)

                    while (robotmover.moving == True):
                        time.sleep(0.1)

                    self.internal_state  = 5

            elif (self.internal_state == 5):
                ## Now we are near the luggage area we need to find "C"
                # Keep this state empty and switch to next statemachine.
                #service.send("robobot/cmd/T0/servo",  f"1 0 0")
                found , ids, xyz, rotations = search_id_list([10,11,12,13,14,15,16,17])



                if found:
                    print("Found luggage area, advanced")

                    id1 = min(ids)
                    id2 = max(ids)

                    if id1 == id2:
                        print("only one id found")
                        found = False
                        succes, center, extra_corner, extra_wall = calculate_center(ids, xyz, rotations)
                        robotmover.move_to_target(center[0], center[1], distfrom=1)
                        continue

                    succes, center, extra_corner, extra_wall = calculate_center(ids, xyz, rotations)
                    if succes:
                        if abs(id1 - id2) ==1:
                            extra = extra_corner
                        else:
                            extra = extra_wall
                        print("two ids found")
                        robotmover.move_to_unload(current=(id1,id2),destination=(14,15),centercords=center ,extra_travel = extra ,radius=0.65 , ForcedDirection="right")
                        print("move backwards")
                        robotmover.driveDist(-0.3, speed=0.2)
                        robotmover.timer.join()
                        found1 , ids, xyz, rotations = search_id_list([14,15])

                        succes, center,extra_corner, extra_wall = calculate_center(ids, xyz, rotations)
                        if 14 in ids or 15 in ids:
                            print(center[0], center[1])
                            robotmover.move_to_target(center[0],center[1], distfrom=0.2)
                            service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_GND} {SERVO_SPEED}")
                            robotmover.driveDist(-0.15, speed=0.2)
                            robotmover.timer.join()

                            self.internal_state = 6

                    else:
                        found = False
                        continue

            elif (self.internal_state == 6):
                print("internal state 6")
                for _ in range(5):

                    found, ids,coords,rots = search_id_list([14,15])
                    print(found)
                    if found:
                        break
                    time.sleep(0.2)

                if found:
                    print("inside internal state found")
                    service.send("robobot/cmd/T0/servo",  f"1 {-700} {SERVO_SPEED}")
                    #service.send("robobot/cmd/T0/servo",  f"1 {350} {SERVO_SPEED}")
                    time.sleep(1.5)
                    print(ids[0])
                    print(rots)
                    if ids[0] == 14:
                        rotation = - np.pi/2  - rots[0][1][0]
                    if ids[0] == 15:
                        rotation = -rots[0][1][0] 
                    print(f"rotation: {rotation}")
                    robotmover.turn(rotation, speed=0, turnrate=2)
                    robotmover.timer.join()

                    robotmover.driveDist(-2, speed=0.15)
                    time.sleep(1)
                self.internal_state = 7
                print("going to state 7")


            elif self.internal_state == 7:
                #edge.printn()

                time.sleep(0.01)
                if (edge.lineValid):
                    # If the line is valid, stop moving
                    robotmover.stopmove()

                    # Turn to the right again abit to get in position.
                    robotmover.turn(np.pi*3/4, speed=0, turnrate=1)
                    robotmover.timer.join()
                    self.internal_state = 8

                    print("found line, going to state 8")
                    #from States.LineFollow import LineFollow
                    #return Transition(LineFollow, lambda: self.internal_state == 8, [])

            elif self.internal_state == 8:
                print("ball sorter done")

            trans_out = self.checkIfDone()
            if trans_out != None:
                return trans_out


