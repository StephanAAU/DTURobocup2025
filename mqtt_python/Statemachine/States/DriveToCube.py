
import cv2 as cv
import numpy as np
from threading import Timer
import time
import random as rand
#import needed instructions

from Libraries.saruco import imageAnalysis
from Libraries.saruco import aruco
from Libraries.RobotMover import robotmover,  calculate_center

from orighelpers.uservice import service
from orighelpers.spose import pose
from orighelpers.scam import cam
from orighelpers.sir import ir

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars
from Libraries.saruco import search_id, search_id_list
from States.Navigation import NavigationSystem
from States.SortingStationToBall import SortingStationToBall

SERVO_SPEED     = 100
#SERVO_POS_GND   = 445
SERVO_POS_GND   = 450
SERVO_POS_INSP  = 50
SERVO_POS_REST  = 340
DRIVE_SPEED    = 0.1
TURN_SPEED     = 0.1
#SEARCH_BLOCK_ID = 20
SEARCH_BLOCK_ID = 53
TURN_DEGREES = -45


def aruco_state_machine():
    print("aruco state machine")
    aruco_state = 0
    service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_REST} 300")
    service.send("robobot/cmd/ti/rc",      "0   0")
    time.sleep(0.5)

    cube_search_radius = np.deg2rad(30)
    accumulated_cube_search_radius = 0


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

      if (aruco_state == 0):
        cube_found, target_robot_x, target_robot_y = search_id(SEARCH_BLOCK_ID)

        if cube_found == False:
          robotmover.move_around_target(x = 0.7, y=0, radius=0.7, rads_driven=cube_search_radius)
          accumulated_cube_search_radius += cube_search_radius

          if (abs(accumulated_cube_search_radius) > 1.3):
            cube_search_radius = -cube_search_radius

          print(f"accumulated_cube_search_radius: {accumulated_cube_search_radius}, new cube_search_radius: {cube_search_radius}")
        else:
          aruco_state = 1


      elif (aruco_state == 1):
        print("aruco state 1")
        cube_found, target_robot_x, target_robot_y = search_id(SEARCH_BLOCK_ID)
        print(f"cube found: {cube_found}, target_robot_x: {target_robot_x}, target_robot_y: {target_robot_y}")
      
        if cube_found == False:
          # Since cube has not been found, lets turn abit and look again
          print("cube not found")
          robotmover.turn(np.deg2rad(TURN_DEGREES), speed=0, turnrate=1) 
          robotmover.timer.join()
          
          #service.send("robobot/cmd/ti/rc","0   0")
        else:
          aruco_state = 2
          print("cube found")
          service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_GND} 300")
          pose.tripBreset()
          #time.sleep(2)
          #pass

        #service.send("robobot/cmd/T0/servo","1 200 200")

      #### State 2
      elif (aruco_state == 2):
        print("aruco state 2")
        robotmover.move_to_target(target_robot_x, target_robot_y, distfrom=0.5)
        while (robotmover.moving == True):
          time.sleep(0.1)

        aruco_state = 3
        # if target_robot_x * 0.6 < 0.4:
        #   aruco_state = 3
        #   cube_found = False

        # else:
        #   aruco_state = 1
        #   cube_found = False


      #### State 3
      elif (aruco_state == 3):
        print("aruco state 3")
        # Now the cube is at the edge of vision (NOT VERIFIED... JUST AN EXAMPLE)
        #move_robot_toward_target(target_robot_x, target_robot_y)

        cube_found, target_robot_x, target_robot_y = search_id(SEARCH_BLOCK_ID)
        
        if (cube_found == True):
            print(f"cube found: {cube_found}, target_robot_x: {target_robot_x}, target_robot_y: {target_robot_y}")
            robotmover.move_to_target(target_robot_x, target_robot_y, distfrom=0, v=.1)

            # distance_to_wall = ir.getdist()[1]
            # robotmover.driveDist(distance_to_wall-0.15, speed=0.2) #drive towards into the wall to make sure the cube is on
            # robotmover.timer.join()
            while (robotmover.moving == True):
              time.sleep(0.1)

            aruco_state = 4

        else:
          # If we dont find the cube, we should probably turn and look again.
          # Since cube has not been found, lets turn abit and look again
          # print("cube not found")
          # robotmover.turn(np.deg2rad(TURN_DEGREES), speed=0,turnrate=1)
          # robotmover.timer.join()
          # time.sleep(0.2)
          # cube_found, target_robot_x, target_robot_y = search_id(SEARCH_BLOCK_ID)
          #service.send("robobot/cmd/ti/rc","0   0")
          #aruco_state = 1
          print("cube not found")
          pose.tripBreset()
          service.send("robobot/cmd/ti/rc","-0.4   0.0")
          aruco_state = 31

      elif (aruco_state == 31):
        if (pose.tripB < -0.3):
          service.send("robobot/cmd/ti/rc","0.0   0.0")
          aruco_state = 1

      #### State 4
      elif (aruco_state == 4):
        service.send("robobot/cmd/ti/rc","0.0   0.0")
        print("aruco state 4")
        # We now have the cube in the arm, lets lift it up.
        service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_INSP} {SERVO_SPEED}")
        #service.send(service.topicCmd + "ti/servo","0 400 1") # from teensy code: -- \tservo i p v\tset one servo i=index 1..5, pos ~ +/-800 (us), v=velocity 0..4000 (val/s), 0=max, 1=sloooow\r\n"
        #service.send(service.topicCmd + "ti/servo","1 0 1") # from teensy code: -- \tservo i p v\tset one servo i=index 1..5, pos ~ +/-800 (us), v=velocity 0..4000 (val/s), 0=max, 1=sloooow\r\n"
        time.sleep(1)
        aruco_state = 5
        
      #### State 5
      elif (aruco_state == 5):
        print("aruco state 5")
        # Since we know that cube will be out of vision, while picking it up, we can verify that we have lift it with the camera
        service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_INSP} {SERVO_SPEED}")
        for _ in range(5):
          cube_found, target_robot_x, target_robot_y = search_id(SEARCH_BLOCK_ID)
          if cube_found: break
          time.sleep(0.1)

        if cube_found:
          service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_REST} {SERVO_SPEED}")
          robotmover.driveDist(-0.15)
          robotmover.timer.join()
          aruco_state = 7
        else:
           aruco_state = 51

      elif (aruco_state == 51):
        pose.tripBreset()
         
        service.send("robobot/cmd/ti/rc","-0.3   0.0")

        aruco_state = 52

      elif (aruco_state == 52):
        print(f"tripB: {pose.tripB}")
        if (pose.tripB < -0.5):
          service.send("robobot/cmd/ti/rc","0.0   0.0")
          time.sleep(1)
          service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_GND} {SERVO_SPEED}")
          time.sleep(1)
          aruco_state = 1

      #### State 7
      elif (aruco_state == 7):
          print("aruco state 7")
          # We now have the cube confirmed. We should now find the luggage thingy and move towards it.
          # We just look for any of the IDs for a start.
          luggage_ids = [10, 11, 12, 13, 14, 15, 16, 17]
          luggage_found = False
          luggage_found, _ids, _coords, _rots = search_id_list(luggage_ids)

          # If we dont find the luggage area, we should probably turn and look again.
          if not luggage_found:
              # Since cube has not been found, lets turn abit and look again
              print("luggage not found")
              robotmover.turn(np.deg2rad(TURN_DEGREES), speed=0, turnrate=1)
              robotmover.timer.join()
              time.sleep(0.2)

          elif luggage_found:
            target_robot_x = _coords[0][0]
            target_robot_y = _coords[0][1]

            print(f"Found luggage area, {target_robot_x}, {target_robot_y}")

            robotmover.move_to_target(target_robot_x, target_robot_y, distfrom=1.3)

            while (robotmover.moving == True):
              time.sleep(0.1)

            aruco_state    = 8

      #### State 8
      elif (aruco_state == 8):
          print("aruco state 8")
          if (robotmover.moving == False):
            aruco_state = 9
      
      #### State 9

      elif (aruco_state == 9):
          print("aruco test state (9)")

          luggage_ids = [10, 11, 12, 13, 14, 15, 16, 17]
          luggage_found = False
          luggage_found, _ids, _coords,rots = search_id_list(luggage_ids)

          if luggage_found:
             print(f"Found luggage: {_ids}")
             print(f"At: {_coords}")
          
          aruco_state = 10

      elif (aruco_state == 10):
        found , ids, xyz, rotations = search_id_list([10,11,12,13,14,15,16,17])
        
        

        if found:
            print("Found luggage area")
            
            id1 = min(ids)
            id2 = max(ids)

            if id1 == id2:
                print("only one id found")
                succes,center,extra_corner, extra_wall = calculate_center(ids, xyz, rotations)

                robotmover.move_to_target(center[0], center[1], distfrom=1)
                found = False
                continue

            succes,center,extra_corner,extra_wall = calculate_center(ids, xyz, rotations)
            if succes:
              if abs(id1 - id2) ==1:
                 extra = extra_corner
              else:
                 extra = extra_wall
              robotmover.move_to_unload(current=(id1,id2),destination=(16,17),centercords=center ,extra_travel = extra ,radius=0.75 , ForcedDirection="left")
              robotmover.driveDist(-0.2, speed=0.2)
              robotmover.timer.join()
              found1 , ids, xyz, rotations = search_id_list([16,17])
              

              if 16 in ids or 17 in ids:
                  robotmover.move_to_target(xyz[0][0],xyz[0][1], distfrom=0.1)
                  service.send("robobot/cmd/T0/servo",  f"1 {SERVO_POS_GND} {SERVO_SPEED}")

                  break
            else:
                found = False
                continue
          
        
      else:
          print("error, oh shiet")
          imageAnalysis(False)
          service.send("robobot/cmd/ti/rc","0.0 0.0")
          time.sleep(2)



class DriveToCube(State):
    def __init__(self,systemvars:systemVars):
        
        self.trans1 = Transition(SortingStationToBall, lambda: systemvars.var1 > 2, []) # initialize with next state, condition as a lambda function and instruction

        transitions = [self.trans1]
        super().__init__(transitions, systemvars)

    def execute(self) -> Transition:
        print("Executing state 1")

        aruco_state_machine()

        return self.trans1
        
        
        

