import cv2 as cv
import pickle
import numpy as np
import sys
from orighelpers.spose import pose
import threading
import math
from orighelpers.uservice import service
import time
# from States.DriveToCube import move_robot_to_coords, search_id
from Libraries.saruco import imageAnalysis

from Libraries.saruco import aruco
from Libraries.saruco import search_id



def calculate_center(ids, coords, rotations):
    #calculate the center of the posemap
    
    coords = np.array(coords)
    extra_rotation = [0,0]
    
    if ids == [0]:
        return False, 0, 0


    for i in range(len(ids)):
        if rotations[i][2]  > 2:
            rotations[i][1] = -rotations[i][1]
            
        theta = float(rotations[i][1])
        if ids[i]%2 == 0: # even ids have the center to the right i.e negative y direction
            coords[i][0] = coords[i][0] + np.cos(theta) * 0.1
            coords[i][1] = coords[i][1] + np.sin(theta) * 0.1
            extra_rotation[0] += (math.pi/4 - theta)
            extra_rotation[1] -= theta
        else: # odd ids have the center to the left i.e positive y direction
            
            coords[i][0] = coords[i][0] + np.cos(theta) * 0.1
            coords[i][1] = coords[i][1] + np.sin(theta) * 0.1
            extra_rotation[0] += (-math.pi/4 - theta)
            extra_rotation[1] -= theta
            
    mean_coords = np.mean(coords[:, :3], axis=0)
    extra_rotation[0] = extra_rotation[0] / len(ids) #extra_rotationcorner
    extra_rotation[1] = extra_rotation[1] / len(ids) #extraa_rotationWall


    print(f"calc center: {mean_coords} \t xtra rot: {extra_rotation}")

    return True, mean_coords, extra_rotation[0], extra_rotation[1] #extra rotation is to been clean on an area

class RobotMovement():
    def __init__(self):
        self.timer = None
        self.moving = False

    def stopmove(self):

        service.send(service.topicCmd + "ti/rc", "0 0")
        self.moving = False
        if self.timer != None:
            self.timer.cancel()


    def stopturn(self):
  
        service.send(service.topicCmd + "ti/rc", "0 0")

        self.timer.cancel()


    def driveDist(self,meters,speed=0.2):
        self.moving = True
    
        service.send(service.topicCmd + "ti/rc", f"{np.sign(meters) *speed} 0")
        
        self.timer = threading.Timer(abs(meters)/(speed), self.stopmove)
        self.timer.start()


    def turn(self,rads,speed=0,turnrate=0.5):
        self.moving = True
    
        service.send(service.topicCmd + "ti/rc", f"{speed} {np.sign(rads)*turnrate}") 
        
        self.timer = threading.Timer(abs(rads)/(turnrate), self.stopturn)
        self.timer.start()


    def move_to_target(self,x,y,distfrom,v=0.2,w=0.5, movespeed_in_turn = 0):
        self.moving = True

        pose.tripBreset()
        if x == 0:
            angle = math.pi/2
        elif x < 0:
            angle = -math.atan(y/abs(x))
        else:
            angle = math.atan(y/x)
     
        dist    = math.sqrt(y**2 + x**2)-distfrom

        if self.timer != None: self.timer.cancel()
        
        self.turn(angle,speed = movespeed_in_turn, turnrate=w)
        r       = v/w
        dist    = dist - 2*r*np.sin(angle/2) #subtract the distance traveled while turning
        self.timer.join()
        time.sleep(0.1)
        self.driveDist(dist,v)
        self.timer.join()


    def move_around_target(self,x,y,radius,rads_driven, endpose=None,speed = 0.2):
        print(x)
        print(y)
        print(radius)
        print(rads_driven)
        print(np.sign(rads_driven))
        sign = np.sign(rads_driven)
        print(f"move around target, sign: {sign}")
        if endpose == None:
            endpose =  0

        self.move_to_target(x,y,radius,v = speed)
        self.timer.join()
        time.sleep(0.1)
        #Be perpendicular to target
        full_dist = 2 * math.pi * radius * (abs(rads_driven)/(math.pi*2))
        print(f"full dist: {full_dist}")

        if full_dist == 0:
            print("full dist = 0, returning")
            return
        self.turn(-(sign) * math.pi/2, 0, 1)

        self.timer.join()

        # move around the target
        
        turnrate = abs(rads_driven)/(full_dist/speed)

        self.turn((rads_driven), speed, turnrate)
        self.timer.join()
        
        self.turn((sign*math.pi/2) + endpose, 0, 1)
        self.timer.join()
       
       
    def move_to_unload(self,current, destination, centercords, extra_travel,  radius ,ForcedDirection = "left"): # assumes 1 meter away esay to fix if needed 
        print("current,destination: ",current, destination)
        destmap = {(10,13):0, 
                   (12,13):math.pi*1/4,
                   (12,15):math.pi*2/4,
                   (14,15):math.pi*3/4,
                   (14,17):math.pi*4/4,
                   (16,17):math.pi*5/4,
                   (11,16):math.pi*6/4,
                   (10,11):math.pi*7/4}
        try:
     
            from_angle =  destmap[current]
            to_angle = destmap[destination]       
        except:
            return -1
        if ForcedDirection == "left":
            
            if from_angle < to_angle:
                driveangle = -((math.pi*2 - (to_angle - from_angle)))
            
            else:
                driveangle = (((to_angle - from_angle)))
        else:  # 'right'
            
            if from_angle < to_angle:
                driveangle = (((to_angle - from_angle)))
            
            else:
                print(f"driveangle will be : {(to_angle - from_angle)}")
                driveangle = ((math.pi*2 - (to_angle - from_angle))) % (math.pi*2)
            
            
      
   
        print("driveangle: ",driveangle)
 

        if to_angle==from_angle:
            print("to angle == from angle, returning")
            return

        print("moving around target")
        
        print(f"radius : {radius}")
        print(f"driveangle + extra_travel : {driveangle + extra_travel}")
        print(f"centercords : {centercords}")
        print(f"centercords[0] : {centercords[0]}")
        print(f"centercords[1] : {centercords[1]}")
        self.move_around_target(x = centercords[0],y=centercords[1], radius = radius ,rads_driven = driveangle + extra_travel)
        


robotmover = RobotMovement()
