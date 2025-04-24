#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

#import sys
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
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

# set title of process, so that it is not just called Python
setproctitle("mqtt-client")

############################################################

# Camera configuration
CAMERA_OFFSET_X = 0.5  # Camera's x offset in robot coordinates (meters)
CAMERA_OFFSET_Y = 0.0  # Camera's y offset in robot coordinates (meters)
CAMERA_ORIENTATION = 0.0  # Camera's orientation relative to the robot (radians)

# Target position in camera coordinates
TARGET_CAMERA_X = 1.0  # Target x in camera coordinates (meters)
TARGET_CAMERA_Y = 1.0  # Target y in camera coordinates (meters)

# Robot movement parameters
MOVE_SPEED = 0.1  # Robot's forward speed (meters per step)
TURN_SPEED = 0.1  # Robot's turning speed (radians per step)
POSITION_TOLERANCE = 0.05  # Tolerance for reaching the target (meters)
ANGLE_TOLERANCE = 0.05  # Tolerance for facing the target (radians)


def camera_to_robot_coordinates(camera_x, camera_y):
  """
  Convert camera coordinates to robot coordinates.
  """
  robot_x = CAMERA_OFFSET_X + camera_x * np.cos(CAMERA_ORIENTATION) - camera_y * np.sin(CAMERA_ORIENTATION)
  robot_y = CAMERA_OFFSET_Y + camera_x * np.sin(CAMERA_ORIENTATION) + camera_y * np.cos(CAMERA_ORIENTATION)
  return robot_x, robot_y



def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok:  # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      #edge.paint(img)
      aruco.robot_coords_id_list(img)
      aruco.debug_draw_cam(img)
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass


############################################################

stateTime = datetime.now()


def stateTimePassed():
  return (datetime.now() - stateTime).total_seconds()


############################################################

def loop():
  from orighelpers.ulog import flog
  #aruco.load_camera_calibration(filename="../camera_calibration.pkl")
  state = 0
  images = 0
  ledon = True
  tripTime = datetime.now()
  oldstate = -1
  if not service.args.now:
    print("% Ready, press start button")
    service.send(service.topicCmd + "T0/leds", "16 30 30 0")  # LED 16: yellow - waiting
  # main state machine
  edge.lineControl(0, 0)  # make sure line control is off

  cross_cnt = 0
  while not (service.stop or gpio.stop()):
    if state == 0:  # wait for start signal
      start = gpio.start() or service.args.now
      if start:
        # print("% Starting")
        # service.send(service.topicCmd + "T0/leds", "16 0 0 30")  # blue: running
        # service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
        # # follow line (at 0.25cm/s)
        # edge.crossingLine = False
        # edge.lineControl(0.25, 0.50)  # m/s and position on line -2.0..2.0
        # state = 666  # until no more line
        # pose.tripBreset()  # use trip counter/timer B
        state = 999

    elif state == 666:
        
        if edge.crossingLine == False and edge.crossingLineCnt > 0:
          print("Line crossed")
          cross_cnt += 1

        if cross_cnt > 4:
          edge.lineControl(0, 0)  # stop following line
          service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)

    elif state == 999:
        edge.lineControl(0, 0)  # stop following line
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
  
    # elif state == 12:  # following line
    #   if edge.lineValidCnt == 0 or pose.tripBtimePassed() > 10:
    #     # no more line
    #     edge.lineControl(0, 0)  # stop following line
    #     pose.tripBreset()
    #     #service.send(service.topicCmd + "ti/rc", "0.1 0.5")  # turn left
    #     state = 14  # turn left
    # elif state == 14:  # turning left
    #   if pose.tripBh > np.pi / 2 or pose.tripBtimePassed() > 10:
    #     state = 20 # finished   =17 go look for line
    #     service.send(service.topicCmd + "ti/rc", "0 0")  # stop for images
    #   print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")
    
    # elif state == 20:  # image analysis
    #   imageAnalysis(images == 2)
    #   images += 1
    #   # blink LED
    #   if ledon:
    #     service.send(service.topicCmd + "T0/leds", "16 0 64 0")
    #     gpio.set_value(20, 1)
    #   else:
    #     service.send(service.topicCmd + "T0/leds", "16 0 30 30")
    #     gpio.set_value(20, 0)
    #   ledon = not ledon
    #   state = 30  # go to camera-based navigation
    #   # finished?
    #   if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
    #     images = 0
        
    #   pass
    # elif state == 30:  # camera-based navigation
    #   # Convert target position from camera coordinates to robot coordinates
    #   #target_robot_x, target_robot_y = camera_to_robot_coordinates(TARGET_CAMERA_X, TARGET_CAMERA_Y)
    #   #print(f"% Target in robot coordinates: x = {target_robot_x:.2f}, y = {target_robot_y:.2f}")
    #   if (aruco.g_aruco is True):
    #     target_robot_x = aruco.g_coords[0][0]
    #     target_robot_y = aruco.g_coords[0][1]

    #     print(target_robot_x)
    #     print(target_robot_y)
    #     # Move the robot toward the target
    #     move_robot_toward_target(target_robot_x, target_robot_y)
    #     state = 99  # mission complete
    #   else:
    #     state = 30
    #     imageAnalysis(False)
    #     print(aruco.g_aruco)


    else:  # abort
      print(f"% Mission finished/aborted; state={state}")
      break
    # allow openCV to handle imshow (if in use)
    # images are almost useless while turning, but
    # used here to illustrate some image processing (painting)
    if (cam.useCam):
     imageAnalysis(False)
     key = cv.waitKey(100)  # ms
     if key > 0:  # e.g. Esc (key=27) pressed with focus on image
       break
    
    # note state change and reset state timer
    if state != oldstate:
      flog.write(state)
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()
    # do not loop too fast
    t.sleep(0.1)
    # tell interface that we are alive
    service.send(service.topicCmd + "ti/alive", str(service.startTime))
    pass  # end of while loop
  # end of mission, turn LEDs off and stop
  service.send(service.topicCmd + "T0/leds", "16 0 0 0")
  gpio.set_value(20, 0)
  edge.lineControl(0, 0)  # stop following line
  service.send(service.topicCmd + "ti/rc", "0 0")
  t.sleep(0.05)
  pass


############################################################

if __name__ == "__main__":
    print("% Starting")
    # where is the MQTT data server:
    #service.setup('localhost') # localhost
    #service.setup('10.197.217.81') # Juniper
    #service.setup('10.197.217.80') # Newton
    service.setup('10.197.219.27') # Sky
    if service.connected:
      loop()
      service.terminate()
    print("% Main Terminated")
