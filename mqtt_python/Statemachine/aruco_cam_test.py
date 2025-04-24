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
setproctitle("aruco_cam_test")

############################################################

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

  while not (service.stop or gpio.stop()):
    if state == 0:  # wait for start signal
      start = gpio.start() or service.args.now
      if start:
        print("% Starting")
        service.send("robobot/cmd/T0/servo","1 99999 0")
        service.send("robobot/cmd/T0/servo","2 99999 0")
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)
    elif state == 666:
        print("% State 666")    

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
    service.setup('10.197.219.27') # Sky
    if service.connected:
      loop()
      service.terminate()
    print("% Main Terminated")
