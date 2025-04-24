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


def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      #edge.paint(img)
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"aruco_distance_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass
def loop():
    service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
    t.sleep(3)
    for _ in range(0, 50):

        service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
        t.sleep(1)
        service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
        t.sleep(1)
        imageAnalysis(True)
  
if __name__ == "__main__":
    print("% Starting")
    # where is the MQTT data server:
    service.setup('localhost') # localhost
    #service.setup('10.197.217.81') # Juniper
    #service.setup('10.197.217.80') # Newton
    #service.setup('10.197.218.172')
    if service.connected:
      loop()
      service.terminate()
    print("% Main Terminated")