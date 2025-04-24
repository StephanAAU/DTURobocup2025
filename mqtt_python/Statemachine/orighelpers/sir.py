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


import time as t
from datetime import *

class SIr:

    ir = [0, 0]
    irUpdCnt = 0
    irTime = datetime.now()
    irInterval = 0

    wallInterval_setup = 0.0 # setup time (ms)
    wallCtrl = False # private
    # try with a P-controller
    wallKp = 15
    wallTauZ = 2
    #wallTauP = 0.15
    wallTauP = 0.1*1.3
    # Lead pre-calculated factors
    tauP2pT = 1.0
    tauP2mT = 0.0
    tauZ2pT = 1.0
    tauZ2mT = 0.0
    # control values
    wallE1 = 0.0 # old error * Kp (rad/s)
    wallY1 = 0.0 # old control output (rad/s)
    wallY = 0.0  # control output (rad/s)


    def getdist(self):
      return (self.ir[0],self.ir[1])
    

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from orighelpers.uservice import service
      loops = 0
      while not service.stop:
        # wait for data to arrive
        t.sleep(0.01)
        if self.irUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% IR sensor (sir.py):: got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% IR sensor (sir.py):: missing data updates after {loops} wait loops (continues).")
          break
        pass
      pass

    def print(self):
      from orighelpers.uservice import service
      print("% IR dist  (" +
            str(self.ir[0]) + ", " +
            str(self.ir[1]) + ", " +
            f") {self.irInterval:.4f} sec " +
            str(self.irUpdCnt))

    def decode(self, topic, msg):
        # decode MQTT message
        
        used = True
        if topic == "T0/ir" or topic == "T0/ird":
          gg = msg.split(" ")
          if (len(gg) >= 3):
            t0 = self.irTime;
            self.irTime = datetime.fromtimestamp(float(gg[0]))
            self.ir[0] = float(gg[1])
            self.ir[1] = float(gg[2])
            t1 = self.irTime;
            if self.irUpdCnt == 2:
              self.irInterval = (t1 -t0).total_seconds()
            else:
              self.irInterval = (self.irInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.irUpdCnt += 1

            #self.print()
            if self.wallCtrl:
                self.followwall()
        else:
          used = False
        return used

    def terminate(self):
        print("% IR terminated")
        pass


    def wallControl(self, velocity, refPosition):
      self.velocity = velocity
      self.refPosition = refPosition
      # velocity 0 is turning off wall control
      self.wallCtrl = velocity > 0.001
      pass

    ##########################################################

    def followwall(self):
      from orighelpers.uservice import service
      # some parameters depend on sample time, adjust
      # print(f"wallCtrl:: sample time {self.wall_nInterval}")
      if abs((self.irInterval - self.wallInterval_setup)/1000) > 2.0: # ms
        self.PIDrecalculate()
        self.wallInterval_setup = self.irInterval
      # wall to (much) the right gives a wall position value.
      # Then the robot is too much to the left.
      # To correct we need a negative turnrate,
      # so sign is OK
      e = self.refPosition - self.ir[0] # wall position (m) - robot position (m)
      self.u = self.wallKp * e; # error times Kp
      # Lead filter
      self.wallY = (self.u * self.tauZ2pT - self.wallE1 * self.tauZ2mT + self.wallY1 * self.tauP2mT)/self.tauP2pT
      #
     
      if self.wallY > 0.5:
        self.wallY = 0.5
      elif self.wallY < -0.5:
        self.wallY = -0.5
      # save old values
      self.wallE1 = self.u
      self.wallY1 = self.wallY
      # make response
      
      par = f"{-self.velocity:.3f} {-self.wallY:.3f} {t.time()}"
      service.send("robobot/cmd/ti/rc", par) # send new turn command, maintaining velocity


    ##########################################################

    def PIDrecalculate(self):
      print(f"wallCtrl:: PIDrecalculate: T={self.wallIntervalSetup:.2f} -> {self.wall_nInterval:.2f} ms")
      Tsec = self.wall_ninterval/1000
      self.tauP2pT = self.wallTauP * 2.0 + Tsec
      self.tauP2mT = self.wallTauP * 2.0 - Tsec
      self.tauZ2pT = self.wallTauZ * 2.0 + Tsec
      self.tauZ2mT = self.wallTauZ * 2.0 - Tsec
      # debug
      print(f"%% Lead: tauZ {self.wallTauZ:.3f} sec, tauP = {self.wallTauP:.3f} sec, T = {self.wall_nInterval:.3f} ms\n")
      print(f"%%       tauZ2pT = {self.tauZ2pT:.4f}, tauZ2mT = {self.tauZ2mT:.4f}, tauP2pT = {self.tauP2pT:.4f}, tauP2mT = {self.tauP2pT:.4f}")


    ##########################################################


    ##########################################################

# create the data object
ir = SIr()

