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

class SImu:

    gyro = [0, 0, 0]
    gyroUpdCnt = 0
    gyroTime =datetime.now()
    gyroInterval = 1

    gyro_bias = [0,0,0]

    acc  = [0, 0, 0]
    accTime = datetime.now()
    accUpdCnt = 0
    accInterval = 1

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from orighelpers.uservice import service
      loops = 0
      while not service.stop:
        t.sleep(0.01)
        if self.gyroUpdCnt == 0 or self.accUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% IMU (simu.py):: got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% IMU (simu.py):: no data updates after {loops} wait loops (continues).")
          break
        pass
        loops += 1
      pass

    def print(self):
      from uservice import service
      print("% IMU acc  " + str(self.accTime - service.startTime) + " (" +
            str(self.acc[0]) + ", " +
            str(self.acc[1]) + ", " +
            str(self.acc[2]) + f") {self.gyroInterval:.4f} sec " +
            str(self.accUpdCnt))
      print("% IMU gyro " + str(self.gyroTime - service.startTime) + " (" +
            str(self.gyro[0]) + ", " +
            str(self.gyro[1]) + ", " +
            str(self.gyro[2]) + f") {self.accInterval:.4f} sec " +
            str(self.gyroUpdCnt))

    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/gyro":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.gyroTime;
            self.gyroTime = datetime.fromtimestamp(float(gg[0]))
            self.gyro[0] = float(gg[1]) - self.gyro_bias[0]  # Subtract the bias
            self.gyro[1] = float(gg[2]) - self.gyro_bias[1]  # Subtract the bias
            self.gyro[2] = float(gg[3]) - self.gyro_bias[2]  # Subtract the bias
            t1 = self.gyroTime;
            if self.gyroUpdCnt == 2:
              self.gyroInterval = (t1 -t0).total_seconds()
            else:
              self.gyroInterval = (self.gyroInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.gyroUpdCnt += 1
            # self.print()
        elif topic == "T0/acc":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.accTime;
            self.accTime = datetime.fromtimestamp(float(gg[0]))
            self.acc[0] = float(gg[1])
            self.acc[1] = float(gg[2])
            self.acc[2] = float(gg[3])
            t1 = self.accTime;
            if self.accUpdCnt == 2:
               self.accInterval = (t1 -t0).total_seconds()
            else:
               self.accInterval = (self.accInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.accUpdCnt += 1
            # self.print()
        else:
          used = False
        return used
    

    def calibrate_gyro(self, num_samples=1000, sample_interval=0.01):
        """
        Calibrate the gyroscope by averaging `num_samples` readings and calculating the gyro bias (offset).
        This function will subtract the calculated bias from the gyroscope readings.

        Args:
            num_samples: The number of samples to take for calibration.
            sample_interval: The interval between each sample in seconds.
        """
        print("Starting gyro calibration...")
        gyro_sum = [0.0, 0.0, 0.0]

        for _ in range(num_samples):
            # Wait for a new gyro reading
            t.sleep(sample_interval)
            gyro_sum[0] += self.gyro[0]
            gyro_sum[1] += self.gyro[1]
            gyro_sum[2] += self.gyro[2]
            print(f"Calibrating... sample {_+1}/{num_samples}")

        # Calculate the average reading for each axis
        self.gyro_bias[0] = gyro_sum[0] / num_samples
        self.gyro_bias[1] = gyro_sum[1] / num_samples
        self.gyro_bias[2] = gyro_sum[2] / num_samples

        print(f"Gyro calibration complete. Bias: {self.gyro_bias}")


    def terminate(self):
        print("% Pose terminated")
        pass

# create the data object
imu = SImu()

