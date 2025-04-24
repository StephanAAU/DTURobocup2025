import time
import numpy as np

#import states
from orighelpers.sedge import edge
from orighelpers.sir import ir
from orighelpers.spose import pose
from orighelpers.uservice import service

from Libraries.RobotMover import robotmover

from Statehelpers.Stateinterface import State
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars

from States.AxePasser import AxePasser
from States.Goalstate import Goalstate

class LineFollow(State):
    def __init__(self,systemvars:systemVars):

        self.spam_stop_cntr = 0

        self.debounce_flag = False
        self.crossing_counter = 0
        self.cross_threshold_pos = 0.2  # m
        self.trend  =   0
        self.timesInIntermediate = 0
        self.ref_pos = 0

        trans1 = Transition(AxePasser, lambda: self.crossing_counter == 3 and self.systemvars.timesInLinefollow == 1, [lambda: setattr(self, 'crossing_counter', 0), lambda : setattr(self,'timesInIntermediate',0)]) # initialize with next state, condition as a lambda function and instruction

        trans2 = Transition(Goalstate, lambda: self.crossing_counter == 6 , [lambda: setattr(self, 'crossing_counter', 0)]) # initialize with next state, condition as a lambda function and instruction

        transitions = [trans1,trans2]

        # trans1 = Transition(None, lambda: self.timesInIntermediate > 2000, [])
        # transitions = [trans1]
        super().__init__(transitions, systemvars)


    def start_following(self, speed):
        edge.crossingLine = False
        pose.tripBreset()  # use trip counter/timer B
        
        edge.lineControl(speed, self.ref_pos)  # m/s and position on line -2.0..2.0
        print(f"start following with speed: {speed} and ref_pos: {self.ref_pos}")


    def stop_following(self):
        edge.lineControl(0, 0)  # stop
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # (forward m/s, turnrate rad/sec)


    def check_for_crossing(self):
        # self.spam_stop_cntr += 1
        # if ((self.spam_stop_cntr % 10) == 0):
        #     print(f"cross cnt: {self.crossing_counter}, tripB: {pose.tripB}")
        #     edge.printn()
        if (edge.crossingLine == True) and (self.debounce_flag == False):
            self.crossing_counter += 1
            self.debounce_flag = True

            print(f"cross cnt: {self.crossing_counter} \t tripB: {pose.tripB}")
            print(f"timesInLine: {self.systemvars.timesInLinefollow} \t timesInIntermediate: {self.timesInIntermediate}")

            pose.tripBreset()  # use trip counter/timer B
        
        if (self.debounce_flag == True) and (pose.tripB > self.cross_threshold_pos):
            self.debounce_flag = False

    def intermediatestep(self):

        if self.systemvars.timesInLinefollow == 1:
            if self.crossing_counter == 1 and self.timesInIntermediate == 0:
                print("turning first corner")
                self.stop_following()

                robotmover.turn(np.pi/8, speed=0.2, turnrate=0.8)
                robotmover.timer.join()

                edge.setslow()
                self.ref_pos = 1.4
                self.start_following(0.2)
                self.timesInIntermediate += 1

        elif self.systemvars.timesInLinefollow == 2:
            #print(self.crossing_counter, self.intermediatestep)
            if self.crossing_counter == 3 and self.timesInIntermediate == 0:
                self.timesInIntermediate += 1
                self.stop_following()
                robotmover.turn(np.pi/5,speed=0.1)
                robotmover.timer.join()
                robotmover.driveDist(4,0.2)
                time.sleep(2.5)
                print("done")

                while not edge.lineValid:
                    time.sleep(0.05)
                
                print("turning")
                
              
               
                self.ref_pos = 0
                print("return")
                robotmover.turn(np.pi/4,speed = 0.2, turnrate=1)
                robotmover.timer.join()
              
                edge.setfast()
                self.start_following(0.4)
                
                
            elif self.crossing_counter == 5 and self.timesInIntermediate == 1:
                time.sleep(1)
                self.ref_pos = 1
                self.timesInIntermediate += 1
                self.start_following(0.3)
            elif self.crossing_counter == 6 and self.timesInIntermediate == 2:
                self.stop_following()
                robotmover.stopmove()
                print("last intermediate")

            return 1



    def speed_adjust(self):
        if self.crossing_counter < 3:
            # This is the initial line following speed
            pass
        elif self.crossing_counter < 4:
            # Now we near the axe thing we need to slow down and offset
            # the other side to avoid following the axe
            edge.lineControl(0.15, -0.50)
            # Now we passed the axe
        elif self.crossing_counter < 6:
            # Now we passed the ball
            edge.lineControl(0.15, -0.50)
        elif self.crossing_counter < 7:
            # Now we are at the next dobbel cross
            edge.lineControl(0.15, 0.50)
        elif self.crossing_counter < 8:
            # We are now ear the goal and need to turn sharp
            pose.tripBreset()
            self.crossing_counter += 1
        elif self.crossing_counter < 9:
            ## Hacky way to detect turn
            if (pose.tripB > 0.1):
                self.crossing_counter += 1
                edge.lineControl(0.15, 0.2)
        elif self.crossing_counter < 10:
            edge.lineControl(0.15, 1.5)
        elif self.crossing_counter < 11:
            edge.lineControl(0.15, 0.2)
            pose.tripBreset()
            if (pose.tripB > 1.5):
                self.crossing_counter += 1
        else:
            pass

    def reset_crossing_counter(self):
        self.crossing_counter = 0


    def execute(self) -> Transition:
        self.systemvars.timesInLinefollow += 1
        service.send("robobot/cmd/T0/servo",  f"1 {350} {200}")
        #print("Executing Line follow")
        service.send("robobot/cmd/T0/servo",  f"1 0 300") ## Lift arm over gnd
        self.reset_crossing_counter()
        self.ref_pos = 0
        edge.setfast()
        #edge.setslow()
        self.start_following(0.4)
        edge.lineControl(0.2, 0.0)

        starttime = time.time()


        while True:
            self.check_for_crossing()
            self.intermediatestep()
               
#            self.speed_adjust()
        
            time.sleep(0.05)
            self.systemvars.var2 = time.time() - starttime
            
            trans_out = self.checkIfDone()
            
            if trans_out != None:
                print(trans_out)
                #self.stop_following()
                return trans_out
            
       
            
        
