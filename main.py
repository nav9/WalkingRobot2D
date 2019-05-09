# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

# Requirements: Python 3.5+
# Installation instructions: First install dependent libraries:
# >>> pip3 install pygame
# >>> pip3 install pymunk
# >>> pip3 install networkx[all]
# >>> pip3 install networkx
# >>> pip3 install numpy
# >>> pip3 install scipy
# >>> pip3 install matplotlib
# Now simply run using: 
# >>> python3 main.py

import time
from Worlds import *
from WalkingRobot import ActionNetwork
from LearningRobot import PartActionNetwork

class TestSimulator(object):
    def __init__(self, legs):
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        self.worlds.append(TestWorld(legs))
       
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            w.runWorld()   
            w.delete() 
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?
    
class Demo1Simulator(object):
    def __init__(self, execLen, legs):
        self.actions = ActionNetwork(execLen, legs)
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        #self.worlds.append(FlatGroundTraining(execLen, legs))
        self.worlds.append(ImaginationTwin(self.actions, execLen, legs))
       
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            #time.sleep(5)
            w.runWorld()
            w.delete()    
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?

class Demo2Simulator(object):
    def __init__(self, legs):
        self.actions = PartActionNetwork(legs)
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        #self.worlds.append(Heaven(legs, self.actions))
        self.worlds.append(ActualImagination(legs, self.actions))
       
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            w.runWorld()    
            w.delete()
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?
#-----------------------------------------------
#-----------------------------------------------
#             PROGRAM STARTS HERE
#-----------------------------------------------
#-----------------------------------------------
if __name__ == '__main__':
    execLen = 10 #how many times motor gets executed per second
    # - Single leg part, -- Two leg parts, # Chassis
    legs = '--#--'
    #sim = Demo1Simulator(execLen, legs)
    sim = Demo2Simulator(legs)
    #sim = TestSimulator(legs)
    while sim.nextWorld():
        pass
