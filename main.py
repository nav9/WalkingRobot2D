# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import time
from Worlds import *
from WalkingRobot import ActionNetwork

class TimeBasedSimulator(object):
    def __init__(self, execLen, legs):
        self.actions = ActionNetwork(execLen, legs)
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        #self.worlds.append(FlatGroundTraining(execLen, legs))
        #self.worlds.append(ImaginationTwin(self.actions, execLen, legs))
       
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            #time.sleep(5)
            w.runWorld()    
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?

class TimeIndepSimulator(object):#TODO: make it independent of exec len
    def __init__(self, execLen, legs):
        self.actions = ActionNetwork(execLen, legs)
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        self.worlds.append(Womb(execLen, legs))
       
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            #time.sleep(5)
            w.runWorld()    
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
    #sim = TimeBasedSimulator(execLen, legs)
    sim = TimeIndepSimulator(execLen, legs)
    while sim.nextWorld():
        pass
        
#     def __drawCollision__(self, arbiter, space, data):        
#         for c in arbiter.contact_point_set.points:
#             r = max( 3, abs(c.distance*5) )
#             r = int(r)
#             p = tuple(map(int, c.point_a))
#             pygame.draw.circle(data["surface"], THECOLORS["red"], p, r, 0)        
    
