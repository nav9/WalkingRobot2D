# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

from Worlds import *
from WalkingRobot import ActionNetwork

class Simulator(object):
    
    def __init__(self):
        self.actions = ActionNetwork()
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        self.worlds.append(FlatGroundTraining())
        #self.worlds.append(ImaginationTwin(self.actions))
       
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            w.runWorld()    
            
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?


#-----------------------------------------------
#-----------------------------------------------
#             PROGRAM STARTS HERE
#-----------------------------------------------
#-----------------------------------------------
if __name__ == '__main__':
    sim = Simulator()
    while sim.nextWorld():
        pass
        
#     def __drawCollision__(self, arbiter, space, data):        
#         for c in arbiter.contact_point_set.points:
#             r = max( 3, abs(c.distance*5) )
#             r = int(r)
#             p = tuple(map(int, c.point_a))
#             pygame.draw.circle(data["surface"], THECOLORS["red"], p, r, 0)        
    