# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

from Environments import *

class Simulator(object):
    worlds = []
    worldOrdinal = -1

    def __init__(self):
        self.worlds.append(TrainingWorld1())#registration of a world

    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initializeTrainingBoundary()
            w.initializeTrainingObjects() 
            w.run()       
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
    
#     def reset_bodies(self):
#         for body in self.space.bodies:
#             if not hasattr(body, 'startPosition'):
#                 continue
#             body.position = Vec2d(body.startPosition)
#             body.force = 0, 0
#             body.torque = 0
#             body.velocity = 0, 0
#             body.angular_velocity = 0
#             body.angle = body.startAngle    
    
#     def __drawCollision__(self, arbiter, space, data):        
#         for c in arbiter.contact_point_set.points:
#             r = max( 3, abs(c.distance*5) )
#             r = int(r)
#             p = tuple(map(int, c.point_a))
#             pygame.draw.circle(data["surface"], THECOLORS["red"], p, r, 0)        
    