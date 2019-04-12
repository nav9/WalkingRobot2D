# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author
import sys
import time
import math
import pygame
from pymunk import Vec2d
import pymunk.pygame_util
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT
from pygame.color import THECOLORS
from WalkingRobot import RobotBody, ActionsNetwork
from Environments import *

class Simulator(object):
    worlds = []
    worldOrdinal = -1
    focusRobotXY = Vec2d(0, 0)#will be overridden below
    robots = []
    numRobots = 1
    collHand = []#collision handler
    actionNetwork = ActionsNetwork()
    world = None
    screen = None
    draw_options = None    

    def __init__(self):
        #NOTE: Pymunk physics coordinates start from the lower right-hand corner of the screen
        self.screenWidth = 800; self.screenHeight = 400
        self.display_flags = 0
        self.minViewX = 100; self.maxViewX = self.screenWidth - self.minViewX
        self.minViewY = 100; self.maxViewY = self.screenHeight - self.minViewY

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50
        self.iterations = 10        
        #self.space.damping = 0.999 
        
        self.worlds.append(TrainingWorld1())#registration of a world
        self.focusRobotID = 0 #the first robot created will be the focus robot. ie: The screen moves with this robot. Focus robot id can be changed dynamically

    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initializeTrainingBoundary(self.space)
            w.initializeTrainingObjects()        
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?

    def draw(self):        
        self.screen.fill(THECOLORS["black"])# Clear screen
        self.screen.fill((30, 30, 30))# Clear screen  
        #self.screen.fill((255, 243, 202))# Clear screen        
        self.space.debug_draw(self.draw_options)# Draw space        
        pygame.display.flip()#flip the display buffer

    def getUpdateBy(self, currentXY):
        updateBy = Vec2d(0,0)
        if currentXY[0] < self.minViewX or currentXY[0] > self.maxViewX:
            updateBy = -1 * Vec2d(currentXY[0] - self.focusRobotXY[0], 0) 
        if currentXY[1] < self.minViewY or currentXY[1] > self.maxViewY:
            updateBy = -1 * Vec2d(0, currentXY[1] - self.focusRobotXY[1])
        return updateBy

    
    def main(self):
        pygame.init()
        pygame.mixer.quit()#disable sound output that causes annoying sound effects if any other external music player is playing
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight), self.display_flags)
        #width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 200,200,200
        
        while self.nextWorld():
            clock = pygame.time.Clock()
            simulating = True

            #---Create the spider robots
            self.focusRobotXY = Vec2d(self.screenWidth/2, self.screenHeight/2)
            for i in range(0, self.numRobots, 1):
                self.robots.append(RobotBody(self.space, self.focusRobotXY, self.actionNetwork))
        
            #prevTime = time.time();
            while simulating:
                for event in pygame.event.get():
                    if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                        #simulating = False
                        sys.exit(0)
#                     elif event.type == KEYDOWN and event.key == K_s:
#                         # Start/stop simulation.
#                         simulate = not simulate
#                     elif event.type == KEYDOWN and event.key == K_r:
#                         # Reset.
#                         # simulate = False
#                         self.reset_bodies()
#                     elif event.type == KEYDOWN and event.key == K_UP:
#                         motor_ba1Left.rate = rotationRate
#                     elif event.type == KEYDOWN and event.key == K_DOWN:
#                         motor_ba1Left.rate = -rotationRate
#                     elif event.type == KEYDOWN and event.key == K_LEFT:
#                         motor_ac1Left.rate = rotationRate
#                     elif event.type == KEYDOWN and event.key == K_RIGHT:
#                         motor_ac1Left.rate = -rotationRate                    
#                     elif event.type == KEYUP:
#                         motor_ba1Left.rate = 0
#                         motor_ac1Left.rate = 0
 
                #---Update physics
                dt = 1.0/float(self.fps)/float(self.iterations)
                for x in range(self.iterations): #iterations to get a more stable simulation
                    self.space.step(dt)
                #---Update world based on player focus
                updateBy = self.getUpdateBy(self.robots[self.focusRobotID].chassis_body.position)
                if updateBy != (0, 0):
                    for obj in self.robots:#update all robot positions
                        obj.updatePosition(updateBy)
                    self.world.updatePosition(updateBy)
                #---iterate robots
                for r in self.robots:
                    r.brainActivity()
    #             if time.time() - prevTime > 0:
    #                 for r in self.robots:
    #                     #r.legs[0].leg_body.angle = 0
    #                     print('Angle'+str(math.degrees(r.legs[0].leg_body.angle)%360))
    #                     #r.brainActivity()
    #                     #r.stopBabyTrainingStage()
                #---draw all objects
                self.draw()
                 
                self.focusRobotXY = self.robots[self.focusRobotID].chassis_body.position#use getter
                clock.tick(self.fps)


#-----------------------------------------------
#-----------------------------------------------
#             PROGRAM STARTS HERE
#-----------------------------------------------
#-----------------------------------------------
if __name__ == '__main__':
    sim = Simulator()
    sim.main()
    
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
    