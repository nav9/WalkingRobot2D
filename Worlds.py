# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import sys
import time
import math
import pygame
from pymunk import Vec2d
import pymunk.pygame_util
import pymunk
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT
from pygame.color import THECOLORS
from WalkingRobot import RobotBody, ActionsNetwork

class Worlds:
    focusRobotXY = Vec2d(0, 0)#will be overridden below
    focusRobotID = 0
    screen = None
    draw_options = None       
    
    boundaryObjects = []
    space = None  
    envX = 0
    envY = 0  
    envWidth = 1000
    envHeight = 300
    wallThickness = 15
    boundaryColor = 170,170,170
    robots = []
    numRobots = 1  
    actionNetwork = ActionsNetwork()

    def __init__(self):
        #NOTE: Pymunk physics coordinates start from the lower right-hand corner of the screen
        self.screenWidth = 300; 
        self.screenHeight = 300
        self.display_flags = 0
        self.minViewX = 100; self.maxViewX = self.screenWidth - self.minViewX
        self.minViewY = 100; self.maxViewY = self.screenHeight - self.minViewY

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50
        self.iterations = 20        
        #self.space.damping = 0.999 
        
        #self.worlds.append(FlatGroundTraining())#registration of a world
        self.focusRobotID = 0 #the first robot created will be the focus robot. ie: The screen moves with this robot. Focus robot id can be changed dynamically

    def initializeTrainingBoundary(self):#NOTE: The x,y position of a body is it's body center. Not the top left coordinate
        #---top boundary        
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX+self.envWidth/2, self.envY+self.envHeight-self.wallThickness/2)
        shape = pymunk.Poly.create_box(body, (self.envWidth, self.wallThickness)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---bottom boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX+self.envWidth/2, self.envY+self.wallThickness/2) 
        shape = pymunk.Poly.create_box(body, (self.envWidth, self.wallThickness)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---left boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX+self.wallThickness/2, self.envY+self.envHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.envHeight)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---right boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX+self.envWidth-self.wallThickness/2, self.envY+self.envHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.envHeight)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
    
    def deleteTrainingBoundary(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list

    def draw(self):        
        self.screen.fill(THECOLORS["black"])# Clear screen
        self.screen.fill((30, 30, 30))# Clear screen  
        #self.screen.fill((255, 243, 202))# Clear screen        
        self.space.debug_draw(self.draw_options)# Draw space        
        pygame.display.flip()#flip the display buffer
        
    def processRobot(self):
        for r in self.robots:
            r.brainActivity()        
    
    def updatePosition(self):#gets overridden in derived class
        updateBy = self.calcUpdateBy(self.robots[self.focusRobotID].getPosition())
        if updateBy != (0, 0):
            for obj in self.robots:#update all robot positions
                obj.updatePosition(updateBy)        

    def calcUpdateBy(self, focusRobotsXY):
        updateBy = Vec2d(0,0)
        if focusRobotsXY[0] < self.minViewX or focusRobotsXY[0] > self.maxViewX:
            updateBy = -1 * Vec2d(focusRobotsXY[0] - self.focusRobotXY[0], 0) 
        if focusRobotsXY[1] < self.minViewY or focusRobotsXY[1] > self.maxViewY:
            updateBy = -1 * Vec2d(0, focusRobotsXY[1] - self.focusRobotXY[1])
        return updateBy   
    
    def initializeRobots(self):
        distFromWall = 100
        robotXY = Vec2d(self.envX+distFromWall, self.envY+self.envHeight/2)
        for i in range(0, self.numRobots, 1):
            self.robots.append(RobotBody(self.space, robotXY, self.actionNetwork))             
       
    def runWorld(self):
        pygame.init()
        pygame.mixer.quit()#disable sound output that causes annoying sound effects if any other external music player is playing
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight), self.display_flags)
        #width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 200,200,200
        
        clock = pygame.time.Clock()
        simulating = True
        self.initializeRobots()
#             #---Create the spider robots
#             self.focusRobotXY = Vec2d(self.screenWidth/2, self.screenHeight/2)
#             for i in range(0, self.numRobots, 1):
#                 self.robots.append(RobotBody(self.space, self.focusRobotXY, self.actionNetwork))
    
        #prevTime = time.time();
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    simulating = False
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
            self.updatePosition()
            self.processRobot()
#                 updateBy = self.calcUpdateBy(self.robots[self.focusRobotID].chassis_body.position)
#                 if updateBy != (0, 0):
#                     for obj in self.robots:#update all robot positions
#                         obj.updatePosition(updateBy)
                #self.updatePosition(updateBy)
#                 #---iterate robots
#                 for r in self.robots:
#                     r.brainActivity()
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

class FlatGroundTraining(Worlds):#inherits
    groundObjects = []
    elevFromBottomWall = 20
    groundThickness = 10

    def initializeObjects(self):     
        self.initializeTrainingBoundary()
        groundX = self.envX+self.wallThickness/2; groundLen = self.envWidth-2*self.wallThickness; groundY = self.elevFromBottomWall
        ground_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); groundStart = Vec2d(groundX, groundY); groundPosition = Vec2d(groundX+groundLen, groundY)
        ground_body.position = groundStart
        ground_shape = pymunk.Segment(ground_body, groundStart, groundPosition, self.groundThickness); ground_shape.friction = 1.0        
        self.space.add(ground_shape); self.groundObjects.append(ground_shape)         
    
    def deleteTrainingObjects(self):
        for ob in self.groundObjects:
            self.space.remove(ob)
        self.groundObjects[:] = []  
          
    def updatePosition(self):
        updateBy = self.calcUpdateBy(self.robots[self.focusRobotID].getPosition())
        if updateBy != (0, 0):
            for ob in self.boundaryObjects:
                ob.body.position += updateBy
            for ob in self.groundObjects:
                ob.body.position += updateBy    
            for obj in self.robots:#update all robot positions
                obj.updatePosition(updateBy) 
                            
                    