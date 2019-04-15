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
from pygame.locals import *
from pygame.color import *
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_LEFTBRACKET, K_RIGHTBRACKET, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT
from pygame.color import THECOLORS
from WalkingRobot import RobotBody, ActionsNetwork
from Behaviours import DifferentialEvolution

class Worlds(object):
    def __init__(self):
        self.focusRobotXY = Vec2d(0, 0)#will be overridden below
        self.focusRobotID = 0
        self.screen = None
        self.draw_options = None       
         
        #NOTE: Pymunk physics coordinates start from the lower right-hand corner of the screen
        self.screenWidth = 1000; 
        self.screenHeight = 700 #keep at at least 350
        self.boundaryObjects = []
        self.worldObjects = []
        self.worldX = 0
        self.worldY = 0 
        self.worldWidth = 1000
        self.worldHeight = 500
        self.wallThickness = 15
        self.boundaryColor = 170,170,170
        self.robots = []
        self.behaviour = None
        self.numRobots = 3#can be overridden in child class
        self.actionNetwork = ActionsNetwork()        
        self.display_flags = 0
        self.minViewX = 100; self.maxViewX = self.screenWidth - self.minViewX
        self.minViewY = 100; self.maxViewY = self.screenHeight - self.minViewY  
        self.statsPos = Vec2d(0, 0)      

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50
        self.iterations = 20        
        #self.space.damping = 0.999 
        self.focusRobotChanged = False
        self.focusRobotID = self.numRobots-1 #the last robot created will be the focus robot. ie: The screen moves with this robot. Focus robot id can be changed dynamically
        
        #---top boundary        
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.worldX+self.worldWidth/2, self.worldY+self.worldHeight-self.wallThickness/2)
        shape = pymunk.Poly.create_box(body, (self.worldWidth, self.wallThickness)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---bottom boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.worldX+self.worldWidth/2, self.worldY+self.wallThickness/2) 
        shape = pymunk.Poly.create_box(body, (self.worldWidth, self.wallThickness)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---left boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.worldX+self.wallThickness/2, self.worldY+self.worldHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.worldHeight)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---right boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.worldX+self.worldWidth-self.wallThickness/2, self.worldY+self.worldHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.worldHeight)); shape.color = self.boundaryColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
    
        self.initializeRobots()
        
    def delete(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
        for ob in self.robots:
            self.space.remove(ob)
        self.robots[:] = []

    def draw(self):        
        #self.screen.fill(THECOLORS["black"])# Clear screen
        self.screen.fill((30, 30, 30))# Clear screen  
        #self.screen.fill((255, 243, 202))# Clear screen        
        self.space.debug_draw(self.draw_options)# Draw space
        self.displayStats(self.behaviour.infoString);        
        pygame.display.flip()#flip the display buffer
        
    def processRobot(self):
        for r in self.robots:
            r.brainActivity()        

    def updatePosition(self):   
        self.robots[self.focusRobotID].setFocusRobotColor()            
        updateBy = self.calcUpdateBy(self.robots[self.focusRobotID].getPosition())
        if updateBy != (0, 0):
            for ob in self.boundaryObjects:
                ob.body.position += updateBy  
            for obj in self.robots:#update all robot positions
                obj.updatePosition(updateBy)
            self.statsPos += updateBy
        return updateBy           
    
    def updateColor(self):
        self.robots[self.focusRobotID].setFocusRobotColor()
        self.focusRobotChanged = False
        for obj in self.robots:#update all robot positions
            if obj == self.robots[self.focusRobotID]: obj.setFocusRobotColor() 
            else: obj.setNormalRobotColor()
                  
    def calcUpdateBy(self, focusRobotsXY):
        updateBy = Vec2d(0,0)
        if focusRobotsXY[0] < self.minViewX or focusRobotsXY[0] > self.maxViewX:
            updateBy = -1 * Vec2d(focusRobotsXY[0] - self.focusRobotXY[0], 0) 
        if focusRobotsXY[1] < self.minViewY or focusRobotsXY[1] > self.maxViewY:
            updateBy = -1 * Vec2d(0, focusRobotsXY[1] - self.focusRobotXY[1])
        return updateBy   
    
    def initializeRobots(self):
        distFromWall = 100
        robotXY = Vec2d(self.worldX+distFromWall, self.worldY+self.worldHeight/2)
        for i in range(0, self.numRobots, 1):
            self.robots.append(RobotBody(self.space, robotXY, self.actionNetwork))             
    
    def displayStats(self, displayStr):
        self.screen.blit(self.font.render(displayStr, 1, THECOLORS["green"]), self.statsPos)
                        
    def runWorld(self):
        pygame.init()
        pygame.mixer.quit()#disable sound output that causes annoying sound effects if any other external music player is playing
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight), self.display_flags)
        self.font = pygame.font.SysFont("Arial", 14)
        #width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 200,200,200
        
        clock = pygame.time.Clock()
        simulating = True
        
        if len(self.robots) <= 0: print('Create at least one robot');return
#             #---Create the spider robots
#             self.focusRobotXY = Vec2d(self.screenWidth/2, self.screenHeight/2)
#             for i in range(0, self.numRobots, 1):
#                 self.robots.append(RobotBody(self.space, self.focusRobotXY, self.actionNetwork))

        #prevTime = time.time();
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    sys.exit(0)
                if event.type == KEYDOWN:
                    if event.key == K_RIGHTBRACKET:
                        self.focusRobotID += 1; self.focusRobotChanged = True
                        if self.focusRobotID == self.numRobots: self.focusRobotID = 0
                    if event.key == K_LEFTBRACKET:
                        self.focusRobotID -= 1; self.focusRobotChanged = True
                        if self.focusRobotID < 0: self.focusRobotID = self.numRobots - 1
#                     if event.key == K_UP:
#                     if event.key == K_DOWN:
#                     if event.key == K_LEFT:
#                     if event.key == K_RIGHT:

            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for x in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            if self.focusRobotChanged: self.updateColor()
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
    def __init__(self):
        super(FlatGroundTraining, self).__init__()
        self.numRobots = 5        
        self.elevFromBottomWall = 20
        self.groundThickness = 10
        groundX = self.worldX+self.wallThickness/2; groundLen = self.worldWidth-2*self.wallThickness; groundY = self.elevFromBottomWall
        ground_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); groundStart = Vec2d(groundX, groundY); groundPosition = Vec2d(groundX+groundLen, groundY)
        ground_body.position = groundStart
        ground_shape = pymunk.Segment(ground_body, groundStart, groundPosition, self.groundThickness); ground_shape.friction = 1.0        
        self.space.add(ground_shape); self.worldObjects.append(ground_shape)  
        self.behaviour = DifferentialEvolution(self.robots)
    
    def processRobot(self):
        self.behaviour.run()        
        
    def delete(self):
        super(FlatGroundTraining, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
    
    def updatePosition(self):  
        updateBy = super(FlatGroundTraining, self).updatePosition()     
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy     
                            
                    