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
from Behaviours import DifferentialEvolution, RunCode
from statsmodels.sandbox.stats.runs import Runs

class Worlds(object):
    def __init__(self):
        self.focusRobotXY = Vec2d(0, 0)#will be overridden below
        self.focusRobotID = 0
        self.screen = None
        self.draw_options = None       
        
        #NOTE: Pymunk physics coordinates start from the lower right-hand corner of the screen
        self.screenWidth = 1000; 
        self.screenHeight = 550 #keep at at least 350
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
        self.prevCameraXY = Vec2d(self.screenWidth/2, self.screenHeight/2)
        self.cameraXY = Vec2d(self.screenWidth/2, self.screenHeight/2) 
        self.cameraMoveSpeed = Vec2d(100, 50)
        
    def initialize(self):
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50
        self.maxMovtTime = self.fps
        self.movtTime = self.maxMovtTime
        self.iterations = 20        
        #self.space.damping = 0.999 
        self.focusRobotChanged = False
        self.focusRobotID = self.numRobots-1 #the last robot created will be the focus robot. ie: The screen moves with this robot. Focus robot id can be changed dynamically
        self.infoString = ""        
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
        self.displayStats(self.infoString);        
        pygame.display.flip()#flip the display buffer
        
    def processRobot(self):
        for r in self.robots:
            r.brainActivity()        

    def updatePosition(self):   
        self.robots[self.focusRobotID].setFocusRobotColor()            
        updateBy = self.cameraXY - self.prevCameraXY #self.calcUpdateBy(self.robots[self.focusRobotID].chassis_body.position)
        self.cameraXY = Vec2d(self.prevCameraXY[0], self.prevCameraXY[1])
        if updateBy != (0, 0):
            for ob in self.boundaryObjects:
                ob.body.position += updateBy  
            for obj in self.robots:#update all robot positions
                obj.updatePosition(updateBy)
            #self.statsPos += updateBy
        return updateBy           
    
    def updateColor(self):
        self.robots[self.focusRobotID].setFocusRobotColor()
        self.focusRobotChanged = False
        for obj in self.robots:#update all robot positions
            if obj == self.robots[self.focusRobotID]: obj.setFocusRobotColor() 
            else: obj.setNormalRobotColor()
                  
#     def calcUpdateBy(self, focusPointXY):
#         updateBy = Vec2d(0,0)
#         if focusPointXY[0] < self.minViewX or focusPointXY[0] > self.maxViewX:
#             updateBy = -1 * Vec2d(focusPointXY[0] - self.focusRobotXY[0], 0) 
#         if focusPointXY[1] < self.minViewY or focusPointXY[1] > self.maxViewY:
#             updateBy = -1 * Vec2d(0, focusPointXY[1] - self.focusRobotXY[1])
#         return updateBy   
    
    def deleteRobots(self):
        for r in self.robots:
            r.delete()
        self.robots[:] = []
        
    def initializeRobots(self):
        distFromWall = 500
        robotXY = Vec2d(self.worldX+distFromWall, self.worldY+200)
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
        runState = RunCode.CONTINUE
        clock = pygame.time.Clock()
        simulating = True
        self.initializeRobots()
        if len(self.robots) <= 0: print('Create at least one robot'); return
        
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
                    if event.key == K_UP: self.cameraXY += Vec2d(0, -self.cameraMoveSpeed[1])
                    if event.key == K_DOWN: self.cameraXY += Vec2d(0, self.cameraMoveSpeed[1])
                    if event.key == K_LEFT: self.cameraXY += Vec2d(self.cameraMoveSpeed[0], 0)
                    if event.key == K_RIGHT: self.cameraXY += Vec2d(-self.cameraMoveSpeed[0], 0)

            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for x in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            if self.focusRobotChanged: self.updateColor()
            if self.movtTime == 0:
                runState = self.processRobot()
                self.movtTime = self.maxMovtTime
            else:
                self.movtTime -= 1
            
            #---draw all objects            
            self.draw()
            
            self.focusRobotXY = self.robots[self.focusRobotID].chassis_body.position#use getter
            clock.tick(self.fps)
            if runState == RunCode.STOP:
                break                   

class FlatGroundTraining(Worlds):#inherits
    def __init__(self):
        super(FlatGroundTraining, self).__init__()
        self.worldWidth = 2000 #overriding
        self.numRobots = 5        
        self.elevFromBottomWall = 20
        self.groundThickness = 10
  
    def initialize(self):
        super(FlatGroundTraining, self).initialize()
        groundX = self.worldX+self.wallThickness/2; groundLen = self.worldWidth-2*self.wallThickness; groundY = self.elevFromBottomWall
        ground_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); groundStart = Vec2d(groundX, groundY); groundPosition = Vec2d(groundX+groundLen, groundY)
        ground_body.position = groundStart
        ground_shape = pymunk.Segment(ground_body, groundStart, groundPosition, self.groundThickness); ground_shape.friction = 1.0        
        self.space.add(ground_shape); self.worldObjects.append(ground_shape)  
        self.behaviour = DifferentialEvolution(self.robots)
        self.sequenceLength = 1
        self.maxSequenceLength = 5 #The number of dT times a leg is moved
        self.epoch = 0 
        self.maxEpochs = 5
        
    def processRobot(self):
        if self.sequenceLength == self.maxSequenceLength:
            return RunCode.STOP
        
        runCode = self.behaviour.run(self.sequenceLength)
        if self.epoch == self.maxEpochs:
            self.sequenceLength += 1 
            self.epoch = 0   
            self.deleteRobots(); self.initializeRobots()            
            self.behaviour.generatingSeq = True              
        else:
            if runCode == RunCode.NEXTEPOCH:
                self.epoch += 1
        self.infoString = "SeqLen: "+str(self.sequenceLength)+"  Epoch: "+str(self.epoch)+"  SeqRep: "+str(self.behaviour.repeatSeq)+"  Seq: "+str(self.behaviour.seqNum)
        
    def initializeRobots(self):
        super(FlatGroundTraining, self).initializeRobots()
        for r in self.robots:
            r.experience = []        
        
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
                            
                    