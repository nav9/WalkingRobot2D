# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import sys
import time
import math
import pygame
import random
import pymunk
import numpy as np
from pymunk import Vec2d
import pymunk.pygame_util
from pygame.color import *
from pygame.locals import *
from pygame.locals import *
from StatesAndSensors import *
import matplotlib.pyplot as plt
from pygame.color import THECOLORS
from WalkingRobot import RobotBody
from WalkingRobot import Constants
from LearningRobot import LearningRobot
from ComputationalIntelligence import DifferentialEvolution, SimpleDE, RunCode

class Worlds(object):
    def __init__(self):
        #self.focusRobotXY = Vec2d(0, 0)#will be overridden below        
        self.screen = None
        self.draw_options = None       
        self.decimalPrecision = 2
        #NOTE: Pymunk physics coordinates start from the lower right-hand corner of the screen
        self.screenWidth = 1300; #can get overridden in child class
        self.screenHeight = 400 #keep at at least 350. Can get overridden in child class
        self.boundaryObjects = []
        self.worldObjects = []
        self.worldWidth = 2000 #may get overridden in child class
        self.worldHeight = 500 #may get overridden in child class
        self.wallThickness = 15
        self.boundaryColor = 170,170,170
        self.groundColor = 0,130,0                
        self.robots = []
        self.behaviour = None
        self.numRobots = 3 #can be overridden in child class
        #self.actionNetwork = ActionsNetwork()        
        self.display_flags = 0        
        self.minViewX = 100; self.maxViewX = self.screenWidth - self.minViewX
        self.minViewY = 100; self.maxViewY = self.screenHeight - self.minViewY  
        self.statsPos = Vec2d(0, 0)
        self.robotInitPos = Vec2d(self.screenWidth/2, self.screenHeight/2) #could be overridden in base class
        self.prevCameraXY = Vec2d(self.screenWidth/2, self.screenHeight/2)
        self.cameraXY = Vec2d(self.screenWidth/2, self.screenHeight/2) 
        self.cameraMoveDist = Vec2d(100, 50)
        self.UNDETERMINED = -1
        #self.maxMovtTime = 50 #how often in time the sequences of the robot get executed        

    def initialize(self):
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50 #frames per second
        self.movtTime = 0 #start value of movt time. Can be anything from 0 to maxMovtTime
        self.iterations = 20        
        #self.space.damping = 0.999 
        #self.focusRobotChanged = False
        self.prevFocusRobotID = self.UNDETERMINED #At first none of the robots will be in focus since fitness hasn't been determined
        self.focusRobotID = self.UNDETERMINED #At first none of the robots will be in focus since fitness hasn't been determined
        self.infoString = ""           
        self.createWorldBoundary(0, 0, self.boundaryColor)
            
        pygame.init()
        pygame.mixer.quit()#disable sound output that causes annoying sound effects if any other external music player is playing
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight), self.display_flags)
        self.font = pygame.font.SysFont("Arial", 14)
        #width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 150,150,150
        #self.draw_options.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES
        #self.draw_options.flags |= pymunk.SpaceDebugDrawOptions.DRAW_COLLISION_POINTS

        self.initializeRobots()
        if len(self.robots) <= 0: print('Create at least one robot'); return

    def createWorldBoundary(self, worldX, worldY, bouColor):
        #---top boundary        
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.worldWidth/2, worldY+self.worldHeight-self.wallThickness/2)
        shape = pymunk.Poly.create_box(body, (self.worldWidth, self.wallThickness)); shape.color = bouColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---bottom boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.worldWidth/2, worldY+self.wallThickness/2) 
        shape = pymunk.Poly.create_box(body, (self.worldWidth, self.wallThickness)); shape.color = bouColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---left boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.wallThickness/2, worldY+self.worldHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.worldHeight)); shape.color = bouColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---right boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.worldWidth-self.wallThickness/2, worldY+self.worldHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.worldHeight)); shape.color = bouColor; shape.friction = 1.0
        self.space.add(shape); self.boundaryObjects.append(shape)
        
    def delete(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
        for ob in self.robots:
            ob.delete()
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
        #self.robots[self.focusRobotID].setFocusRobotColor()            
        updateBy = self.cameraXY - self.prevCameraXY #self.calcUpdateBy(self.robots[self.focusRobotID].chassis_body.position)
        self.cameraXY = Vec2d(self.prevCameraXY[0], self.prevCameraXY[1])
        if updateBy != (0, 0):
            for ob in self.boundaryObjects:
                ob.body.position += updateBy  
            for obj in self.robots:#update all robot positions
                obj.updatePosition(updateBy)
            self.robotInitPos += updateBy
            #self.statsPos += updateBy
        return updateBy           
    
    def updateColor(self):
        for obj in self.robots:#update all robot positions
            if self.focusRobotID >= 0:
                if obj == self.robots[self.focusRobotID]: obj.setFocusRobotColor() 
                else: obj.setNormalRobotColor()
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
        for _ in range(0, self.numRobots, 1):
            self.robots.append(RobotBody(self.space, self.robotInitPos, self.legsCode))             
    
    def displayStats(self, displayStr):
        if isinstance(displayStr, str): self.screen.blit(self.font.render(displayStr, 1, THECOLORS["green"]), self.statsPos)
        else:
            sep = 15
            for i in range(0,len(displayStr),1): self.screen.blit(self.font.render(displayStr[i], 1, THECOLORS["green"]), self.statsPos+(0,i*sep))

    def runWorld(self): #may get overridden in child class
        runState = RunCode.CONTINUE
        clock = pygame.time.Clock()
        simulating = True        
        #prevTime = time.time();
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    sys.exit(0)
                if event.type == KEYDOWN:
                    if event.key == K_UP: self.cameraXY += Vec2d(0, -self.cameraMoveDist[1])
                    if event.key == K_DOWN: self.cameraXY += Vec2d(0, self.cameraMoveDist[1])
                    if event.key == K_LEFT: self.cameraXY += Vec2d(self.cameraMoveDist[0], 0)
                    if event.key == K_RIGHT: self.cameraXY += Vec2d(-self.cameraMoveDist[0], 0)                    
#                     if event.key == K_RIGHTBRACKET:
#                         self.focusRobotID += 1; self.focusRobotChanged = True
#                         if self.focusRobotID == self.numRobots: self.focusRobotID = 0
#                     if event.key == K_LEFTBRACKET:
#                         self.focusRobotID -= 1; self.focusRobotChanged = True
#                         if self.focusRobotID < 0: self.focusRobotID = self.numRobots - 1

            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for _ in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            if self.prevFocusRobotID != self.focusRobotID: 
                self.updateColor()
                self.prevFocusRobotID = self.focusRobotID
            if self.movtTime == 0:
                runState = self.processRobot()
                self.movtTime = self.maxMovtTime
            else:
                self.movtTime -= 1
            
            #---draw all objects            
            self.draw()
            
            #self.focusRobotXY = self.robots[self.focusRobotID].chassis_body.position#use getter
            clock.tick(self.fps)
            if runState == RunCode.STOP:
                break  
            
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class RunStep:
    IMAGINARY_MOTOR_EXEC = 0
    IMAGINARY_GENERATION = 1
    #IMAGINARY_EPOCH = 2
    REAL_MOTOR_EXEC = 3
    REAL_GENERATION = 4   
    
class MoveMotors:#to move the motors for time n*dT, where n is the number of frames and dT is the frame duration
    def __init__(self, listOfRobots, parent):
        self.robots = listOfRobots    
        self.isMainRobot = (len(self.robots) == 1)
        self.world = parent
        self.maxDuration = 5 #the duration (number of frames) the motor has to move
        self.currDuration = 0        
    def run(self):
        if self.currDuration == 0:
            self.start()
            self.currDuration += 1              
        else:        
            if self.currDuration == self.maxDuration:
                self.stop()                   
                if self.isMainRobot: self.world.runState = RunStep.REAL_GENERATION
                else: self.world.runState = RunStep.IMAGINARY_GENERATION
            else:             
                #---do something 
                if self.isMainRobot: print('Main Duration ',self.currDuration)
                else: print('Duration ',self.currDuration)
                self.currDuration += 1
    def start(self):
        for robo in self.robots:
            robo.startMotion()
    def stop(self):
        self.currDuration = 0 #ready for next movement when state switches back to this object
        for robo in self.robots:
            robo.stopMotion()            

class Generation:#to run MoveMotors for g generations where each g = n*dT
    def __init__(self, listOfRobots, parent):
        self.robots = listOfRobots
        self.isMainRobot = (len(self.robots) == 1)
        self.world = parent  
        if self.isMainRobot: self.maxGens = 1 
        else: self.maxGens = 5
        self.currGen = 0  
    def run(self):        
        if not self.isMainRobot:
            pass #do DE here        
        if self.currGen == 0:
            self.start()
            self.currGen += 1
            if self.isMainRobot: self.world.runState = RunStep.REAL_MOTOR_EXEC 
            else: self.world.runState = RunStep.IMAGINARY_MOTOR_EXEC 
        else:    
            if self.currGen == self.maxGens:
                #---do whatever is done at end of a generation
                self.stop()
                if self.isMainRobot: self.world.runState = RunStep.REAL_MOTOR_EXEC 
                else: self.world.runState = RunStep.IMAGINARY_GENERATION               
            else:                
                #---do something                 
                if self.isMainRobot: 
                    print('Main Gen ',self.currGen)
                    self.world.runState = RunStep.REAL_MOTOR_EXEC
                else: 
                    print('Gen ',self.currGen)
                    self.world.runState = RunStep.IMAGINARY_MOTOR_EXEC
                self.currGen += 1
    def start(self):
        if not self.isMainRobot:
            self.world.setImaginaryRobotPositionAndAnglesToRealRobot()        
    def stop(self):
        self.currGen = 0  

# class Epoch:#to run g generations e number of times
#     def __init__(self, listOfRobots, parent):
#         self.robots = listOfRobots
#         #self.isMainRobot = (len(self.robots) == 1)            
#         self.world = parent
#         self.maxEpochs = 1 #increase if needed
#         self.currEpoch = 0
#     def run(self):        
#         if self.currEpoch == 0:
#             self.start()
#             self.currEpoch += 1   
#         else:    
#             if self.currEpoch == self.maxEpochs:
#                 self.stop()     
#             else:                        
#                 #---do something in this epoch
#                 print('Epoch ',self.currEpoch)
#                 self.currEpoch += 1
#         self.world.runState = RunStep.IMAGINARY_GENERATION                           
#     def start(self):
#         pass        
#     def stop(self):
#         self.currEpoch= 0  

#The world that has twins above which represent the imagination and run ComputationalIntelligence for a while before the 
#original robot takes the best motor rates and runs them
class ImaginationTwin(Worlds):#inherits
    def __init__(self, execLen, legCode): #def __init__(self, actions, execLen, legCode):        
        super(ImaginationTwin, self).__init__()
        self.legsCode = legCode
        self.maxMovtTime = execLen        
        #self.actionNetwork = actions
        self.screenWidth = 900
        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 1500 #overriding
        self.worldHeight = 300
        self.worldEndPos = self.worldWidth - 150
        self.imaginaryWorldYOffset = self.worldHeight 
        self.numRobots = 1        
        self.numImaginaryRobots = 5 #min 4 robots required for ComputationalIntelligence
        self.imaginaryRobots = []
        self.elevFromBottomWall = 0
        self.groundThickness = 10
        self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
        self.prevRobotPos = self.robotInitPos
        self.moveCameraAtThisDistDiff = 75
        self.imaginationColor = 80,80,80
        self.imaginationGroundColor = 80,130,80        
        self.runState = RunStep.IMAGINARY_MOTOR_EXEC
        self.nextNode = None 
        self.cons = Constants()       
        
    def initialize(self):
        super(ImaginationTwin, self).initialize()
        self.createDebris(self.elevFromBottomWall, self.imaginationColor)
        self.copyDebrisToImaginary(self.imaginaryWorldYOffset, self.imaginationColor)       
        self.createGround(0, self.elevFromBottomWall, self.groundColor)
        self.createWorldBoundary(0, self.imaginaryWorldYOffset, self.imaginationColor)       
        self.createGround(0, self.imaginaryWorldYOffset, self.imaginationGroundColor)        
        #ubp = self.robots[self.cons.mainRobotID].getUniqueBodyAngles()
        self.cumulativePosUpdateBy = Vec2d(0,0)      
        self.createImaginaryRobots()
        self.behaviour = SimpleDE(self.imaginaryRobots, self.robots)
        #---to run imaginary robots
        self.moveMotorsStateImagined = MoveMotors(self.imaginaryRobots, self)
        self.genStateImagined = Generation(self.imaginaryRobots, self)
        #self.epochStateImagined = Epoch(self.imaginaryRobots, self)
        #---to run main robot(s)
        self.moveMotorsStateReal = MoveMotors(self.robots, self)
        self.genStateReal = Generation(self.robots, self)        
        #self.epochStateReal = Epoch(self.robots, self)
        #---set execution state to start with         

        
#     def processRobot(self):
#         #---check if reached end of world
#         if self.robots[self.cons.mainRobotID].getPosition()[self.cons.xID] - self.cumulativePosUpdateBy[0] > self.worldEndPos:#reached end of world
#             self.runState = RunCode.STOP
#             print('SUCCESSFULLY REACHED END OF WORLD')
#             return False
#         resetMovtTime = True    
#         
#         if self.runState == RunCode.PAUSE_AND_SWITCH_TO_IMAGINATION:
#             self.setForImagination()            
#         
#         if self.runState == RunCode.EXPERIENCE:#appropriate action edge found so just execute what is in it
#             if self.sequenceLength > self.maxSequenceLength:
#                 self.robots[self.cons.mainRobotID].stopMotion()
#                 self.runState = RunCode.PAUSE_AND_SWITCH_TO_IMAGINATION         
#             else: #---run experience for each leg
#                 self.robots[self.cons.mainRobotID].setMotorRateForSequence(self.sequenceLength-1)
#                 self.sequenceLength += 1
# 
#         if self.runState == RunCode.CONTINUE:#bottom robot's movement
#             resetMovtTime = False
#             self.experienceInfoString()
#             mRates = sum([abs(x) for x in self.behaviour.fittestRobotsMotorRates])
#             if mRates == 0:#no movement because either no fit robot was found or imagination was not run yet
#                 self.setForImagination()
#             else:
#                 self.robots[self.cons.mainRobotID].setExperience(self.behaviour.fittestRobotsMotorRates)
#                 self.sequenceLength = 1 #should be at least 1   
#                 self.runState = RunCode.EXPERIENCE             
# 
#         
#         if self.runState == RunCode.IMAGINE:#imaginary robot's movement
#             resetMovtTime = self.runImagination()
#         return resetMovtTime
    
    def experienceInfoString(self):
        self.infoString = ""
        #self.infoStringStuckAndRoamingDir()
        self.infoString += ",  x: "+str(round(self.robots[self.cons.mainRobotID].getPosition()[self.cons.xID] - self.cumulativePosUpdateBy[0], self.decimalPrecision))
    
#     def setForImagination(self):
#         #self.robots[0].brain.movementThinking(self.robots[0].getPosition())
#         self.runState = RunCode.IMAGINE
#         self.behaviour.startNewEpoch()
#         self.setImaginaryRobotPositionAndAnglesToRealRobot()
#         self.sequenceLength = 1 #should be at least 1
#         self.gen = 0        
        
#     def runImagination(self):
#         resetMovtTime = True
#         if self.sequenceLength > self.maxSequenceLength:#completion of all experience length's
#             self.runState = RunCode.CONTINUE
#             self.infoString = ""
#             resetMovtTime = False                
#             return resetMovtTime
# 
#         rs = self.behaviour.run(self.sequenceLength)
#         if rs == RunCode.NEXTGEN:#reset for next generation
#             self.gen += 1            
#             if self.gen == self.maxGens:#completion of one epoch
#                 print('gen complete --------------------------------------------')
#                 #self.createNewActionNodes() 
#                 self.behaviour.storeExperienceOfFittestRobot()                                     
#                 #self.storeBestFitness() #TODO: DELETE THIS
#             self.deleteImaginaryRobots(); self.createImaginaryRobots()  
#             self.setImaginaryRobotPositionAndAnglesToRealRobot()          
#             self.behaviour.startNewGen()         
#             if self.gen == self.maxGens:#completion of one epoch
#                 self.sequenceLength += 1 
#                 self.gen = 0                      
#                 self.behaviour.startNewEpoch()
#         self.generateInfoString()  
#         return resetMovtTime
        

    def runWorld(self):
        clock = pygame.time.Clock()
        simulating = True        
        #prevTime = time.time()
        
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    #sys.exit(0)
                    simulating = False
                if event.type == KEYDOWN:
                    #if event.key == K_UP: self.cameraXY += Vec2d(0, -self.cameraMoveDist[1])
                    #if event.key == K_DOWN: self.cameraXY += Vec2d(0, self.cameraMoveDist[1])
                    if event.key == K_LEFT: self.moveCameraBy(self.cameraMoveDist[0])
                    if event.key == K_RIGHT: self.moveCameraBy(-self.cameraMoveDist[0])
            if not simulating: break #coz break within event for loop won't exit while
            #---camera follow robot
            robotMovedByX = self.prevRobotPos[self.cons.xID] - self.robots[self.cons.mainRobotID].getPosition()[self.cons.xID]            
            if abs(robotMovedByX) > self.moveCameraAtThisDistDiff:
                self.moveCameraBy(robotMovedByX)
            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for _ in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on camera focus
            self.updatePosition()
#             if self.prevFocusRobotID != self.focusRobotID: 
#                 self.updateColor()
#                 self.prevFocusRobotID = self.focusRobotID
            
                        
            if self.runState == RunStep.IMAGINARY_GENERATION: self.genStateImagined.run()
            if self.runState == RunStep.IMAGINARY_MOTOR_EXEC: self.moveMotorsStateImagined.run()                    
            #if self.runState == RunStep.IMAGINARY_EPOCH: self.epochStateImagined.run()
            if self.runState == RunStep.REAL_MOTOR_EXEC: self.moveMotorsStateReal.run()
            if self.runState == RunStep.REAL_GENERATION: self.genStateReal.run()
            
            #---if robot reaches goal, stop
            if self.robots[self.cons.mainRobotID].getPosition()[self.cons.xID] - self.cumulativePosUpdateBy[0] > self.worldEndPos:#reached end of world
                break
            
            #---draw all objects            
            self.draw()                
            
            #self.focusRobotXY = self.robots[self.focusRobotID].chassis_body.position#use getter
            clock.tick(self.fps)
#             if self.runState == RunCode.STOP: 
#                 break  
        
        #---actions to do after simulation
        #self.actionNetwork.saveNetwork() 
        
    def moveCameraBy(self, dist):
        self.cameraXY += Vec2d(dist, 0)
        self.prevRobotPos = self.robots[0].getPosition()        
        
    def createGround(self, groundX, groundY, grColor):
        self.createBox(groundX+self.worldWidth/2, groundY+self.wallThickness+self.wallThickness/2, self.worldWidth-2*self.wallThickness, self.wallThickness, grColor)

    def createDebris(self, groundY, debColor):
        debrisStartCol = 200; debrisMaxHt = 50; boxMinSz = 5; boxMaxSz = 30
        for _ in range(0, 100, 1):
            self.createBox(random.randint(debrisStartCol, self.worldWidth-2*self.wallThickness), random.randint(groundY+2*self.wallThickness, groundY+debrisMaxHt), random.randint(boxMinSz, boxMaxSz), random.randint(boxMinSz, boxMaxSz), debColor)        
        
    def copyDebrisToImaginary(self, groundY, debColor):
        for i in range(0, len(self.worldObjects), 1):
            self.createBox(self.worldObjects[i].body.position[0], groundY+self.worldObjects[i].body.position[1], self.worldObjects[i].body.width, self.worldObjects[i].body.height, debColor)        
    
    def createBox(self, x, y, wd, ht, colour):
        body = pymunk.Body(body_type = pymunk.Body.KINEMATIC); body.position = Vec2d(x, y); body.width = wd; body.height = ht
        shape = pymunk.Poly.create_box(body, (wd, ht)); shape.color = colour; shape.friction = 1.0; 
        self.space.add(shape); self.worldObjects.append(shape)  
        
    def setImaginaryRobotPositionAndAnglesToRealRobot(self):
        pos = self.robots[self.cons.mainRobotID].getPositions()
        angles = self.robots[self.cons.mainRobotID].getUniqueBodyAngles()
        for robo in self.imaginaryRobots:
            p = pos[:]; a = angles[:] #copying values instead of references
            robo.setBodyPositionAndAngles(p, a, Vec2d(0, self.imaginaryWorldYOffset))
            robo.stopMotion()
        
    def generateInfoString(self):
        genFittestRoboString = "-"; currFittestRoboString = "-"
        if self.behaviour.epochBestFitness > 0: genFittestRoboString = str(self.behaviour.epochFittestRobot)
        if self.behaviour.currentFittestRobot > 0: currFittestRoboString = str(self.behaviour.currentFittestRobot)        
        self.infoString = ""
        #self.infoStringStuckAndRoamingDir()
        self.infoString += ",  SeqLen: "+str(self.sequenceLength)+"/"+str(self.maxSequenceLength)+"  Gen: "+str(self.gen)+"/"+str(self.maxGens)
        self.infoString += "  SeqRep: "+str(self.behaviour.repeatSeq)+"/"+str(self.behaviour.maxSeqRepetitions)
        self.infoString += "  Seq: "+str(self.behaviour.seqNum+1)+"/"+str(self.sequenceLength)
        self.infoString += "  Fittest: "+str(currFittestRoboString)+" | "+str(genFittestRoboString)+"  Fit: "+str(self.behaviour.currentBestFitness)+" | "+str(self.behaviour.epochBestFitness)
        
#     def infoStringStuckAndRoamingDir(self):
#         disp = "-"
#         if self.robots[self.cons.mainRobotID].brain.stuck > 0: disp = "Y"
#         self.infoString += "Stuck: "+disp; disp = "-"
#         if self.robots[self.cons.mainRobotID].brain.roaming > 0: disp = "Y"
#         self.infoString += "  Roam: "+disp
#         #---get direction
#         dirn = self.robots[self.cons.mainRobotID].brain.direction
#         for d, v in self.robots[self.cons.mainRobotID].brain.ori.items():
#             if dirn == v: self.infoString += " Dir: "+d 
                
    def delete(self):
        super(ImaginationTwin, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
    
    def updatePosition(self):  
        updateBy = super(ImaginationTwin, self).updatePosition()    
        self.cumulativePosUpdateBy += updateBy
#         if self.behaviour.currentFittestRobot != self.focusRobotID:
#             self.focusRobotID = self.behaviour.currentFittestRobot
#         if self.behaviour.unfitThisFullGen[self.focusRobotID]:
#             self.focusRobotID = self.UNDETERMINED
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy   
            for obj in self.imaginaryRobots:
                obj.updatePosition(updateBy)                
                
    def updateColor(self):
        for obj in self.robots:
            obj.setNormalRobotColor()
#         for obj in self.imaginaryRobots:
#             if self.focusRobotID >= 0:
#                 if obj == self.imaginaryRobots[self.focusRobotID]: obj.setFocusRobotColor() 
#                 else: obj.setImaginaryRobotColor()
#             else: obj.setImaginaryRobotColor()                 
    
    def createImaginaryRobots(self):      
        for _ in range(0, self.numImaginaryRobots, 1):
            self.imaginaryRobots.append(RobotBody(self.space, self.robotInitPos + Vec2d(0, self.imaginaryWorldYOffset), self.legsCode))#deliberately placing it outside screen since it'll be brought back on screen in robot's position soon
            
    def deleteImaginaryRobots(self):
        for r in self.imaginaryRobots:
            r.delete()
        self.imaginaryRobots[:] = []    

                             
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class TestWorld(Worlds):#inherits
    def __init__(self, legsCode):
        self.legsCode = legsCode
        super(TestWorld, self).__init__()
        self.screenWidth = 900
        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 900 #overriding
        self.worldHeight = 620 #overriding        
        self.numRobots = 1 
        self.statsPos = Vec2d(15,15)
    
    def runWorld(self): #may get overridden in child class
        clock = pygame.time.Clock()
        simulating = True
        rot = 0.2
        r1 = self.robots[0]
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    sys.exit(0)
                if event.type == KEYDOWN:
                    if event.key == K_UP: self.cameraXY += Vec2d(0, -self.cameraMoveDist[1])
                    if event.key == K_DOWN: self.cameraXY += Vec2d(0, self.cameraMoveDist[1])
                    if event.key == K_LEFT: self.cameraXY += Vec2d(self.cameraMoveDist[0], 0)
                    if event.key == K_RIGHT: self.cameraXY += Vec2d(-self.cameraMoveDist[0], 0) 
                    if event.key == K_1: r1.legs[0].motor.rate = rot
                    if event.key == K_2: r1.legs[1].motor.rate = rot
                    if event.key == K_3: r1.legs[2].motor.rate = rot
                    if event.key == K_4: r1.legs[3].motor.rate = rot
                if event.type == KEYUP:
                    self.robots[0].stopMotion()
            
            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for x in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            self.infoString = []
            self.infoString.append("Tip1: "+str(r1.legs[0].getTip())+"Tip2: "+str(r1.legs[1].getTip())+"Tip3: "+str(r1.legs[2].getTip())+"Tip3: "+str(r1.legs[3].getTip()))
            self.infoString.append("Quadrants: "+str(r1.getLegQuadrants()))
            #---draw all objects
            self.draw()            
            clock.tick(self.fps)
    
    def initializeRobots(self):#overriding  
        widthSep = 100; heightSep = 100; counter = 0; sf = pymunk.ShapeFilter(group=1)
        for i in range(350, self.worldHeight, heightSep):
            if counter >= self.numRobots: break
            for j in range(450, self.worldWidth, widthSep):
                robo = LearningRobot(self.space, Vec2d(j, i), self.legsCode, None)
                self.robots.append(robo); x = robo.getPosition()[0]; y = robo.getPosition()[1]; w = 100
                for k in range(0, 100, self.robots[0].quadrantAccuracy):
                    l1 = pymunk.Segment(self.space.static_body, (x-w, y+k), (x+w, y+k), 0.1)
                    l2 = pymunk.Segment(self.space.static_body, (x-w, y-k), (x+w, y-k), 0.1)
                    l3 = pymunk.Segment(self.space.static_body, (x+k, y-w), (x+k, y+w), 0.1)
                    l4 = pymunk.Segment(self.space.static_body, (x-k, y-w), (x-k, y+w), 0.1)
                    l1.filter = sf; l2.filter = sf; l3.filter = sf; l4.filter = sf;   
                    l1.color = (100, 100, 100); l2.color = (100, 100, 100); l3.color = (100, 100, 100); l4.color = (100, 100, 100)
                    self.space.add(l1, l2, l3, l4)
                    for leg in robo.legs:
                        tip = leg.getTip()
#                         leg.tipBody = pymunk.Body(0, 0); leg.tipBody.position = leg.getTip()
#                         leg.tipShape = pymunk.Poly.create_box(leg.tipBody, (1,1))          
                        leg.tip = pymunk.Segment(self.space.static_body, tip, tip+(1,1), 0.1)
                        leg.tip.color = (0, 255, 0); self.space.add(leg.tip);#self.space.add(leg.tipBody, leg.tipShape)
                counter += 1 
                if counter >= self.numRobots: break
#         for robo in self.robots:
#             robo.setState(BrainState_RandomMovement(robo))

    def removeBoundary(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
                         
    def delete(self):
        super(TestWorld, self).delete()   
        for ob in self.worldObjects: self.space.remove(ob)
        self.worldObjects[:] = []  
        for ob in self.space: self.space.remove(ob)
     
    def updatePosition(self):  
        updateBy = super(TestWorld, self).updatePosition()    
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy 
#         for leg in self.robots[0].legs:
#             leg.tipBody.position = leg.getTip()

#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class FitnessStore:
    def __init__(self):
        self.fitness = [] 
    def addGenerationsBestFitness(self, fit): #list of highest fitness achieved by any robot in each gen of epoch 
        self.fitness.append(fit)
    def showFitnessAcrossIncreasingSeqLengths(self):
        genLen = len(self.fitness[0]); epochLen = len(self.fitness)
        self.fig = plt.figure() 
        self.fig.patch.set_facecolor('white')           
        plt.clf(); styles = ['-r','-b','-g','-c','-m','-y','-k','--r','--b','--g','--c','--m','--y','--k','-.r','-.b','-.g','-.c','-.m','-.y','-.k',':r',':b',':g',':c',':m',':y',':k']
        for e in range(0, epochLen, 1):
            plt.plot(np.linspace(1, genLen, genLen), self.fitness[e], styles[e], label='seqlen: '+str(e))
        plt.title('Fitness across'+str(epochLen)+' epochs for '+str(genLen)+' gen')
        plt.xlabel('generations', fontsize=12);plt.ylabel('fitness', fontsize=12)
        plt.rcParams['axes.facecolor'] = 'white';plt.rcParams['patch.facecolor'] = 'white'
        self.fig.canvas.toolbar.pack_forget() #remove bottom bar  
        plt.pause(0.001); plt.legend(loc='best');plt.show()           
    def showFitnessAcrossEpochs(self):
        self.fig = plt.figure() 
        self.fig.patch.set_facecolor('white')           
        genLen = len(self.fitness[0]); epochLen = len(self.fitness)
        avg = []
        for g in range(0, genLen, 1):
            tot = 0
            for e in range(0, epochLen, 1):
                tot += self.fitness[e][g]
            avg.append(tot / epochLen)
        plt.clf()
        plt.plot(np.linspace(1, genLen, genLen), avg)
        plt.title('Average best fitness across'+str(epochLen)+' epochs for '+str(genLen)+' gen')
        plt.xlabel('generations', fontsize=12);plt.ylabel('fitness', fontsize=12)
        self.fig.canvas.toolbar.pack_forget() #remove bottom bar  
        plt.pause(0.001); plt.show() 

class FlatGroundTraining(Worlds):#inherits
    def __init__(self, execLen, legsCode):
        super(FlatGroundTraining, self).__init__()
        self.legsCode = legsCode
        self.maxMovtTime = execLen
        self.screenWidth = 900 #overriding
        self.screenHeight = 620#320 #overriding
        self.worldHeight = 600#280 #overriding
        self.worldWidth = 20000 #overriding
        self.numRobots = 15 #min 4 robots required for ComputationalIntelligence
        self.elevFromBottomWall = 10
        self.groundThickness = 10
        self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
        self.elevFromBottomWall = 0 
        self.fitGraph = FitnessStore()
        self.genBestFit = []       
   
    def initialize(self):
        super(FlatGroundTraining, self).initialize() 
        self.createGround(0, self.elevFromBottomWall, self.groundColor)
        self.behaviour = DifferentialEvolution(self.robots)
        self.sequenceLength = 1 #Epoch. start seq len. Should start with anything from 1 to maxSequenceLength        
        self.gen = 0 #start gen. Starts with 0
        self.maxGens = 50
        self.maxSequenceLength = 10 #The number of dT times a leg is moved. Total epochs (not related to gen coz seqLen changes)
         
    def createGround(self, groundX, groundY, grColor):
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(groundX+self.worldWidth/2, groundY+self.wallThickness+self.wallThickness/2)
        shape = pymunk.Poly.create_box(body, (self.worldWidth-2*self.wallThickness, self.wallThickness)); shape.color = grColor; shape.friction = 1.0
        self.space.add(shape); self.worldObjects.append(shape)   
                 
    def processRobot(self):
        if self.sequenceLength > self.maxSequenceLength:#completion of all experience length's
            self.fitGraph.showFitnessAcrossIncreasingSeqLengths()
            self.fitGraph.showFitnessAcrossEpochs()
            return RunCode.STOP
         
        runCode = self.behaviour.run(self.sequenceLength)
        if runCode == RunCode.NEXTGEN:#reset for next generation
            self.genBestFit.append(self.behaviour.currentBestFitness)
            self.gen += 1
            self.deleteRobots(); self.initializeRobots()            
            self.behaviour.startNewGen()         
            if self.gen == self.maxGens:#completion of one epoch
                self.fitGraph.addGenerationsBestFitness(self.genBestFit)
                self.sequenceLength += 1 
                self.gen = 0; self.genBestFit = []
                self.behaviour.startNewEpoch()
        #---info dashboard
        genFittestRoboString = "-"; currFittestRoboString = "-"
        if self.behaviour.epochBestFitness > 0: genFittestRoboString = str(self.behaviour.epochFittestRobot)
        if self.behaviour.currentFittestRobot > 0: currFittestRoboString = str(self.behaviour.currentFittestRobot)
        self.infoString = "SeqLen: "+str(self.sequenceLength)+"/"+str(self.maxSequenceLength)+"  Gen: "+str(self.gen)+"/"+str(self.maxGens)
        self.infoString += "  SeqRep: "+str(self.behaviour.repeatSeq)+"/"+str(self.behaviour.maxSeqRepetitions)
        self.infoString += "  Seq: "+str(self.behaviour.seqNum+1)+"/"+str(self.sequenceLength)
        self.infoString += "  Fittest: "+str(currFittestRoboString)+" | "+str(genFittestRoboString)+"  Fit: "+str(self.behaviour.currentBestFitness)+" | "+str(self.behaviour.epochBestFitness)
         
    def delete(self):
        super(FlatGroundTraining, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
     
    def updatePosition(self):  
        updateBy = super(FlatGroundTraining, self).updatePosition()    
        if self.behaviour.currentFittestRobot != self.focusRobotID:
            self.focusRobotID = self.behaviour.currentFittestRobot
        if self.behaviour.unfitThisFullGen[self.focusRobotID]:
            self.focusRobotID = self.UNDETERMINED
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy     
                            
                            
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#A world where multiple robots learn actions to generate a basic action network
class Heaven(Worlds):#inherits
    def __init__(self, legsCode): #def __init__(self, legsCode, actionNet):
        self.legsCode = legsCode
        #self.actions = actionNet        
        super(Heaven, self).__init__()
        self.screenWidth = 900
        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 900 #overriding
        self.worldHeight = 620 #overriding        
        self.numRobots = 24 #378
    
    def runWorld(self): #may get overridden in child class
        clock = pygame.time.Clock()
        simulating = True
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    simulating = False #sys.exit(0)
                if event.type == KEYDOWN:
                    if event.key == K_UP: self.cameraXY += Vec2d(0, -self.cameraMoveDist[1])
                    if event.key == K_DOWN: self.cameraXY += Vec2d(0, self.cameraMoveDist[1])
                    if event.key == K_LEFT: self.cameraXY += Vec2d(self.cameraMoveDist[0], 0)
                    if event.key == K_RIGHT: self.cameraXY += Vec2d(-self.cameraMoveDist[0], 0)                    
#                     if event.key == K_n: 
#                         print('Getting ready to display action network...'); 
#                         self.actions.displayNetwork()
            if not simulating: 
                break               
            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for x in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            for robo in self.robots:
                robo.run()
            #---draw all objects
            self.draw()            
            clock.tick(self.fps)
        #---actions to do after simulation
        #self.actions.saveNetwork() 

    def initialize(self):
        super(Heaven, self).initialize() 
        self.removeBoundary()
    
    def initializeRobots(self):#overriding  
        widthSep = 100; heightSep = 100; counter = 0
        for i in range(100, self.worldHeight, heightSep):
            if counter >= self.numRobots: break
            for j in range(100, self.worldWidth, widthSep):
                self.robots.append(LearningRobot(self, Vec2d(j, i), self.legsCode, False))
                counter += 1 
                if counter >= self.numRobots: break
        for robo in self.robots:
            robo.setState(BrainState_RandomMovement(robo))

    def removeBoundary(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
                         
    def delete(self):
        super(Heaven, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
     
    def updatePosition(self):  
        updateBy = super(Heaven, self).updatePosition()    
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy     
                            
                            
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#The world where the tips of the limbs and contact with terrain is visualized to show what a blind
#robot actually "sees"
class ActualImagination(Worlds):#inherits
    def __init__(self, legCode):    #def __init__(self, legCode, actionNet):        
        super(ActualImagination, self).__init__()
        self.legsCode = legCode      
        #self.actions = actionNet
        self.screenWidth = 900
        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 3000 #overriding
        self.worldHeight = 300
        self.sensedObjects = []
        #---create array representation of world for imagination
        for r in range(0, self.worldWidth, 1):
            col = []
            for c in range(0, self.worldHeight, 1):
                col.append(0)
            self.sensedObjects.append(col)
        self.imaginaryWorldYOffset = self.worldHeight 
        self.numRobots = 1
        #self.numImaginaryRobots = 4 #min 4 robots required for ComputationalIntelligence
        self.imaginaryRobots = []
        self.elevFromBottomWall = 0
        self.groundThickness = 10
        self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
        self.imaginationColor = 100,100,100
        self.imaginationGroundColor = 100,150,100
        self.runState = RunCode.CONTINUE
        self.nextNode = None 
        self.cons = Constants()       
        
    def initialize(self):
        super(ActualImagination, self).initialize()       
        self.createGround(0, self.elevFromBottomWall, self.groundColor)
        self.createWorldBoundary(0, self.imaginaryWorldYOffset, self.imaginationColor)
        self.createFewObjects()       
        #self.createGround(0, self.imaginaryWorldYOffset, self.imaginationGroundColor)        
        self.cumulativePosUpdateBy = Vec2d(0,0)            
    
    def runWorld(self): #may get overridden in child class
        clock = pygame.time.Clock()
        simulating = True
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    simulating = False #sys.exit(0)
                if event.type == KEYDOWN:
                    if event.key == K_UP: self.cameraXY += Vec2d(0, -self.cameraMoveDist[1])
                    if event.key == K_DOWN: self.cameraXY += Vec2d(0, self.cameraMoveDist[1])
                    if event.key == K_LEFT: self.cameraXY += Vec2d(self.cameraMoveDist[0], 0)
                    if event.key == K_RIGHT: self.cameraXY += Vec2d(-self.cameraMoveDist[0], 0)                    
#                     if event.key == K_n: 
#                         print('Getting ready to display action network...'); 
#                         self.actions.displayNetwork()
            if not simulating: 
                break               
            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for x in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            for robo in self.robots:
                robo.run()
            #---draw all objects
            self.draw()            
            clock.tick(self.fps)
        #---actions to do after simulation
        #self.actions.saveNetwork() 
    
    def initializeRobots(self):#overriding  
        widthSep = 100; heightSep = 100; counter = 0
        for i in range(100, self.worldHeight, heightSep):
            if counter >= self.numRobots: break
            for j in range(100, self.worldWidth, widthSep):
                self.robots.append(LearningRobot(self, Vec2d(j, i), self.legsCode)) #self.robots.append(LearningRobot(self, Vec2d(j, i), self.legsCode, self.actions))
                counter += 1 
                if counter >= self.numRobots: break
        for robo in self.robots:
            robo.makeRobotDynamic()
            robo.setState(BrainState_WhatHappensIfITryThis(robo))

    def removeBoundary(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
                         
    def delete(self):
        super(ActualImagination, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
     
    def updatePosition(self):  
        updateBy = super(ActualImagination, self).updatePosition()    
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy     
                                    
    def createGround(self, groundX, groundY, grColor):
        self.createBox(groundX+self.worldWidth/2, groundY+self.wallThickness+self.wallThickness/2, self.worldWidth-2*self.wallThickness, self.wallThickness, grColor)

    def createFewObjects(self):
        w = 10; h = 40
        self.createBox(50, 50, w, w, self.groundColor)
        self.createBox(120, 60, w, h, self.groundColor)
        
    def createBox(self, x, y, wd, ht, colour):
        body = pymunk.Body(body_type = pymunk.Body.KINEMATIC); body.position = Vec2d(x, y); body.width = wd; body.height = ht
        shape = pymunk.Poly.create_box(body, (wd, ht)); shape.color = colour; shape.friction = 1.0; 
        self.space.add(shape); self.worldObjects.append(shape)   
        
                                   
