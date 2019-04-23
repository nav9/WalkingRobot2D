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
from WalkingRobot import RobotBody
from Behaviours import DifferentialEvolution, ImaginationDifferentialEvolution, RunCode

class Worlds(object):
    def __init__(self):
        #self.focusRobotXY = Vec2d(0, 0)#will be overridden below        
        self.screen = None
        self.draw_options = None       
        
        #NOTE: Pymunk physics coordinates start from the lower right-hand corner of the screen
        self.screenWidth = 1300; #can get overridden in child class
        self.screenHeight = 400 #keep at at least 350. Can get overridden in child class
        self.boundaryObjects = []
        self.worldObjects = []
        self.worldWidth = 1000 #may get overridden in child class
        self.worldHeight = 500 #may get overridden in child class
        self.wallThickness = 15
        self.boundaryColor = 170,170,170
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
        
    def createBoundary(self, worldX, worldY, bouColor):
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
        
    def initialize(self):
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50
        self.maxMovtTime = 10 #how often in time the sequences of the robot get executed
        self.movtTime = self.fps #start value of movt time. Can be anything from 0 to maxMovtTime
        self.iterations = 20        
        #self.space.damping = 0.999 
        #self.focusRobotChanged = False
        self.prevFocusRobotID = self.UNDETERMINED #At first none of the robots will be in focus since fitness hasn't been determined
        self.focusRobotID = self.UNDETERMINED #At first none of the robots will be in focus since fitness hasn't been determined
        self.infoString = ""   
        
        self.createBoundary(0, 0, self.boundaryColor)
            
        pygame.init()
        pygame.mixer.quit()#disable sound output that causes annoying sound effects if any other external music player is playing
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight), self.display_flags)
        self.font = pygame.font.SysFont("Arial", 14)
        #width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 150,150,150

        self.initializeRobots()
        if len(self.robots) <= 0: print('Create at least one robot'); return


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
        for i in range(0, self.numRobots, 1):
            self.robots.append(RobotBody(self.space, self.robotInitPos))             
    
    def displayStats(self, displayStr):
        self.screen.blit(self.font.render(displayStr, 1, THECOLORS["green"]), self.statsPos)

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
            for x in range(self.iterations): #iterations to get a more stable simulation
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

# class FlatGroundTraining(Worlds):#inherits
#     def __init__(self):
#         super(FlatGroundTraining, self).__init__()
#         self.worldWidth = 2000 #overriding
#         self.numRobots = 4 #min 4 robots required for DE
#         self.elevFromBottomWall = 10
#         self.groundThickness = 10
#         self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
#   
#     def initialize(self):
#         super(FlatGroundTraining, self).initialize()
#         groundX = self.worldX+self.wallThickness/2; groundLen = self.worldWidth-2 * self.wallThickness; groundY = self.elevFromBottomWall
#         ground_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); groundStart = Vec2d(groundX, groundY); groundPosition = Vec2d(groundX+groundLen, groundY)
#         ground_body.position = groundStart
#         ground_shape = pymunk.Segment(ground_body, groundStart, groundPosition, self.groundThickness); ground_shape.friction = 1.0        
#         self.space.add(ground_shape); self.worldObjects.append(ground_shape)  
#         self.behaviour = DifferentialEvolution(self.robots)
#         self.sequenceLength = 1 #start seq len. Should start with anything from 1 to maxSequenceLength
#         self.maxSequenceLength = 10 #The number of dT times a leg is moved
#         self.gen = 0 #start gen
#         self.maxGens = 20 
#         
#     def processRobot(self):
#         if self.sequenceLength > self.maxSequenceLength:#completion of all experience length's
#             return RunCode.STOP
#         
#         runCode = self.behaviour.run(self.sequenceLength)
#         if runCode == RunCode.NEXTGEN:#reset for next generation
#             self.gen += 1
#             self.deleteRobots(); self.initializeRobots()            
#             self.behaviour.startNewGen()         
#             if self.gen == self.maxGens:#completion of one epoch
#                 self.sequenceLength += 1 
#                 self.gen = 0
#                 self.behaviour.startNewEpoch()
#         #---info dashboard
#         genFittestRoboString = "-"; currFittestRoboString = "-"
#         if self.behaviour.epochBestFitness > 0: genFittestRoboString = str(self.behaviour.epochFittestRobot)
#         if self.behaviour.currentFittestRobot > 0: currFittestRoboString = str(self.behaviour.currentFittestRobot)
#         self.infoString = "SeqLen: "+str(self.sequenceLength)+"/"+str(self.maxSequenceLength)+"  Gen: "+str(self.gen)+"/"+str(self.maxGens)
#         self.infoString += "  SeqRep: "+str(self.behaviour.repeatSeq)+"/"+str(self.behaviour.maxSeqRepetitions)
#         self.infoString += "  Seq: "+str(self.behaviour.seqNum+1)+"/"+str(self.sequenceLength)
#         self.infoString += "  Fittest: "+str(currFittestRoboString)+" | "+str(genFittestRoboString)+"  Fit: "+str(self.behaviour.currentBestFitness)+" | "+str(self.behaviour.epochBestFitness)
#         
#     def delete(self):
#         super(FlatGroundTraining, self).delete()   
#         for ob in self.worldObjects:
#             self.space.remove(ob)
#         self.worldObjects[:] = []  
#     
#     def updatePosition(self):  
#         updateBy = super(FlatGroundTraining, self).updatePosition()    
# #         self.behaviour.updateChassisBodyPositionForFitness(updateBy[0]) 
#         if self.behaviour.currentFittestRobot != self.focusRobotID:
#             self.focusRobotID = self.behaviour.currentFittestRobot
#         if self.behaviour.unfitThisFullGen[self.focusRobotID]:
#             self.focusRobotID = self.UNDETERMINED
#         if updateBy != (0, 0):
#             for ob in self.worldObjects:
#                 ob.body.position += updateBy     
                            
                    
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class ImaginationTwin(Worlds):#inherits
    def __init__(self, actions):        
        super(ImaginationTwin, self).__init__()
        self.actionNetwork = actions

        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 2000 #overriding
        self.worldHeight = 300
        self.imaginaryWorldYOffset = self.worldHeight 
        self.numRobots = 1
        self.numImaginaryRobots = 4 #min 4 robots required for DE
        self.imaginaryRobots = []
        self.elevFromBottomWall = 0
        self.groundThickness = 10
        self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
        self.imaginationColor = 100,100,100
        self.imaginationGroundColor = 100,150,100
        self.groundColor = 0,170,0
        self.runState = RunCode.CONTINUE
        self.nextNode = None
        
    def createGround(self, groundX, groundY, grColor):
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(groundX+self.worldWidth/2, groundY+self.wallThickness+self.wallThickness/2)
        shape = pymunk.Poly.create_box(body, (self.worldWidth-2*self.wallThickness, self.wallThickness)); shape.color = grColor; shape.friction = 1.0
        self.space.add(shape); self.worldObjects.append(shape)        
        
    def initializeImaginaryRobots(self):      
        for i in range(0, self.numImaginaryRobots, 1):
            self.imaginaryRobots.append(RobotBody(self.space, self.robotInitPos + Vec2d(0, self.imaginaryWorldYOffset)))            
            
    def deleteImaginaryRobots(self):
        for r in self.imaginaryRobots:
            r.delete()
        self.imaginaryRobots[:] = []            
    
    def initialize(self):
        #---actual world (the world seen below)
        super(ImaginationTwin, self).initialize()
        self.createGround(0, self.elevFromBottomWall, self.groundColor)
        #---imaginary world (the world seen above)
        self.createBoundary(0, self.imaginaryWorldYOffset, self.imaginationColor)
        self.createGround(0, self.imaginaryWorldYOffset, self.imaginationGroundColor)
        ubp = self.robots[0].getUniqueBodyPosition()
        self.actionNetwork.addNode(ubp)
        self.robots[0].currentActionNode = ubp        
        self.initializeImaginaryRobots()
        self.behaviour = ImaginationDifferentialEvolution(self.imaginaryRobots)
        self.sequenceLength = 1 #start seq len. Should start with anything from 1 to maxSequenceLength
        self.maxSequenceLength = 1 #The number of dT times a leg is moved
        self.gen = 0 #start gen
        self.maxGens = 5          
        
    def processRobot(self):
        if self.runState == RunCode.EXPERIENCE:
            print('runcode is EXPERIENCE')
            if self.sequenceLength > self.maxSequenceLength:
                self.robots[0].stopMotion()
                self.robots[0].currentActionNode = self.nextNode
                self.runState = RunCode.CONTINUE
                print('no experience, so CONTINUE')
            else: 
                print('runExperienceForEachLeg')
                #---run experience for each leg
                self.robots[0].setMotorRateForSequence(self.sequenceLength-1)
                self.sequenceLength += 1
                
        if self.runState == RunCode.CONTINUE:#bottom robot's movement
            print('runcode is CONTINUE. Get best successor node')
            successors = self.actionNetwork.getBestSuccessorNode(self.robots[0].currentActionNode)
            if successors == None:#no successor node found, so start imagining
                print('no successor. runcode now IMAGINE and start new epoch')
                self.runState = RunCode.IMAGINE
                self.behaviour.startNewEpoch()
                self.sequenceLength = 1 #should be at least 1
                self.gen = 0
                #*************position all imaginary robots at angles of robot
            else:#successor found so get the experience action to perform
                print('successor found')
                greatestWeight = -1; imaginedExperience = []
                for successor in successors:
                    self.nextNode = successor
                    edge = self.actionNetwork.getEdge(self.robots[0].currentActionNode, self.nextNode)
                    for e in edge:#choose next node as per best weight
                        wt = edge[e]['weight']
                        print('weight='+str(wt))
                        if wt > greatestWeight:
                            greatestWeight = wt
                            imaginedExperience = list(edge[e]['experience'])
                            print('exp='+str(imaginedExperience))
                        else: continue
                #---make preparations to run the node's experience
                self.robots[0].setExperience(imaginedExperience)
                self.sequenceLength = 1 #should be at least 1
                print('runcode is now EXPERIENCE')
                
                self.runState = RunCode.EXPERIENCE
        if self.runState == RunCode.IMAGINE:#imaginary robot's movement
            print('imaginaion being run')
            self.runImagination()
    
    def runImagination(self):
        if self.sequenceLength > self.maxSequenceLength:#completion of all experience length's
            self.runState = RunCode.CONTINUE
            self.infoString = ''  
            print('imagination done. Continuing now')          
            return
        
        rs = self.behaviour.run(self.sequenceLength)
        if rs == RunCode.NEXTGEN:#reset for next generation
            self.gen += 1
            self.createNewActionNodeIfPossible()
            self.deleteImaginaryRobots(); self.initializeImaginaryRobots()            
            self.behaviour.startNewGen()         
            if self.gen == self.maxGens:#completion of one epoch
                self.sequenceLength += 1 
                self.gen = 0
                self.behaviour.startNewEpoch()
        self.generateInfoString()        
                
    def createNewActionNodeIfPossible(self):
        if False in self.behaviour.unfitThisFullGen:
            maxi = 0; fittestImaginaryRobot = -1
            for i in range(0, len(self.behaviour.unfitThisFullGen), 1):
                if not self.behaviour.unfitThisFullGen[i] and self.behaviour.fit[i] > maxi:
                        maxi = self.behaviour.fit[i]
                        fittestImaginaryRobot = i
            if fittestImaginaryRobot >= 0:
                expe = self.imaginaryRobots[fittestImaginaryRobot].getValues()
                node = self.imaginaryRobots[fittestImaginaryRobot].getUniqueBodyPosition()
                self.actionNetwork.addEdge(self.robots[0].currentActionNode, node, maxi, expe)
        
        
    def generateInfoString(self):
        genFittestRoboString = "-"; currFittestRoboString = "-"
        if self.behaviour.epochBestFitness > 0: genFittestRoboString = str(self.behaviour.epochFittestRobot)
        if self.behaviour.currentFittestRobot > 0: currFittestRoboString = str(self.behaviour.currentFittestRobot)        
        self.infoString = "SeqLen: "+str(self.sequenceLength)+"/"+str(self.maxSequenceLength)+"  Gen: "+str(self.gen)+"/"+str(self.maxGens)
        self.infoString += "  SeqRep: "+str(self.behaviour.repeatSeq)+"/"+str(self.behaviour.maxSeqRepetitions)
        self.infoString += "  Seq: "+str(self.behaviour.seqNum+1)+"/"+str(self.sequenceLength)
        self.infoString += "  Fittest: "+str(currFittestRoboString)+" | "+str(genFittestRoboString)+"  Fit: "+str(self.behaviour.currentBestFitness)+" | "+str(self.behaviour.epochBestFitness)
                
    def delete(self):
        super(ImaginationTwin, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
    
    def updatePosition(self):  
        updateBy = super(ImaginationTwin, self).updatePosition()    
#         self.behaviour.updateChassisBodyPositionForFitness(updateBy[0]) 
        if self.behaviour.currentFittestRobot != self.focusRobotID:
            self.focusRobotID = self.behaviour.currentFittestRobot
        if self.behaviour.unfitThisFullGen[self.focusRobotID]:
            self.focusRobotID = self.UNDETERMINED
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy   
                
    def updateColor(self):
        for obj in self.robots:
            obj.setNormalRobotColor()
        for obj in self.imaginaryRobots:
            if self.focusRobotID >= 0:
                if obj == self.imaginaryRobots[self.focusRobotID]: obj.setFocusRobotColor() 
                else: obj.setImaginaryRobotColor()
            else: obj.setImaginaryRobotColor()                 
    
    def runWorld(self):
        self.runState = RunCode.CONTINUE
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

            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for x in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on camera focus
            self.updatePosition()
            if self.prevFocusRobotID != self.focusRobotID: 
                self.updateColor()
                self.prevFocusRobotID = self.focusRobotID
            if self.movtTime == 0:
                self.processRobot()
                self.movtTime = self.maxMovtTime
            else:
                self.movtTime -= 1
            
            #---draw all objects            
            self.draw()
            
            #self.focusRobotXY = self.robots[self.focusRobotID].chassis_body.position#use getter
            clock.tick(self.fps)
            if self.runState == RunCode.STOP:
                break    
            