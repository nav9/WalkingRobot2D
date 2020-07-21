# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import sys
import time
import math
import pygame
import random
import pymunk
import statistics
import numpy as np
from pymunk import Vec2d
import pymunk.pygame_util
from pygame.color import *
from pygame.locals import *
from pygame.locals import *
from StatesAndSensors import *
import matplotlib.pyplot as plt
from WalkingRobot import RobotBody
from WalkingRobot import Constants
from LearningRobot import LearningRobot
from pymunk.shape_filter import ShapeFilter
from Analytics import TestAnalyticsForMovementAccuracy, FileOperations
from ComputationalIntelligence import SimpleDE, RunCode, RandomBest, SimplePSO

class RunStep:
    IMAGINARY_MOTOR_EXEC = 0
    IMAGINARY_GENERATION = 1
    REAL_MOTOR_EXEC = 2
    REAL_GENERATION = 3   
    
class RunCI:
    RANDOM = 'RANDOM'
    DE = 'DE'
    PSO = 'PSO'
    
class Terrains:
    FLAT_GROUND = 'FLAT_GROUND' 
    RANDOM_BOXES_LOW_DENSE = 'RANDOM_BOXES_LOW_DENSE'
    RANDOM_SPHERES_LOW_DENSE = 'RANDOM_SPHERES_LOW_DENSE'
    STAIRCASE_SINGLE_RIGHTWARD = 'STAIRCASE_SINGLE_RIGHTWARD'
    STEEPLE_CHASE = 'STEEPLE_CHASE'
    
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
        self.highFriction = 20
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
        self.font = pygame.font.SysFont("arial", 16)
        #width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 140,140,140
        #self.draw_options.flags = pymunk.SpaceDebugDrawOptions.DRAW_SHAPES
        #self.draw_options.flags |= pymunk.SpaceDebugDrawOptions.DRAW_COLLISION_POINTS

        self.initializeRobots()
        if len(self.robots) <= 0: print('Create at least one robot'); return

    def createWorldBoundary(self, worldX, worldY, bouColor):
        #---top boundary        
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.worldWidth/2, worldY+self.worldHeight-self.wallThickness/2)
        shape = pymunk.Poly.create_box(body, (self.worldWidth, self.wallThickness)); shape.color = bouColor; shape.friction = self.highFriction
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---bottom boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.worldWidth/2, worldY+self.wallThickness/2) 
        shape = pymunk.Poly.create_box(body, (self.worldWidth, self.wallThickness)); shape.color = bouColor; shape.friction = self.highFriction
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---left boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.wallThickness/2, worldY+self.worldHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.worldHeight)); shape.color = bouColor; shape.friction = self.highFriction
        self.space.add(shape); self.boundaryObjects.append(shape)
        #---right boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(worldX+self.worldWidth-self.wallThickness/2, worldY+self.worldHeight/2)
        shape = pymunk.Poly.create_box(body, (self.wallThickness, self.worldHeight)); shape.color = bouColor; shape.friction = self.highFriction
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
            self.robots.append(RobotBody(self.space, self.robotInitPos, self.legsCode, self.fps))             
    
    def displayStats(self, displayStr):
        if isinstance(displayStr, str): self.screen.blit(self.font.render(displayStr, 1, THECOLORS["gray"]), self.statsPos)
        else:
            sep = 15
            for i in range(0,len(displayStr),1): self.screen.blit(self.font.render(displayStr[i], 1, THECOLORS["gray"]), self.statsPos+(0,i*sep))

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
    
class MoveMotors:#to move the motors for time n*dT, where n is the number of frames and dT is the frame duration
    def __init__(self, listOfRobots, parent):
        self.robots = listOfRobots    
        self.isMainRobot = (len(self.robots) == 1)
        self.world = parent
        self.maxDuration = 50 #this duration will get overridden at start 
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
                #has to exit run() function now. Should not increment currDuration
            else:
                #---do something 
                self.currDuration += 1
    def start(self):
        for robo in self.robots:
            robo.startMotion()
    def stop(self):
        self.currDuration = 0 #ready for next movement when state switches back to this object
        for robo in self.robots:
            robo.stopMotion()
            if self.isMainRobot:
                robo.setLegMotorRates([]) #passing empty list sets rate store list to zeroes            

class Generation:#to run MoveMotors for g generations where each g = n*dT
    def __init__(self, listOfRobots, parent):
        self.robots = listOfRobots
        self.isMainRobot = (len(self.robots) == 1)
        self.world = parent  
        if self.isMainRobot: self.maxGens = 1 
        else: self.maxGens = 5
        self.currGen = 0
        if not self.isMainRobot:
            if self.world.runWhichCI == RunCI.RANDOM: self.CI = RandomBest(self.robots)
            if self.world.runWhichCI == RunCI.DE: self.CI = SimpleDE(self.robots)
            if self.world.runWhichCI == RunCI.PSO: self.CI = SimplePSO(self.robots)

    def run(self):        
        if self.currGen == 0:#first generation
            for robo in self.robots:
                robo.makeRobotDynamic()
            if not self.isMainRobot:
                self.CI.reinitialize()        
        if self.currGen == self.maxGens:#this is the end of the epoch
            #---do whatever is done at end of a generation
            self.stop()
            for robo in self.robots:
                robo.makeRobotStatic()            
            if self.isMainRobot: 
                self.world.runState = RunStep.IMAGINARY_GENERATION 
            else: 
                self.world.runState = RunStep.REAL_GENERATION
            #has to exit this run() function now            
        else:
            if not self.isMainRobot:
                self.CI.run()
            self.start()
            #---state switching etc
            if self.isMainRobot:
                for robo in self.robots:
                    robo.setLegMotorRates(self.world.genStateImagined.CI.motorRatesOfFittest)
                self.world.runState = RunStep.REAL_MOTOR_EXEC
            else: 
                self.world.runState = RunStep.IMAGINARY_MOTOR_EXEC
            self.currGen += 1                  
    def start(self):
        if not self.isMainRobot:
            self.world.setImaginaryRobotPositionAndAnglesToRealRobot()
    def stop(self):
        self.currGen = 0
    def getInfoString(self):
        whichGen = ("-" if self.currGen<=0 else str(self.currGen)) + '/' + ("-" if self.currGen<=0 else str(self.maxGens)) 
        return "Gen: " + whichGen + self.CI.getInfoString()
    def getFittestRobot(self):
        return self.CI.getFittestRobot()

class ShapeTypes:
    RECTANGLE = 1
    CIRCLE = 2
    
class ShapeProperties:
    COL = 'COL'
    ROW = 'ROW'
    WIDTH = 'WIDTH'
    HEIGHT = 'HEIGHT'
    RADIUS = 'RADIUS'
        
#The world that has twins above which represent the imagination and run ComputationalIntelligence for a while before the 
#original robot takes the best motor rates and runs them
class ImaginationTwin(Worlds):#inherits
    def __init__(self, legCode, runWhichCI, runWhichTerrain, trialNum): #def __init__(self, actions, execLen, legCode):        
        super(ImaginationTwin, self).__init__()
        self.legsCode = legCode
        self.runWhichCI = runWhichCI
        self.runWhichTerrain = runWhichTerrain
        self.screenWidth = 900
        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 900 #overriding
        self.worldHeight = 300
        self.finishLine = self.worldWidth - 700 
        self.imaginaryWorldYOffset = self.worldHeight 
        self.numRobots = 1        
        self.numImaginaryRobots = 4 #min 4 robots required for ComputationalIntelligence
        self.imaginaryRobots = []
        self.debrisElevFromBottomWall = 0
        self.groundThickness = 10
        self.robotInitPos = Vec2d(100, 100) #overrides the base class pos
        self.prevRobotPos = self.robotInitPos
        self.moveCameraAtThisDistDiff = 75
        self.imaginationColor = 80,80,80
        self.imaginationGroundColor = 80,130,80     
        self.finishLineColor = 225, 200 , 10    
        self.runState = RunStep.IMAGINARY_GENERATION
        self.robotBodyShapeFilter = pymunk.ShapeFilter(group = 1) #to prevent collisions between robot and objects
        self.cons = Constants()
        self.trialNumber = trialNum #Used when multi-trials are being run. If "None", the trial uses a randomized terrain. If it has a value, the trial either loads a previously stored terrain from a file or if none exists, it creates a terrain for that trial
        self.fileOps = FileOperations()
        
    def initialize(self):
        super(ImaginationTwin, self).initialize()
        if self.runWhichTerrain == Terrains.FLAT_GROUND: pass #no need to create any obstacles
        if self.runWhichTerrain == Terrains.RANDOM_BOXES_LOW_DENSE: self.createTerrainRandomBoxesLowDense()
        if self.runWhichTerrain == Terrains.RANDOM_SPHERES_LOW_DENSE: self.createSpheresTerrain()
        if self.runWhichTerrain == Terrains.STAIRCASE_SINGLE_RIGHTWARD: self.createStaircaseTerrain()
        if self.runWhichTerrain == Terrains.STEEPLE_CHASE: self.createTerrainBoxesInRowWithSpaces()

        #self.replicateDebrisToImaginary(self.imaginaryWorldYOffset, self.imaginationColor)       
        self.createGround(0, self.debrisElevFromBottomWall, self.groundColor)
        self.createWorldBoundary(0, self.imaginaryWorldYOffset, self.imaginationColor) 
        self.__addFinishLine__()      
        self.createGround(0, self.imaginaryWorldYOffset, self.imaginationGroundColor)        
        self.cumulativePosUpdateBy = Vec2d(0,0)      
        self.createImaginaryRobots()
        #---to run imaginary robots
        self.moveMotorsStateImagined = MoveMotors(self.imaginaryRobots, self)
        self.genStateImagined = Generation(self.imaginaryRobots, self)
        #---to run main robot(s)
        self.moveMotorsStateReal = MoveMotors(self.robots, self)
        self.genStateReal = Generation(self.robots, self)        

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
            #self.__makeCameraFollowRobot__()
            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for _ in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on camera focus
            self.updatePosition()
            self.updateColor()
            
            if self.runState == RunStep.IMAGINARY_GENERATION: self.genStateImagined.run()
            if self.runState == RunStep.IMAGINARY_MOTOR_EXEC: self.moveMotorsStateImagined.run()                    
            if self.runState == RunStep.REAL_MOTOR_EXEC: self.moveMotorsStateReal.run()
            if self.runState == RunStep.REAL_GENERATION: self.genStateReal.run()
            self.generateInfoString()
            
            #---if robot reaches goal, stop
            if self.robots[self.cons.mainRobotID].getPosition()[self.cons.xID] - self.cumulativePosUpdateBy[0] > self.finishLine:#reached end of world
                print('Crossed finish line. Congratulations!')
                break
            
            #---draw all objects            
            self.draw()                            
            clock.tick(self.fps)

    #--------------------------------------------------------------------------------------------
    #------------------------------------ helper functions --------------------------------------
    #--------------------------------------------------------------------------------------------
    
    def moveCameraBy(self, dist):
        self.cameraXY += Vec2d(dist, 0)
        self.prevRobotPos = self.robots[0].getPosition()

    def __makeCameraFollowRobot__(self):
        robotMovedByX = self.prevRobotPos[self.cons.xID] - self.robots[self.cons.mainRobotID].getPosition()[self.cons.xID]            
        if abs(robotMovedByX) > self.moveCameraAtThisDistDiff:
            self.moveCameraBy(robotMovedByX)                            
        
    def __addFinishLine__(self):
        self.createBox(self.finishLine, self.wallThickness+142, 2, self.worldHeight-47, self.finishLineColor, self.robotBodyShapeFilter)
    
    def createGround(self, groundX, groundY, grColor):
        self.createBox(groundX+self.worldWidth/2, groundY+self.wallThickness+self.wallThickness/2, self.worldWidth-2*self.wallThickness, self.wallThickness, grColor, None)
        
    def createTerrainObjects(self, terrainObjects): #terrainObjects = {RECTANGLE: [{COL: val}, {ROW: val} ...], CIRCLE: [{}, {} ...]}
        for shapeType in terrainObjects:#find rectangles or circles
            for o in terrainObjects[shapeType]:#iterate list of dicts that represent objects              
                if shapeType == ShapeTypes.RECTANGLE:           
                    self.createBox(o[ShapeProperties.COL], o[ShapeProperties.ROW], o[ShapeProperties.WIDTH], o[ShapeProperties.HEIGHT], self.imaginationColor, None)
                    self.createBox(o[ShapeProperties.COL], self.imaginaryWorldYOffset+o[ShapeProperties.ROW], o[ShapeProperties.WIDTH], o[ShapeProperties.HEIGHT], self.imaginationColor, None)
                if shapeType == ShapeTypes.CIRCLE:
                    self.createSphere(o[ShapeProperties.COL], o[ShapeProperties.ROW], o[ShapeProperties.RADIUS])
                    self.createSphere(o[ShapeProperties.COL], self.imaginaryWorldYOffset+o[ShapeProperties.ROW], o[ShapeProperties.RADIUS])
    
    def getUniqueNameForTerrainTrials(self):
        return self.runWhichTerrain + str(self.trialNumber) + '_' + str(self.numImaginaryRobots) + '.pickle'
    
    def loadOrCreateTerrain(self):        
        filename = self.getUniqueNameForTerrainTrials()
        self.fileOps.createDirectoryIfNotExisting(self.cons.terrainObjectsFolder)
        fileExists = self.fileOps.checkIfFileExists(self.cons.terrainObjectsFolder, filename) #check if a terrain is already generated for a trial number
        if fileExists: terrainObjects = self.fileOps.loadPickleFile(self.cons.terrainObjectsFolder, filename)
        else: terrainObjects = None
        return terrainObjects, filename, fileExists
        
    def createTerrainRandomBoxesLowDense(self):
        terrainObjects = None; fileExists = None
        if self.trialNumber:
            terrainObjects, filename, fileExists = self.loadOrCreateTerrain()
        if terrainObjects == None or self.trialNumber == None: #then create fresh randomized objects
            terrainObjects = {ShapeTypes.RECTANGLE: []}
            numObjects = 100; debrisStartCol = 200; debrisMaxHt = 50; boxMinSz = 5; boxMaxSz = 30
            for _ in range(numObjects):                
                col = random.randint(debrisStartCol, self.worldWidth-2*self.wallThickness)
                row = random.randint(self.debrisElevFromBottomWall+2*self.wallThickness, self.debrisElevFromBottomWall+debrisMaxHt)
                wid = random.randint(boxMinSz, boxMaxSz)
                ht = random.randint(boxMinSz, boxMaxSz)
                rect = {ShapeProperties.COL: col}; rect[ShapeProperties.ROW] = row; rect[ShapeProperties.WIDTH] = wid; rect[ShapeProperties.HEIGHT] = ht
                terrainObjects[ShapeTypes.RECTANGLE].append(rect)
        self.createTerrainObjects(terrainObjects)
        #---write
        if not fileExists and self.trialNumber:#write to file only if it's one of the trials
            self.fileOps.savePickleFile(self.cons.terrainObjectsFolder, filename, terrainObjects)
            
    def createTerrainBoxesInRowWithSpaces(self):
        if self.trialNumber:
            self.loadOrCreateTerrain()
        else:        
            w = 20; h = 45
            for col in range(200, self.finishLine, w*3):
                self.createBox(col, h, w, h, self.imaginationColor, None)
                self.createBox(col, self.imaginaryWorldYOffset+h, w, h, self.imaginationColor, None)  
    
    def createStaircaseTerrain(self):
        if self.trialNumber:
            self.loadOrCreateTerrain()
        else:         
            w = 20; h = 10; row = 35
            for col in range(300, self.finishLine, w):
                self.createBox(col, row, w, h, self.imaginationColor, None)
                self.createBox(col, row+self.imaginaryWorldYOffset, w, h, self.imaginationColor, None) 
                row = row + 8           
    
    def createSpheresTerrain(self):
        if self.trialNumber:
            self.loadOrCreateTerrain()
        else:        
            numSpheres = 50; minSize = 5; maxSize = 20
            minX = 200; maxX = self.finishLine
            yPosition = 40
            for _ in range(numSpheres):
                xPos = random.randint(minX, maxX)
                sphereSize = random.randint(minSize, maxSize)
                self.createSphere(xPos, yPosition, sphereSize)
                self.createSphere(xPos, self.imaginaryWorldYOffset+yPosition, sphereSize)                                                         
    
    def createBox(self, x, y, wd, ht, colour, fil):
        body = pymunk.Body(body_type = pymunk.Body.KINEMATIC)
        body.position = Vec2d(x, y)
        body.width = wd
        body.height = ht
        try:
            shape = pymunk.Poly.create_box(body, (wd, ht))
            if fil:
                shape.filter = self.robotBodyShapeFilter
            shape.color = colour
        except:
            pass
        shape.friction = self.highFriction
        self.space.add(shape)
        self.worldObjects.append(shape)  
        
    def createSphere(self, xPosition, yPosition, radius):
        sphereMass = 5000
        sphereInertia = pymunk.moment_for_circle(sphereMass, 0, radius, (0, 0))
        body = pymunk.Body(sphereMass, sphereInertia, body_type=pymunk.Body.KINEMATIC)
        #x = random.randint(115, 350)
        body.position = xPosition, yPosition
        shape = pymunk.Circle(body, radius, (0, 0))
        #shape.elasticity = 0.95
        shape.friction = 20
        shape.color = self.imaginationColor
        self.space.add(body, shape)
        self.worldObjects.append(shape)          
        
    def setImaginaryRobotPositionAndAnglesToRealRobot(self):
        pos = self.robots[self.cons.mainRobotID].getPositions()
        angles = self.robots[self.cons.mainRobotID].getUniqueBodyAngles()
        for robo in self.imaginaryRobots:
            p = pos[:]; a = angles[:] #copying values instead of references
            robo.setBodyPositionAndAngles(p, a, Vec2d(0, self.imaginaryWorldYOffset))
            robo.saveGenStartPos()
            robo.stopMotion()
        
    def generateInfoString(self):     
        self.infoString = self.genStateImagined.getInfoString() + ", terrain: " + self.runWhichTerrain.lower()
                
    def delete(self):
        super(ImaginationTwin, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
    
    def updatePosition(self):  
        updateBy = super(ImaginationTwin, self).updatePosition()    
        self.cumulativePosUpdateBy += updateBy
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy   
            for obj in self.imaginaryRobots:
                obj.updatePosition(updateBy)       
    
    def updateColor(self):
        for robo in self.robots:#for the main robot
            robo.setNormalRobotColor()
        fittestRobotID = self.genStateImagined.getFittestRobot()
        fittestRobotID = fittestRobotID if fittestRobotID else -1
        for i in range(0, len(self.imaginaryRobots)):
            if i == fittestRobotID: self.imaginaryRobots[i].setFocusRobotColor()
            else: self.imaginaryRobots[i].setImaginaryRobotColor()
            
    def createImaginaryRobots(self):      
        for _ in range(0, self.numImaginaryRobots, 1):
            self.imaginaryRobots.append(RobotBody(self.space, self.robotInitPos + Vec2d(0, self.imaginaryWorldYOffset), self.legsCode, self.fps))#deliberately placing it outside screen since it'll be brought back on screen in robot's position soon
            
    def deleteImaginaryRobots(self):
        for r in self.imaginaryRobots:
            r.delete()
        self.imaginaryRobots[:] = []    
             
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
            for _ in range(self.iterations): #iterations to get a more stable simulation
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
        for _ in range(0, self.worldWidth, 1):
            col = []
            for _ in range(0, self.worldHeight, 1):
                col.append(0)
            self.sensedObjects.append(col)
        self.imaginaryWorldYOffset = self.worldHeight 
        self.numRobots = 1
        #self.numImaginaryRobots = 4 #min 4 robots required for ComputationalIntelligence
        self.imaginaryRobots = []
        self.debrisElevFromBottomWall = 0
        self.groundThickness = 10
        self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
        self.imaginationColor = 100,100,100
        self.imaginationGroundColor = 100,150,100
        self.runState = RunCode.CONTINUE
        self.nextNode = None 
        self.cons = Constants()       
        
    def initialize(self):
        super(ActualImagination, self).initialize()       
        self.createGround(0, self.debrisElevFromBottomWall, self.groundColor)
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
            for _ in range(self.iterations): #iterations to get a more stable simulation
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
        shape = pymunk.Poly.create_box(body, (wd, ht)); shape.color = colour; shape.friction = self.highFriction; 
        self.space.add(shape); self.worldObjects.append(shape)   
        
                                   
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
class TestRunMode:
    CREATING_RESULTS = 1
    VIEWING_RESULTS = 2
    
class TestWorld(Worlds):#inherits
    def __init__(self, legCode):    #def __init__(self, legCode, actionNet):        
        super(TestWorld, self).__init__()
        self.legsCode = legCode      
        self.screenWidth = 900
        self.screenHeight = 620 #keep at at least 350        
        self.worldWidth = 3000 #overriding
        self.worldHeight = 300
        self.sensedObjects = []
        #---create array representation of world for imagination
        for _ in range(0, self.worldWidth, 1):
            col = []
            for _ in range(0, self.worldHeight, 1):
                col.append(0)
            self.sensedObjects.append(col)
        self.imaginaryWorldYOffset = self.worldHeight 
        self.numRobots = 1
        self.imaginaryRobots = []
        self.debrisElevFromBottomWall = 0
        self.groundThickness = 10
        self.robotInitPos = Vec2d(self.screenWidth/2, 50) 
        self.imaginationColor = 100,100,100
        self.imaginationGroundColor = 100,150,100
        self.cons = Constants()   
        self.numberOfTimesLegsTouchSurface = None
        self.chassisAngle = None #[[angle1, angle2...angleNumFrames], [],...]
        #------------------------------------------
        #--- run mode
        #------------------------------------------  
        #self.runMode = TestRunMode.CREATING_RESULTS
        self.runMode = TestRunMode.VIEWING_RESULTS
        
    def initialize(self):
        super(TestWorld, self).initialize()       
        self.createGround(0, self.debrisElevFromBottomWall, self.groundColor)
        self.createWorldBoundary(0, self.imaginaryWorldYOffset, self.imaginationColor)
        #self.createFewObjects()       
        #self.createGround(0, self.imaginaryWorldYOffset, self.imaginationGroundColor)        
        self.cumulativePosUpdateBy = Vec2d(0,0)            
    
    def runWorld(self): #may get overridden in child class
        #---first test scenario
        self.testAccuracyOfRepeatedSimilarMotorRates()
    
    def testAccuracyOfRepeatedSimilarMotorRates(self):
        analytics = TestAnalyticsForMovementAccuracy()
        folderToStoreResults = "testAccuracyOfRepeatedSimilarMotorRates/"
        originalPosition = self.robots[0].getPositions()
        originalAngles = self.robots[0].getUniqueBodyAngles()
        numTrials = 20
        numSimulations = 100
        durationToRun = 50
        if self.runMode == TestRunMode.CREATING_RESULTS:#run the trials
            for trial in range(numTrials):            
                self.chassisAngle = []       
                rates = self.robots[0].setRandomLegMotorRates()                
                print('Trial ', trial, "----------. Motor rates:", rates)
                ratesAndPositions = []
                ratesAndPositions.append(rates)
                filename1 = analytics.generateRatesPositionFilename(trial)
                filename2 = analytics.generateSurfaceTouchFilename(trial)
                filename3 = analytics.generateChassisAngleFilename(trial)
                for sim in range(numSimulations):                     
                    self.robots[0].setLegMotorRates(rates)   
                    self.runSimulation(durationToRun)
                    position = self.robots[0].getPosition()
                    ratesAndPositions.append([position[0], position[1]])
                    self.resetRobotToOriginalPosition(originalPosition, originalAngles)
                    self.infoString = "Trial:" + str(trial) + "/" + str(numTrials) + ", RateRep: " + str(sim) + "/" + str(numSimulations) + ", MotorRates: " + str([round(x, 2) for x in rates])
                analytics.saveDataToDisk(folderToStoreResults, filename1, ratesAndPositions)
                analytics.saveDataToDisk(folderToStoreResults, filename2, self.numberOfTimesLegsTouchSurface)
                analytics.saveDataToDisk(folderToStoreResults, filename3, self.chassisAngle)
        
        if self.runMode == TestRunMode.VIEWING_RESULTS:#view analytics of saved results 
            xPositions = []; yPositions = []; rates = []; surfaceTouches = []; chassisAngles = []
            for trial in range(numTrials):   
                filename1 = analytics.generateRatesPositionFilename(trial)
                filename2 = analytics.generateSurfaceTouchFilename(trial)
                filename3 = analytics.generateChassisAngleFilename(trial)
                dx = []; dy = []
                rate, positions = analytics.loadRatesPositionDataFromDisk(folderToStoreResults, filename1)
                surfaceTouch = analytics.loadSurfaceTouchDataFromDisk(folderToStoreResults, filename2)
                chAng = analytics.loadChassisAngleDataFromDisk(folderToStoreResults, filename3)
                for pos in positions:
                    dx.append(pos[0]); dy.append(pos[1])
                rates.append(rate); surfaceTouches.append(surfaceTouch); chassisAngles.append(chAng)
                print('Rate: ', rate, ' surfaceTouch:', surfaceTouch)                
                print('dx: mean=', statistics.mean(dx), ", variance=", statistics.variance(dx), "std deviation=", statistics.stdev(dx))
                xPositions.append(dx); yPositions.append(dy)
            analytics.plot(folderToStoreResults, xPositions, yPositions, rates, surfaceTouches, chassisAngles)
    
    def runSimulation(self, durationToRun):
        clock = pygame.time.Clock()
        simulating = True
        self.robots[0].startMotion()    
        self.numberOfTimesLegsTouchSurface = [0] * len(self.robots[0].legs) 
        chAngle = []
        counter = 0
        while simulating:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    simulating = False #sys.exit(0)                   
            if not simulating: 
                break               
            #---Update physics
            dt = 1.0 / float(self.fps) / float(self.iterations)
            for _ in range(self.iterations): #iterations to get a more stable simulation
                self.space.step(dt)
            #---Update world based on player focus
            self.updatePosition()
            #---get tactile info
            tactileInputPerLeg = self.robots[0].getNumContactPointsForEachLeg()
            for i in range(len(tactileInputPerLeg)):
                self.numberOfTimesLegsTouchSurface[i] = self.numberOfTimesLegsTouchSurface[i] + tactileInputPerLeg[i]  
            #---get angle
            chAngle.append(self.robots[0].getBodyAngle())
            #---draw all objects
            self.draw()            
            clock.tick(self.fps)
            counter = counter + 1
            if counter >= durationToRun: break
        #---actions to do after simulation  
        self.chassisAngle.append(chAngle)
    
    def initializeRobots(self):#overriding  
        widthSep = 100; heightSep = 100; counter = 0
        motorMovtDuration = 50
        for i in range(50, self.worldHeight, heightSep):
            if counter >= self.numRobots: break
            for j in range(200, self.worldWidth, widthSep):
                self.robots.append(RobotBody(self.space, Vec2d(j, i), self.legsCode, motorMovtDuration)) #self.robots.append(LearningRobot(self, Vec2d(j, i), self.legsCode, self.actions))
                counter += 1 
                if counter >= self.numRobots: break
        for robo in self.robots:
            robo.makeRobotDynamic()
            
    def resetRobotToOriginalPosition(self, pos, angles):
        for robo in self.robots:
            p = pos[:]; a = angles[:] #copying values instead of references
            robo.setBodyPositionAndAngles(p, a, Vec2d(0, 0))
            robo.saveGenStartPos()
            robo.stopMotion()          

    def removeBoundary(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
                         
    def delete(self):
        super(TestWorld, self).delete()   
        for ob in self.worldObjects:
            self.space.remove(ob)
        self.worldObjects[:] = []  
     
    def updatePosition(self):  
        updateBy = super(TestWorld, self).updatePosition()    
        if updateBy != (0, 0):
            for ob in self.worldObjects:
                ob.body.position += updateBy     
                                    
    def createGround(self, groundX, groundY, grColor):
        self.createBox(groundX+self.worldWidth/2, groundY+self.wallThickness+self.wallThickness/2, self.worldWidth-2*self.wallThickness, self.wallThickness, grColor)

#     def createFewObjects(self):
#         w = 10; h = 40
#         self.createBox(50, 50, w, w, self.groundColor)
#         self.createBox(120, 60, w, h, self.groundColor)
        
    def createBox(self, x, y, wd, ht, colour):
        body = pymunk.Body(body_type = pymunk.Body.KINEMATIC); body.position = Vec2d(x, y); body.width = wd; body.height = ht
        shape = pymunk.Poly.create_box(body, (wd, ht)); shape.color = colour; shape.friction = self.highFriction; 
        self.space.add(shape); self.worldObjects.append(shape)   
        