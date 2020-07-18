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
from WalkingRobot import RobotBody
from WalkingRobot import Constants
from LearningRobot import LearningRobot
from ComputationalIntelligence import SimpleDE, RunCode, RandomBest, SimplePSO
from pymunk.shape_filter import ShapeFilter

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
    REAL_MOTOR_EXEC = 2
    REAL_GENERATION = 3   
    
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
            #self.CI = RandomBest(self.robots)
            #self.CI = SimpleDE(self.robots)
            self.CI = SimplePSO(self.robots)
            
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
        return "Gen: " + ("-" if self.currGen<=0 else str(self.currGen)) + '/' + ("-" if self.currGen<=0 else str(self.maxGens)) + self.CI.getInfoString()
    def getFittestRobot(self):
        return self.CI.getFittestRobot()


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
        self.worldWidth = 900 #overriding
        self.worldHeight = 300
        self.finishLine = self.worldWidth - 100 
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
        
    def initialize(self):
        super(ImaginationTwin, self).initialize()
        self.createDebris(self.debrisElevFromBottomWall, self.imaginationColor)
        self.copyDebrisToImaginary(self.imaginaryWorldYOffset, self.imaginationColor)       
        self.createGround(0, self.debrisElevFromBottomWall, self.groundColor)
        self.createWorldBoundary(0, self.imaginaryWorldYOffset, self.imaginationColor) 
        self.__addFinishLine__()      
        self.createGround(0, self.imaginaryWorldYOffset, self.imaginationGroundColor)        
        self.cumulativePosUpdateBy = Vec2d(0,0)      
        self.createImaginaryRobots()
        #self.behaviour = SimpleDE(self.imaginaryRobots, self.robots)
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

    def createDebris(self, groundY, debColor):
        debrisStartCol = 200; debrisMaxHt = 50; boxMinSz = 5; boxMaxSz = 30
        for _ in range(0, 100, 1):
            self.createBox(random.randint(debrisStartCol, self.worldWidth-2*self.wallThickness), random.randint(groundY+2*self.wallThickness, groundY+debrisMaxHt), random.randint(boxMinSz, boxMaxSz), random.randint(boxMinSz, boxMaxSz), debColor, None)        
        
    def copyDebrisToImaginary(self, groundY, debColor):
        for i in range(0, len(self.worldObjects), 1):
            self.createBox(self.worldObjects[i].body.position[0], groundY+self.worldObjects[i].body.position[1], self.worldObjects[i].body.width, self.worldObjects[i].body.height, debColor, None)        
    
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
        
    def setImaginaryRobotPositionAndAnglesToRealRobot(self):
        pos = self.robots[self.cons.mainRobotID].getPositions()
        angles = self.robots[self.cons.mainRobotID].getUniqueBodyAngles()
        for robo in self.imaginaryRobots:
            p = pos[:]; a = angles[:] #copying values instead of references
            robo.setBodyPositionAndAngles(p, a, Vec2d(0, self.imaginaryWorldYOffset))
            robo.saveGenStartPos()
            robo.stopMotion()
        
    def generateInfoString(self):     
        self.infoString = self.genStateImagined.getInfoString()         
                
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
class BoxesInRowWithSpacesTerrain:
    def __init__(self, parent, world):
        self.parent = parent
        self.world = world
        
    def createTerrain(self, groundX, elevFromBottom, worldWidth, wallThickness, screenWidth):
        self.world.robotInitPos = Vec2d(200, 50) 
        self.parent.createBox(groundX+worldWidth/2, elevFromBottom+wallThickness+wallThickness/2, worldWidth-2*wallThickness, wallThickness, self.parent.terrainColor, None)
        self.world.initializeRobots()
        w = 20; h = 10
        for col in range(300, self.parent.finishLine, w*3):
            self.parent.createBox(col, 35, w, h, self.parent.obstacleColor, None) 
            
    def getMetrics(self, metricsInst):
        #metricsInst.results[RLT.TERRAIN_NAME] = self.__class__.__name__
        return metricsInst           

class StepsTerrain:
    def __init__(self, parent, world):
        self.parent = parent
        self.world = world
        
    def createTerrain(self, groundX, elevFromBottom, worldWidth, wallThickness, screenWidth):
        self.world.robotInitPos = Vec2d(200, 50) 
        self.parent.createBox(groundX+worldWidth/2, elevFromBottom+wallThickness+wallThickness/2, worldWidth-2*wallThickness, wallThickness, self.parent.terrainColor, None)
        self.world.initializeRobots()
        w = 20; h = 10; row = 35
        for col in range(300, self.parent.finishLine, w):
            self.parent.createBox(col, row, w, h, self.parent.obstacleColor, None) 
            row = row + 5
            
    def getMetrics(self, metricsInst):
        #metricsInst.results[RLT.TERRAIN_NAME] = self.__class__.__name__
        return metricsInst  
    
class SpheresTerrain:
    def __init__(self, parent, world):
        self.parent = parent
        self.world = world
        
    def createTerrain(self, groundX, elevFromBottom, worldWidth, wallThickness, screenWidth):
        self.world.robotInitPos = Vec2d(200, 50) 
        self.parent.createBox(groundX+worldWidth/2, elevFromBottom+wallThickness+wallThickness/2, worldWidth-2*wallThickness, wallThickness, self.parent.terrainColor, None)
        self.world.initializeRobots()
        numSpheres = 50; minSize = 5; maxSize = 6
        minX = 250; maxX = 950
        yPosition = 40
        for _ in range(numSpheres):
            xPos = random.randint(minX, maxX)
            sphereSize = minSize #random.randint(minSize, maxSize)
            self.parent.createSphere(xPos, yPosition, sphereSize) 
            
    def getMetrics(self, metricsInst):
        #metricsInst.results[RLT.TERRAIN_NAME] = self.__class__.__name__
        return metricsInst      

class MinMaxTerrain:
    def __init__(self, parent, world):
        self.parent = parent
        self.world = world
        
    def createTerrain(self, groundX, elevFromBottom, worldWidth, wallThickness, screenWidth):
        self.world.robotInitPos = Vec2d(worldWidth/2, 250) 
        self.parent.createBox(groundX+worldWidth/2, elevFromBottom+wallThickness+wallThickness/2, worldWidth-2*wallThickness, wallThickness, self.parent.terrainColor, None)
        self.world.initializeRobots()
        self.createDebris(elevFromBottom)
        
    def createDebris(self, groundY):
        debrisStartCol = self.world.wallThickness * 2
        debrisMaxHt = 200
        boxMinSz = 5
        boxMaxSz = 30
        for _ in range(0, 100, 1):
            self.parent.createBox(random.randint(debrisStartCol, self.world.worldWidth-2*self.world.wallThickness), random.randint(groundY+2*self.world.wallThickness, groundY+debrisMaxHt), random.randint(boxMinSz, boxMaxSz), random.randint(boxMinSz, boxMaxSz), self.parent.debrisColor, None)        
         
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
        
                                   
