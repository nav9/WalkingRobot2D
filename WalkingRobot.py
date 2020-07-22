# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

# Note: WalkingRobot is for ImaginationTwin and LearningRobot is for ActualImagination

import math
import pymunk
import random
import numpy as np
from pymunk import Vec2d    
from Enums import Directions
from ComputationalIntelligence import Constants  

class LegPart:#This is one leg part. Could be part A that's connected to the chassis or part B that's connected to part A
    def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight, limbOrder):
        self.space = pymunk.Space()
        d = Directions()
        self.ori = d.getDirn()
        self.leftRight = self.ori['LEFT']  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
        self.prevBodyXY = 0
        self.chassisWd = 0  # chassis width
        self.legWd = 20 #leg thickness (width)
        self.legHt = 2 #leg height
        self.legMass = 0.5
        self.relativeAnguVel = 0
        self.obj_body = None
        self.leg_shape = None
        self.pinJoint = None
        self.motor = None   
        self.id = limbOrder #A, B or C
        self.attr = None #L1_A etc. Gets assigned in the function that creates legs
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.prevBodyXY = prevBody.position
        self.chassisWd = prevBodyWidth
        self.leftRight = leftOrRight
        self.__createLegPart__()
        self.__linkLegPartWithPrevBodyPart__(prevBody)
        #self.experience = []
    
    def __createLegPart__(self):
        self.obj_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd, self.legHt)))
        if self.leftRight == self.ori['LEFT']: self.obj_body.position = self.prevBodyXY - ((self.chassisWd / 2) + (self.legWd / 2), 0)            
        if self.leftRight == self.ori['RIGHT']: self.obj_body.position = self.prevBodyXY + ((self.chassisWd / 2) + (self.legWd / 2), 0)
        self.leg_shape = pymunk.Poly.create_box(self.obj_body, (self.legWd, self.legHt))            
        self.leg_shape.filter = self.shapeFilter
        self.leg_shape.color = 200, 200, 200  
        self.leg_shape.friction = 20.0 
        self.space.add(self.leg_shape, self.obj_body) 
        
    def delete(self): self.space.remove(self.leg_shape, self.obj_body, self.pinJoint, self.motor)
    
    def getTip(self):  
        v = self.leg_shape.get_vertices()    
        if self.leftRight == self.ori['LEFT']: v = v[2].rotated(self.leg_shape.body.angle) + self.leg_shape.body.position #TODO/BUG: This will have to be changed based on polygon shape chosen in future
        if self.leftRight == self.ori['RIGHT']: v = v[1].rotated(self.leg_shape.body.angle) + self.leg_shape.body.position #TODO/BUG: This will have to be changed based on polygon shape chosen in future        
        return Vec2d(v)
    
    def __linkLegPartWithPrevBodyPart__(self, prevBody):
        maxMotorRate = 6
        motorRateRangePieces = (maxMotorRate * 2 + 1) * 10
        #---link left leg A with Chassis
        if self.leftRight == self.ori['LEFT']:
            self.pinJoint = pymunk.PinJoint(self.obj_body, prevBody, (self.legWd / 2, 0), (-self.chassisWd / 2, 0))
        if self.leftRight == self.ori['RIGHT']:
            self.pinJoint = pymunk.PinJoint(self.obj_body, prevBody, (-self.legWd / 2, 0), (self.chassisWd / 2, 0))            
        self.motor = pymunk.SimpleMotor(self.obj_body, prevBody, self.relativeAnguVel) 
        self.space.add(self.pinJoint, self.motor)
        self.motor.rate = 0
        self.motor.max_force = 10000000
        self.motor.legRateRange = np.linspace(-maxMotorRate, maxMotorRate, motorRateRangePieces)
        #print('legRateRange: ',str(self.motor.legRateRange), 'len:', len(self.motor.legRateRange))         
        
    def updatePosition(self, offsetXY): self.obj_body.position = self.obj_body.position + offsetXY         
    def getLegAngle(self): return round(math.degrees(self.obj_body.angle) % 360)        

class RobotBody:
    def __init__(self, pymunkSpace, chassisCenterPoint, legCode, maxMotorMovementDuration):#the maxMotorMovementDuration can be changed to any value. It's kept at the fps falue just to keep movements short
        self.legsCode = legCode
        self.ownBodyShapeFilter = pymunk.ShapeFilter(group=1) #to prevent collisions between robot body parts
        d = Directions()  #leg at left or right of chassis
        self.ori = d.getDirn()
        self.chassisWd = 30 #chassis width 
        self.chassisHt = 20 #chassis height
        self.chassisMass = 5
        self.obj_body = None #chassis body
        self.chassis_shape = None #chassis shape 
        self.legs = []
        self.limbMotorRates = []
        self.motorMovementDuration = 50 #set according to the frames per second, but will be changed by CI classes
        self.motorMovementDurationRange = np.linspace(0, maxMotorMovementDuration, int(maxMotorMovementDuration/5))
        #print('motorMovementDurationRange: ',str(self.motorMovementDurationRange), 'len:', len(self.motorMovementDurationRange))
        self.direction = self.ori['RIGHT'] #direction the robot needs to go in
        #self.currentActionNode = []#node on the action network        
        #self.brain = None       
        self.space = pymunkSpace
        self.quadrantAccuracy = 3 #pixels
        self.__createBody__(chassisCenterPoint)
        self.decimalPrecision = 2
        self.const = Constants()              
        
    def __createBody__(self, chassisXY):
        self.obj_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.chassisWd, self.chassisHt)))
        self.obj_body.body_type = pymunk.Body.KINEMATIC
        self.obj_body.position = chassisXY
        self.obj_body.startPosition = Vec2d(self.obj_body.position[0], self.obj_body.position[1])
        #self.brain = Brain(self.obj_body.startPosition)    
        self.chassis_shape = pymunk.Poly.create_box(self.obj_body, (self.chassisWd, self.chassisHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.setNormalRobotColor() 
        self.chassis_shape.friction = 20.0
        self.space.add(self.chassis_shape, self.obj_body) 
        self.createLegs()
        self.makeRobotDynamic()
        self.robotGenStartPos = Vec2d(self.getPosition()[0], self.getPosition()[1])
        self.tactileSensor = Tactile_Sensor()  
        for leg in self.legs:
            self.tactileSensor.add(leg)#adding the chassis also will affect the analytics code which needs points of contact, so check that part     
    
    def createLegs(self):
        s = self.legsCode.split("#")
        lt = s[0]; rt = s[1]; rt = rt[::-1]#reverse string rt
        for s in lt.split(","):#number of left legs
            if len(s) == 1:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT'], 'A'); self.legs.append(leftLegA)
            if len(s) == 2:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT'], 'A'); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['LEFT'], 'B'); self.legs.append(leftLegB)                
            if len(s) == 3:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT'], 'A'); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['LEFT'], 'B'); self.legs.append(leftLegB)                 
                leftLegC = LegPart(self.space, self.ownBodyShapeFilter, leftLegB.obj_body, leftLegB.legWd, self.ori['LEFT'], 'C'); self.legs.append(leftLegC)                                 
        for s in rt.split(","):#number of right legs
            if len(s) == 1:
                rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT'], 'A'); self.legs.append(rightLegA)
            if len(s) == 2:
                rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT'], 'A'); self.legs.append(rightLegA) 
                rightLegB = LegPart(self.space, self.ownBodyShapeFilter, rightLegA.obj_body, rightLegA.legWd, self.ori['RIGHT'], 'B'); self.legs.append(rightLegB)                        
            if len(s) == 3:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT'], 'A'); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['RIGHT'], 'B'); self.legs.append(leftLegB)                 
                leftLegC = LegPart(self.space, self.ownBodyShapeFilter, leftLegB.obj_body, leftLegB.legWd, self.ori['RIGHT'], 'C'); self.legs.append(leftLegC)
        i = 0
        for limb in self.legs:
            limb.attr = 'L' + str(i) + '_' + str(limb.id) #L1_A, L0_C ... etc.
            i = i + 1                                                 
        #---limbMotorRates will store rates for DE to use, but the starting and stopping of motors can be done independent of the values in limbMotorRates
        self.setRandomLegMotorRates()
        self.stopMotion()
    
    def __getFitnessBasedOnDirection__(self, prevPos, currPos):
        if self.robotDirection(prevPos, currPos) == self.direction: 
            return round(abs(math.sqrt((currPos[0]-prevPos[0])**2 + (currPos[1]-prevPos[1])**2)), self.decimalPrecision)
        else: 
            return self.const.NOTFIT 
        
    def robotDirection(self, prevPos, currPos): 
        ang = round(math.degrees(math.atan2((currPos[1]-prevPos[1]), (currPos[0]-prevPos[0])))) % 360
        if ang > 270 or ang <= 91: direc = self.ori['RIGHT']
        if ang > 91 and ang <= 135: direc = self.ori['UP']
        if ang > 135 and ang <= 225: direc = self.ori['LEFT']
        if ang > 225 and ang <= 270: direc = self.ori['DOWN']
        return direc        
    
    def stopMotion(self):
        for leg in self.legs: 
            leg.motor.rate = 0
            
    def startMotion(self):
        for i in range(0, len(self.legs)):
            self.legs[i].motor.rate = self.limbMotorRates[i]
            
    def setRandomLegMotorRates(self):
        self.limbMotorRates = []
        for leg in self.legs:
            self.limbMotorRates.append(random.choice(leg.motor.legRateRange))
        return self.limbMotorRates
            
    def getMinMaxLegRates(self):
        return min(self.legs[0].motor.legRateRange), max(self.legs[0].motor.legRateRange) 

    def setLegMotorRates(self, motorRates):#passing an empty list to this will set rates to zero
        if len(motorRates) == 0: 
            motorRates = [0] * len(self.legs) #[0,0,0,0]
        self.limbMotorRates = []
        for i in range(0, len(self.legs)):
            self.limbMotorRates.append(motorRates[i])
            
    def getLegMotorRates(self):
        return self.limbMotorRates                                           
    
    def setFocusRobotColor(self): self.chassis_shape.color = (45, 160, 185)
    def setNormalRobotColor(self): self.chassis_shape.color = (231, 30, 30)
    
    def setImaginaryRobotColor(self):
        self.chassis_shape.color = (110, 110, 110)  
        for leg in self.legs: 
            leg.leg_shape.color = (110, 110, 110)
        
    def makeRobotStatic(self):
        self.obj_body.body_type = pymunk.Body.KINEMATIC
        
    def makeRobotDynamic(self):
        self.obj_body.body_type = pymunk.Body.DYNAMIC
        self.obj_body.mass = self.chassisMass
        self.obj_body.moment = pymunk.moment_for_box(self.chassisMass, (self.chassisWd, self.chassisHt))            
        
    def delete(self):
        self.space.remove(self.chassis_shape); self.space.remove(self.obj_body)
        for legPart in self.legs: legPart.delete()
        self.legs[:] = [] #clear the list
        
    def getPosition(self):
        return self.obj_body.position
    
    def saveGenStartPos(self):        
        self.robotGenStartPos = Vec2d(self.getPosition()[0], self.getPosition()[1])#current position
    
    def getFitness(self):#returns 0 if movement is not in desired direction or an absolute positive value of the magnitude of displacement in desired direction
        return self.__getFitnessBasedOnDirection__(self.robotGenStartPos, self.getPosition())
        
    def updatePosition(self, offsetXY):
        self.obj_body.position += offsetXY 
        self.obj_body.startPosition += offsetXY
        for leg in self.legs: 
            leg.updatePosition(offsetXY)
    
    def getBodyAngle(self): return round(math.degrees(self.obj_body.angle) % 360)
    
    def getUniqueBodyAngles(self):
        ang = [self.roundToNearest(self.getBodyAngle())]
        for leg in self.legs: ang.append(self.roundToNearest(leg.getLegAngle()))
        return ang
    
    def setBodyPositionAndAngles(self, pos, angles, offset):
        p = pos.pop(0)
        self.obj_body.position = Vec2d(p[0], p[1]) + offset
        self.obj_body.startPosition = Vec2d(p[0], p[1]) + offset
        self.obj_body.angle = math.radians(angles.pop(0))
        for leg in self.legs:
            p = pos.pop(0)
            leg.obj_body.position = Vec2d(p[0], p[1]) + offset
            leg.obj_body.angle = math.radians(angles.pop(0))
        return pos    
    
    def getPositions(self):
        pos = [Vec2d(self.obj_body.position)]
        for leg in self.legs: pos.append(Vec2d(leg.obj_body.position))
        return pos
        
    def roundToNearest(self, num):
        roundOffPrecision = 5
        rem = num % roundOffPrecision
        if rem < roundOffPrecision / 2: num = int(num/roundOffPrecision) * roundOffPrecision
        else: num = int((num+roundOffPrecision) / roundOffPrecision) * roundOffPrecision
        return num
    
    def getLegQuadrants(self):
        quads = []
        for i in range(0, len(self.body.legs), 1):
            quads.append(self.__getQuadrant__(self.body.legs[i].getTip()))
        return quads
            
    def __getQuadrant__(self, pt):#pt should be Vec2d or tuple
        translateOffset = -Vec2d(self.obj_body.position)
        tPt = pt + translateOffset #translate
        theta = -self.getBodyAngle() #for clockwise rotation
        tPt[0] = tPt[0] * math.cos(theta) - tPt[1] * math.sin(theta) #rotate
        tPt[1] = tPt[0] * math.sin(theta) + tPt[1] * math.cos(theta) #rotate
        return (math.floor(tPt[0]/self.quadrantAccuracy), math.floor(tPt[1]/self.quadrantAccuracy))
        
    def getNumContactPointsForEachLeg(self):#can use this only if tactile sensor is added
        return self.tactileSensor.get() #[numContactPointsForLeg1, numContactPointsForLeg2, numContactPointsForLeg3, numContactPointsForLeg4]

class Tactile_Sensor:
    def __init__(self):
        self.bodyParts = [] 
        self.partPointMap = {}
        self.points = None
        self.bodyPartBeingArbited = None
    
    def add(self, bodyPart):
        self.bodyParts.append(bodyPart)
    
    def get(self):
        self.partPointMap = {}
        for bodyPart in self.bodyParts: self.partPointMap[bodyPart.attr] = 0       
        self.points = []
        for bodyPart in self.bodyParts:            
            self.bodyPartBeingArbited = bodyPart
            bodyPart.obj_body.each_arbiter(self.__contactInfo__) #invoke callback fn
            self.points.append(self.partPointMap[self.bodyPartBeingArbited.attr])
        return self.points
    
    def __contactInfo__(self, arbiter): #callback fn    
        self.partPointMap[self.bodyPartBeingArbited.attr] = self.partPointMap[self.bodyPartBeingArbited.attr] + len(arbiter.contact_point_set.points)
        
        