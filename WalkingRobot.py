import pymunk
from pymunk import Vec2d, shapes
import random
from collections import defaultdict
import hashlib
from abc import ABCMeta, abstractmethod

class AbstractActions:
    __metaclass__ = ABCMeta
    @abstractmethod
    def setValues(self):#The force applied by the action
        pass    
    @abstractmethod
    def getValues(self):#The force applied by the action
        pass
    @abstractmethod
    def setRange(self):#The range of values the action can perform
        pass    
    @abstractmethod
    def getRange(self):#The range of values the action can perform
        pass
    
# class LegActions(AbstractActions):    
#     legRate = 0
#     max_force = 10000000
#     legRateRange = []
#     
#     def __init__(self, legMotor):
#         maxMotorRate = 5; motorRateRangeStep = 2
#         self.legRateRange = range(-maxMotorRate, maxMotorRate, motorRateRangeStep)#range(start,stop,step) 
#         legMotor.max_force = 10000000
#         legMotor.rate = 0

#     def getHash(self):
#         # Assumes the default UTF-8
#         mystring = ""
#         hash_object = hashlib.md5(mystring.encode()) #hash_object = hashlib.sha1(b'Hello World')
#         print(hash_object.hexdigest())
    
class ActionsCortex:
    actionSeq = defaultdict(list)
    
class Directions:
    UP = 1
    DOWN = 2
    LEFT = 3    
    RIGHT = 4

class RobotLeg:#This is one leg. Legs may consist of multiple parts. PartA is connected to the chassis. part B is connected to partA
    space = pymunk.Space()
    shapeFilter = None
    ori = Directions()
    legLeftOrRight = ori.LEFT  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
    prevBodyXY = 0
    chassisWd = 0  # chassis width
    #chHt = 0  # chassis height
    legWd = 20 #leg thickness (width)
    legHt = 2 #leg height
    #legWd = 20
    #legHt_b = 2
    legMass = 1
    relativeAnguVel = 0
    leg_body = None
    leg_shape = None
    #legB_body = None
    #legB_shape = None
    pinJoint = None
    motor = None
    #LegAChassis_pinJoint = None
    #LegAChassis_motor = None
    #---actions
    #legAction = []#stores action object for each leg part
    
    def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight):
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.prevBodyXY = prevBody.position
        self.chassisWd = prevBodyWidth
#        self.chHt = chassisHeight
        self.legLeftOrRight = leftOrRight
        self.__createLegPart__()
        self.__linkLegPartWithPrevBodyPart__(prevBody)
    
    def __createLegPart__(self):
        self.leg_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd, self.legHt)))
        if self.legLeftOrRight == self.ori.LEFT:
            self.leg_body.position = self.prevBodyXY - ((self.chassisWd / 2) + (self.legWd / 2), 0)
        if self.legLeftOrRight == self.ori.RIGHT:
            self.leg_body.position = self.prevBodyXY + ((self.chassisWd / 2) + (self.legWd / 2), 0)
        self.leg_body.startPosition = Vec2d(self.leg_body.position)
        self.leg_body.startAngle = self.leg_body.angle
        self.leg_shape = pymunk.Poly.create_box(self.leg_body, (self.legWd, self.legHt))        
        self.leg_shape.filter = self.shapeFilter
        self.leg_shape.color = 255, 0, 0  
        self.leg_shape.friction = 10.0 
        self.space.add(self.leg_body, self.leg_shape)   

        
#         self.legA_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd_a, self.legHt_a)))
#         if self.legLeftOrRight == self.ori.LEFT:
#             self.legA_body.position = self.focusRobotXY - ((self.chassisWd / 2) + (self.legWd_a / 2), 0)
#         if self.legLeftOrRight == self.ori.RIGHT:
#             self.legA_body.position = self.focusRobotXY + ((self.chassisWd / 2) + (self.legWd_a / 2), 0)
#         self.legA_body.startPosition = Vec2d(self.legA_body.position)
#         self.legA_body.startAngle = self.legA_body.angle
#         self.legA_shape = pymunk.Poly.create_box(self.legA_body, (self.legWd_a, self.legHt_a))        
#         self.legA_shape.filter = self.shapeFilter
#         self.legA_shape.color = 255, 0, 0  
#         self.legA_shape.friction = 10.0 
#         self.space.add(self.legA_body, self.legA_shape)        
        
#     def __createLegB__(self):
#         self.legB_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd_b, self.legHt_b)))
#         if self.legLeftOrRight == self.ori.LEFT:
#             self.legB_body.position = self.legA_body.position - ((self.legWd_a / 2) + (self.legWd_b / 2), 0)
#         if self.legLeftOrRight == self.ori.RIGHT:
#             self.legB_body.position = self.legA_body.position + ((self.legWd_a / 2) + (self.legWd_b / 2), 0)
#         self.legB_body.startPosition = Vec2d(self.legB_body.position)
#         self.legB_body.startAngle = self.legB_body.angle
#         self.legB_shape = pymunk.Poly.create_box(self.legB_body, (self.legWd_b, self.legHt_b))
#         self.legB_shape.filter = self.shapeFilter        
#         self.legB_shape.color = 0, 255, 0 
#         self.legB_shape.friction = 10.0 
#         self.space.add(self.legB_body, self.legB_shape)
    
    def __linkLegPartWithPrevBodyPart__(self, prevBody):
        maxMotorRate = 5
        motorRateRangeStep = 2
#         #---link left leg B with left leg A       
#         if self.legLeftOrRight == self.ori.LEFT:
#             self.pinJoint = pymunk.PinJoint(self.leg_body, self.leg_body, (self.legWd / 2, 0), (-self.legWd / 2, 0))  # anchor point coordinates are wrt the body; not the space
#         if self.legLeftOrRight == self.ori.RIGHT:
#             self.pinJoint = pymunk.PinJoint(self.leg_body, self.leg_body, (-self.legWd / 2, 0), (self.legWd / 2, 0))  # anchor point coordinates are wrt the body; not the space                        
#         self.motor = pymunk.SimpleMotor(self.leg_body, self.leg_body, self.relativeAnguVel)
#         self.space.add(self.pinJoint, self.motor)
#         self.motor.rate = 0
#         self.motor.max_force = 10000000
#         self.motor.legRateRange = range(-maxMotorRate, maxMotorRate, motorRateRangeStep)#range(start,stop,step) 
        
        #---link left leg A with Chassis
        if self.legLeftOrRight == self.ori.LEFT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (self.legWd / 2, 0), (-self.chassisWd / 2, 0))
        if self.legLeftOrRight == self.ori.RIGHT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (-self.legWd / 2, 0), (self.chassisWd / 2, 0))            
        self.motor = pymunk.SimpleMotor(self.leg_body, prevBody, self.relativeAnguVel) 
        self.space.add(self.pinJoint, self.motor)
        self.motor.rate = -5
        self.motor.max_force = 10000000
        self.motor.legRateRange = range(-maxMotorRate, maxMotorRate, motorRateRangeStep)#range(start,stop,step)         
        
    def updatePosition(self, offsetXY):
        self.leg_body.position = self.leg_body.position + offsetXY 
        #self.legB_body.position = self.legB_body.position + offsetXY 

#     def getLegStatesAndMotorRates(self):
#         bodyStates = []; motorRates = []
#         bodyStates.append(self.legA_body.angle)
#         #bodyStates.append(self.legA_body.position)
#         bodyStates.append(self.legB_body.angle)
#         #bodyStates.append(self.legB_body.position)
#         motorRates.append(self.LegAChassis_motor.rate)
#         motorRates.append(self.LegBLegA_motor.rate)
#         return (bodyStates, motorRates)
    
class RobotBody:
    space = None
    ownBodyShapeFilter = pymunk.ShapeFilter(group=1)#to prevent collisions between robot body parts
    ori = Directions()  #leg at left or right of chassis
    chassisWd = 30 #chassis width 
    chHt = 20 #chassis height
    chassisMass = 10
    chassis_body = None  #chassis body
    chassis_shape = None  #chassis shape    
    legs = []
    numLegs = 1
    
    def __init__(self, pymunkSpace, chassisCenterPoint):
        self.space = pymunkSpace
        #self.chassisXY = chassisCenterPoint
        self.__createBody__(chassisCenterPoint)
        
    def __createBody__(self, chassisXY):
        self.chassis_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.chassisWd, self.chHt)))
        self.chassis_body.position = chassisXY
        self.chassis_body.startPosition = Vec2d(self.chassis_body.position)#you can assign your own properties to body
        self.chassis_body.startAngle = self.chassis_body.angle        
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_body, (self.chassisWd, self.chHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.chassis_shape.color = 200, 200, 200
        self.chassis_shape.friction = 10.0
        self.space.add(self.chassis_body, self.chassis_shape)  
        for i in range(0, self.numLegs, 1):
            legA = RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, self.chassisWd, self.ori.LEFT)
            legB = RobotLeg(self.space, self.ownBodyShapeFilter, legA.leg_body, legA.legWd, self.ori.LEFT)
            self.legs.append(legA)
            self.legs.append(legB)
            legA = RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, self.chassisWd, self.ori.RIGHT)
            legB = RobotLeg(self.space, self.ownBodyShapeFilter, legA.leg_body, legA.legWd, self.ori.RIGHT)
            self.legs.append(legA)
            self.legs.append(legB)            
        #self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, self.chassisWd, self.ori.RIGHT))
        #def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight):

    def updatePosition(self, offsetXY):
        self.chassis_body.position = self.chassis_body.position + offsetXY 
        for leg in self.legs:
            leg.updatePosition(offsetXY)

    def arbiterCallback(self, arbiter, shapes):
        pass
        #print(arbiter.shapes)
        #print(arbiter.contact_point_set.points)
                
    def getCollission(self):
        a = self.chassis_body.each_arbiter(self.arbiterCallback, shapes)
        
    def getFullBodyStatesAndMotorRates(self):
        bodyStates = []; motorRates = []
        bodyStates.append(self.chassis_body.angle)
        bodyStates.append(self.chassis_body.position)
        for leg in self.legs:
            val = leg.getLegStatesAndMotorRates()
            bodyStates.append(val[0])
            motorRates.append(val[1])
        return (bodyStates, motorRates)
    
#     def setMotorRates(self, motorRates):
#         for leg in self.legs:
#             leg.setMotorRates(motorRates)
            
            