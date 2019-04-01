import pymunk
from pymunk import Vec2d
import random

class Directions:
    UP = 1
    DOWN = 2
    LEFT = 3    
    RIGHT = 4

class RobotLeg:
    space = pymunk.Space()
    shapeFilter = None
    ori = Directions()
    legPosition = ori.LEFT  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
    focusRobotXY = 0
    chWd = 0  # chassis width
    chHt = 0  # chassis height
    legWd_a = 15 #leg thickness (width)
    legHt_a = 2 #leg height
    legWd_b = 20
    legHt_b = 2
    legMass = 1
    relativeAnguVel = 0
    legA_body = None
    legA_shape = None
    legB_body = None
    legB_shape = None
    pinJ_LegBLegA = None
    motor_LegBLegA = None
    pinJ_LegAChassis = None
    motor_LegAChassis = None      
    
    def __init__(self, pymunkSpace, ownBodyShapeFilter, chassisBody, chassisCenterPosition, chassisWidth, chassisHeight, leftOrRight, rate):
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.focusRobotXY = chassisCenterPosition
        self.chWd = chassisWidth
        self.chHt = chassisHeight
        self.legPosition = leftOrRight
        self.__createLegA__()
        self.__createLegB__()
        self.__linkLegsAndChassis__(chassisBody, rate)
    
    def __createLegA__(self):
        self.legA_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd_a, self.legHt_a)))
        if self.legPosition == self.ori.LEFT:
            self.legA_body.position = self.focusRobotXY - ((self.chWd / 2) + (self.legWd_a / 2), 0)
        if self.legPosition == self.ori.RIGHT:
            self.legA_body.position = self.focusRobotXY + ((self.chWd / 2) + (self.legWd_a / 2), 0)
        self.legA_body.startPosition = Vec2d(self.legA_body.position)
        self.legA_body.startAngle = self.legA_body.angle
        self.legA_shape = pymunk.Poly.create_box(self.legA_body, (self.legWd_a, self.legHt_a))        
        self.legA_shape.filter = self.shapeFilter
        self.legA_shape.color = 255, 0, 0  
        self.legA_shape.friction = 10.0 
        self.space.add(self.legA_body, self.legA_shape)
        
    def __createLegB__(self):
        self.legB_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd_b, self.legHt_b)))
        if self.legPosition == self.ori.LEFT:
            self.legB_body.position = self.legA_body.position - ((self.legWd_a / 2) + (self.legWd_b / 2), 0)
        if self.legPosition == self.ori.RIGHT:
            self.legB_body.position = self.legA_body.position + ((self.legWd_a / 2) + (self.legWd_b / 2), 0)
        self.legB_body.startPosition = Vec2d(self.legB_body.position)
        self.legB_body.startAngle = self.legB_body.angle
        self.legB_shape = pymunk.Poly.create_box(self.legB_body, (self.legWd_b, self.legHt_b))
        self.legB_shape.filter = self.shapeFilter        
        self.legB_shape.color = 0, 255, 0 
        self.legB_shape.friction = 10.0 
        self.space.add(self.legB_body, self.legB_shape)
    
    def __linkLegsAndChassis__(self, chassisBody, rate):
        #---link left leg B with left leg A       
        if self.legPosition == self.ori.LEFT:
            self.pinJ_LegBLegA = pymunk.PinJoint(self.legB_body, self.legA_body, (self.legWd_b / 2, 0), (-self.legWd_a / 2, 0))  # anchor point coordinates are wrt the body; not the space
        if self.legPosition == self.ori.RIGHT:
            self.pinJ_LegBLegA = pymunk.PinJoint(self.legB_body, self.legA_body, (-self.legWd_b / 2, 0), (self.legWd_a / 2, 0))  # anchor point coordinates are wrt the body; not the space                        
        self.motor_LegBLegA = pymunk.SimpleMotor(self.legB_body, self.legA_body, self.relativeAnguVel)
        self.motor_LegBLegA.max_force = 10000000
        self.space.add(self.pinJ_LegBLegA, self.motor_LegBLegA)
        self.motor_LegBLegA.rate = rate
        #---link left leg A with Chassis
        if self.legPosition == self.ori.LEFT:
            self.pinJ_LegAChassis = pymunk.PinJoint(self.legA_body, chassisBody, (self.legWd_a / 2, 0), (-self.chWd / 2, 0))
        if self.legPosition == self.ori.RIGHT:
            self.pinJ_LegAChassis = pymunk.PinJoint(self.legA_body, chassisBody, (-self.legWd_a / 2, 0), (self.chWd / 2, 0))            
        self.motor_LegAChassis = pymunk.SimpleMotor(self.legA_body, chassisBody, self.relativeAnguVel) 
        self.motor_LegAChassis.max_force = 10000000       
        self.space.add(self.pinJ_LegAChassis, self.motor_LegAChassis)
        self.motor_LegAChassis.rate = rate
    
    def updatePosition(self, xOffset, yOffset):
        self.legA_body.position = self.legA_body.position + (xOffset, yOffset) 
        self.legB_body.position = self.legB_body.position + (xOffset, yOffset) 

    
class RobotBody:
    space = None
    ownBodyShapeFilter = pymunk.ShapeFilter(group=1)#to prevent collisions between robot body parts
    ori = Directions()  # leg at left or right of chassis
    chWd = 30 #chassis width 
    chHt = 20 #chassis height
    chassisMass = 10
    chassis_body = None  # chassis body
    chassis_shape = None  # chassis shape
    maxMotorRate = 5
    legs = []
    
    def __init__(self, pymunkSpace, chassisCenterPoint):
        self.space = pymunkSpace
        #self.chassisXY = chassisCenterPoint
        self.__createBody__(chassisCenterPoint)
        
    def __createBody__(self, chassisXY):
        self.chassis_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.chWd, self.chHt)))
        self.chassis_body.position = chassisXY
        self.chassis_body.startPosition = Vec2d(self.chassis_body.position)#you can assign your own properties to body
        self.chassis_body.startAngle = self.chassis_body.angle        
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_body, (self.chWd, self.chHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.chassis_shape.color = 200, 200, 200
        self.chassis_shape.friction = 10.0
        self.space.add(self.chassis_body, self.chassis_shape)
        # print("chassis position");print(self.chassis_body.position)
        self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, chassisXY, self.chWd, self.chHt, self.ori.LEFT, random.choice(range(-self.maxMotorRate, self.maxMotorRate,2))))
        self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, chassisXY, self.chWd, self.chHt, self.ori.RIGHT, random.choice(range(-self.maxMotorRate, self.maxMotorRate,2))))
        #self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, chassisXY, self.chWd, self.chHt, self.ori.LEFT, random.choice(range(-self.maxMotorRate, self.maxMotorRate,2))))
        #self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_body, chassisXY, self.chWd, self.chHt, self.ori.RIGHT, random.choice(range(-self.maxMotorRate, self.maxMotorRate,2))))        

    def updatePosition(self, xOffset, yOffset):
        self.chassis_body.position = self.chassis_body.position + (xOffset, yOffset) 
        for leg in self.legs:
            leg.updatePosition(xOffset, yOffset)
        