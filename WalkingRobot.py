import pymunk
from pymunk import Vec2d, shapes
import random
from collections import defaultdict
import hashlib
#from threading import Thread, Lock

#from abc import ABCMeta, abstractmethod
# class AbstractActionsNetwork:
#     __metaclass__ = ABCMeta
#     hashID = {}
#     IdHash = {}
#         
#     @abstractmethod
#     def setValues(self):#The force applied by the action
#         pass    
# class LegActions(AbstractActions):    
#     legRate = 0
#     max_force = 10000000
#     legRateRange = []


class ActionsNetwork:#For now, a single instance of this is created which all walking robots can contribute to and access
    #mutex = Lock()
    hashID = {}
    IdHash = {}
    actionID = 0
    def __init__(self):
        pass
    def addAction(self, actionList):#expects a list of numbers of strings or a combination of both
        actionStr = ''.join(str(x) for x in actionList)
        hashedStr = hashlib.md5(actionStr.encode()) #hash_object = hashlib.sha1(b'Hello World')
#         self.mutex.acquire()
#         try:
#             self.actionID += 1
#         finally:
#             self.mutex.release()        
        self.actionID += 1 #should be protected by mutex if using threads
        self.hashID[hashedStr] = self.actionID
        self.IdHash[self.actionID] = hashedStr

class TactileCortex:
    body = None
    def __init__(self, bodyRef):
        self.body = bodyRef
    def getTactileInputs(self):
        self.body.chassis_body.each_arbiter(self.contactInfo)
    def contactInfo(self, arbiter):    
#         print(arbiter.shapes)#gives the type of objects that are colliding [chassis,line seg]
#         print(arbiter.contact_point_set.normal)#direction of contact
#         print(arbiter.contact_point_set.points[0].distance)#distance is the penetration distance of the two shapes. Overlapping means it will be negative. This value is calculated as dot(point2 - point1), normal) and is ignored when you set the Arbiter.contact_point_set.
#         print(arbiter.total_impulse)#Returns the impulse that was applied this step to resolve the collision Vec2d(xImpulse, yImpulse)
#         print(arbiter.contact_point_set.points[0].point_a)#point_a and point_b are the contact position on the surface of each shape.
#         print(arbiter.contact_point_set.points[0].point_b)
        pass

class ActionsCortex:
    actionSeq = defaultdict(list)
    body = None    
    def __init__(self, bodyRef):
        self.body = bodyRef
        for leg in self.body.legs:
            leg.motor.rate = 0

class Brain:
    motorCortex = None
    tactileCortex = None
    def __init__(self, bodyRef):
        self.motorCortex = ActionsCortex(bodyRef)
        self.tactileCortex = TactileCortex(bodyRef)        
    def getSensoryInputs(self):
        self.tactileCortex.getTactileInputs()
        #TODO: add visual input
    def decideWhatToDo(self):
        pass
    
class Directions:
    UP = 1
    DOWN = 2
    LEFT = 3    
    RIGHT = 4

class LegPart:#This is one leg part. Could be part A that's connected to the chassis or part B that's connected to part A
    space = pymunk.Space()
    shapeFilter = None
    ori = Directions()
    legLeftOrRight = ori.LEFT  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
    prevBodyXY = 0
    chassisWd = 0  # chassis width
    legWd = 20 #leg thickness (width)
    legHt = 2 #leg height
    legMass = 0.5
    relativeAnguVel = 0
    leg_body = None
    leg_shape = None
    pinJoint = None
    motor = None
    
    def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight):
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.prevBodyXY = prevBody.position
        self.chassisWd = prevBodyWidth
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
        self.leg_shape.color = 200, 200, 200  
        self.leg_shape.friction = 10.0 
        self.space.add(self.leg_body, self.leg_shape)   
    
    def __linkLegPartWithPrevBodyPart__(self, prevBody):
        maxMotorRate = 5
        motorRateRangeStep = 2
        #---link left leg A with Chassis
        if self.legLeftOrRight == self.ori.LEFT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (self.legWd / 2, 0), (-self.chassisWd / 2, 0))
        if self.legLeftOrRight == self.ori.RIGHT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (-self.legWd / 2, 0), (self.chassisWd / 2, 0))            
        self.motor = pymunk.SimpleMotor(self.leg_body, prevBody, self.relativeAnguVel) 
        self.space.add(self.pinJoint, self.motor)
        self.motor.rate = 5
        self.motor.max_force = 10000000
        self.motor.legRateRange = range(-maxMotorRate, maxMotorRate, motorRateRangeStep)#range(start,stop,step)         
        
    def updatePosition(self, offsetXY):
        self.leg_body.position = self.leg_body.position + offsetXY 

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
    chassisHt = 20 #chassis height
    chassisMass = 10
    chassis_body = None  #chassis body
    chassis_shape = None  #chassis shape    
    legs = []
    numLegsOnEachSide = 1
    actionNetwork = None
    brain = None
    trainingSupport = []
    
    def __init__(self, pymunkSpace, chassisCenterPoint, globalActionNetwork):
        self.space = pymunkSpace
        self.__createBody__(chassisCenterPoint)
        self.actionNetwork = globalActionNetwork
        self.createTrainingHolder()
        #self.__activateBrain__()
        
    def __createBody__(self, chassisXY):
        self.chassis_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.chassisWd, self.chassisHt)))
        self.chassis_body.position = chassisXY
        self.chassis_body.startPosition = Vec2d(self.chassis_body.position)#you can assign your own properties to body
        self.chassis_body.startAngle = self.chassis_body.angle        
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_body, (self.chassisWd, self.chassisHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.chassis_shape.color = 200, 200, 200
        self.chassis_shape.friction = 10.0
        self.space.add(self.chassis_body, self.chassis_shape)  
        for i in range(0, self.numLegsOnEachSide, 1):
            leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.chassisWd, self.ori.LEFT)
            leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.leg_body, leftLegA.legWd, self.ori.LEFT)
            self.legs.append(leftLegA)
            self.legs.append(leftLegB)
            rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.chassisWd, self.ori.RIGHT)
            rightLegB = LegPart(self.space, self.ownBodyShapeFilter, rightLegA.leg_body, rightLegA.legWd, self.ori.RIGHT)
            self.legs.append(rightLegA)
            self.legs.append(rightLegB)   
            
    def createTrainingHolder(self):#holds the robot body in space so it can learn some moves
        supportX1 = self.chassis_body.position[0]-self.chassisWd/2; supportWd = self.chassisWd
        supportY1 = self.chassis_body.position[1]-(self.chassisHt/2)-1#increasing this value makes it support1_shape get positioned higher
        support1_shape = pymunk.Segment(pymunk.Body(body_type=pymunk.Body.STATIC), (supportX1, supportY1), (supportX1+supportWd, supportY1), 1.0)
        support1_shape.friction = 1.0        
        self.space.add(support1_shape)        
        self.trainingSupport.append(support1_shape) 
        supportX2 = self.chassis_body.position[0]-self.chassisWd/2; supportWd = self.chassisWd
        supportY2 = self.chassis_body.position[1]+(self.chassisHt/2)+1#increasing this value makes it support1_shape get positioned higher
        support2_shape = pymunk.Segment(pymunk.Body(body_type=pymunk.Body.STATIC), (supportX2, supportY2), (supportX2+supportWd, supportY2), 1.0)
        support2_shape.friction = 1.0        
        self.space.add(support2_shape)        
        self.trainingSupport.append(support2_shape)         
    
    def removeTrainingHolder(self):
        for holder in self.trainingSupport:
            self.space.remove(holder)
        
    def __activateBrain__(self):
        self.brain = Brain(self)
        
    def brainActivity(self):
        self.brain.getSensoryInputs()
        self.brain.decideWhatToDo()
        
    def updatePosition(self, offsetXY):
        self.chassis_body.position = self.chassis_body.position + offsetXY 
        for leg in self.legs:
            leg.updatePosition(offsetXY)
        
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
            
            