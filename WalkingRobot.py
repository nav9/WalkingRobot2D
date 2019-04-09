import pymunk
from pymunk import Vec2d, shapes
import random
from collections import defaultdict
import hashlib
import math
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
        actionStr = ''.join(str(x)+',' for x in actionList)
        #hashedStr = hashlib.md5(actionStr.encode()) #hash_object = hashlib.sha1(b'Hello World')        
        if actionStr in self.hashID:
            print('---------skipped')
            return
        else:
            print(actionStr)
#         self.mutex.acquire()
#         try:
#             self.actionID += 1
#         finally:
#             self.mutex.release()        
        self.actionID += 1 #should be protected by mutex if using threads
        self.hashID[actionStr] = self.actionID
        self.IdHash[self.actionID] = actionStr

class TactileCortex:
    body = None
    def __init__(self, bodyRef):
        self.body = bodyRef
    def getTactileInputs(self):
        return self.body.chassis_body.each_arbiter(self.contactInfo)
    def contactInfo(self, arbiter):    
#         print(arbiter.shapes)#gives the type of objects that are colliding [chassis,line seg]
#         print(arbiter.contact_point_set.normal)#direction of contact
#         print(arbiter.contact_point_set.points[0].distance)#distance is the penetration distance of the two shapes. Overlapping means it will be negative. This value is calculated as dot(point2 - point1), normal) and is ignored when you set the Arbiter.contact_point_set.
#         print(arbiter.total_impulse)#Returns the impulse that was applied this step to resolve the collision Vec2d(xImpulse, yImpulse)
#         print(arbiter.contact_point_set.points[0].point_a)#point_a and point_b are the contact position on the surface of each shape.
#         print(arbiter.contact_point_set.points[0].point_b)
        return arbiter.contact_point_set

class ActionsCortex:
    actionSeq = defaultdict(list)
    body = None    
    actionsNetworkPresent = False
    randomRates = []
    angleAccuracy = 10 #degrees 
    
    def __init__(self, bodyRef):
        self.body = bodyRef
        for leg in self.body.legs:
            leg.motor.rate = 0#keep motor still when initialized
        #TODO: load actions network
    
    def generateNewActions(self, stopActionSequence):
        if len(self.randomRates) == 0:
            if len(self.body.legs) > 0:
                self.randomRates = random.sample(self.body.legs[0].motor.legRateRange, len(self.body.legs)) #sample(range,numNumbers) = sampling without replacement. Generates unique random samples within range
                #print(self.randomRates)
                
        if stopActionSequence:
            self.randomRates = []
            for leg in self.body.legs:
                leg.motor.rate = 0
        else: 
            for i in range(0, len(self.body.legs), 1):
                self.body.legs[i].motor.rate = self.randomRates[i]
                ang = round((math.degrees(self.body.legs[i].leg_body.angle)%360)/self.angleAccuracy)
                self.body.actionNetwork.addAction([ang, self.body.legs[i].motor.rate])
        
    def stopRandomActionNetworkCreation(self):
        self.body.chassis_body.body_type = pymunk.Body.DYNAMIC
        self.body.chassis_body.mass = self.body.chassisMass
        self.body.chassis_body.moment = pymunk.moment_for_box(self.body.chassisMass, (self.body.prevBodyWd, self.body.chassisHt))
        
class Brain:
    motorCortex = None
    tactileCortex = None
    experience = 20#number of action iterations it can handle
    trainingExperienceCounter = 0
        
    def __init__(self, bodyRef):
        self.motorCortex = ActionsCortex(bodyRef)
        self.tactileCortex = TactileCortex(bodyRef)        
    def getSensoryInputsAndDecideWhatToDo(self):#TODO: add visual sensory input
        if self.motorCortex.actionsNetworkPresent:
            senses = self.tactileCortex.getTactileInputs()
        else:
            if self.trainingExperienceCounter == 0:
                self.motorCortex.generateNewActions(self.trainingExperienceCounter==0)
                self.trainingExperienceCounter = self.experience
            else:
                self.motorCortex.generateNewActions(self.trainingExperienceCounter==0)
                self.trainingExperienceCounter -= 1  

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
    prevBodyWd = 0  # chassis width
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
        self.prevBodyWd = prevBodyWidth
        self.legLeftOrRight = leftOrRight
        self.__createLegPart__()
        self.__linkLegPartWithPrevBodyPart__(prevBody)
    
    def __createLegPart__(self):
        self.leg_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd, self.legHt)))
        if self.legLeftOrRight == self.ori.LEFT:
            self.leg_body.position = self.prevBodyXY - ((self.prevBodyWd / 2) + (self.legWd / 2), 0)
        if self.legLeftOrRight == self.ori.RIGHT:
            self.leg_body.position = self.prevBodyXY + ((self.prevBodyWd / 2) + (self.legWd / 2), 0)
        self.leg_body.startPosition = Vec2d(self.leg_body.position)
        self.leg_body.startAngle = self.leg_body.angle
        self.leg_shape = pymunk.Poly.create_box(self.leg_body, (self.legWd, self.legHt))        
        self.leg_shape.filter = self.shapeFilter
        self.leg_shape.color = 200, 200, 200  
        self.leg_shape.friction = 10.0 
        self.space.add(self.leg_body, self.leg_shape)   
    
    def __linkLegPartWithPrevBodyPart__(self, prevBody):
        maxMotorRate = 5
        motorRateRangeStep = 3
        #---link left leg A with Chassis
        if self.legLeftOrRight == self.ori.LEFT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (self.legWd / 2, 0), (-self.prevBodyWd / 2, 0))
        if self.legLeftOrRight == self.ori.RIGHT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (-self.legWd / 2, 0), (self.prevBodyWd / 2, 0))            
        self.motor = pymunk.SimpleMotor(self.leg_body, prevBody, self.relativeAnguVel) 
        self.space.add(self.pinJoint, self.motor)
        #self.motor.rate = 0.1
        self.motor.max_force = 10000000
        self.motor.legRateRange = range(-maxMotorRate, maxMotorRate, motorRateRangeStep)#range(start,stop,step)         
        
    def updatePosition(self, offsetXY):
        self.leg_body.position = self.leg_body.position + offsetXY 

    
class RobotBody:
    space = None
    ownBodyShapeFilter = pymunk.ShapeFilter(group=1)#to prevent collisions between robot body parts
    ori = Directions()  #leg at left or right of chassis
    prevBodyWd = 30 #chassis width 
    chassisHt = 20 #chassis height
    chassisMass = 10
    chassis_body = None  #chassis body
    chassis_shape = None  #chassis shape    
    legs = []
    numLegsOnEachSide = 1
    actionNetwork = None
    brain = None
    
    def __init__(self, pymunkSpace, chassisCenterPoint, globalActionNetwork):
        self.space = pymunkSpace
        self.__createBody__(chassisCenterPoint)
        self.actionNetwork = globalActionNetwork
        self.__activateBrain__()
        
    def __createBody__(self, chassisXY):
        self.chassis_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.prevBodyWd, self.chassisHt)))
        self.chassis_body.body_type = pymunk.Body.KINEMATIC
        self.chassis_body.position = chassisXY
        self.chassis_body.startPosition = Vec2d(self.chassis_body.position)#you can assign your own properties to body
        self.chassis_body.startAngle = self.chassis_body.angle        
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_body, (self.prevBodyWd, self.chassisHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.chassis_shape.color = 170, 170, 170
        self.chassis_shape.friction = 10.0
        self.space.add(self.chassis_body, self.chassis_shape)  
        for i in range(0, self.numLegsOnEachSide, 1):
            leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.prevBodyWd, self.ori.LEFT)
            leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.leg_body, leftLegA.legWd, self.ori.LEFT)
            self.legs.append(leftLegA)
            self.legs.append(leftLegB)
            rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.prevBodyWd, self.ori.RIGHT)
            rightLegB = LegPart(self.space, self.ownBodyShapeFilter, rightLegA.leg_body, rightLegA.legWd, self.ori.RIGHT)
            self.legs.append(rightLegA)
            self.legs.append(rightLegB)  
        
    def __activateBrain__(self):
        self.brain = Brain(self)
        
    def brainActivity(self):
        self.brain.getSensoryInputsAndDecideWhatToDo()
        
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
            
            