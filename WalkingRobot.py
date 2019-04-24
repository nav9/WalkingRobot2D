# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import pymunk
from pymunk import Vec2d, shapes
import random
from collections import defaultdict
import hashlib
import math
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from pygame import _numpysurfarray
from matplotlib.animation import FuncAnimation

#from threading import Thread, Lock

# class ActionsNetwork:#For now, a single instance of this is created which all walking robots can contribute to and access
#     def __init__(self):
#         #mutex = Lock()
#         self.hashID = {}
#         #IdHash = {}
#         #actionID = 0        
#     def addAction(self, actionList):#expects a list of numbers of strings or a combination of both
#         actionStr = ''.join(str(x)+',' for x in actionList)
#         #hashedStr = hashlib.md5(actionStr.encode()) #hash_object = hashlib.sha1(b'Hello World')        
#         if actionStr in self.hashID:
#             return
# #         self.mutex.acquire()
# #         try:
# #             self.actionID += 1
# #         finally:
# #             self.mutex.release()        
#         #self.actionID += 1 #should be protected by mutex if using threads
#         self.hashID[actionStr] = []#self.actionID
#         #self.IdHash[self.actionID] = actionStr
# 
# class TactileCortex:
#     def __init__(self, bodyRef):
#         self.body = bodyRef
#     def getTactileInputs(self):
#         return self.body.chassis_body.each_arbiter(self.contactInfo)
#     def contactInfo(self, arbiter):    
# #         print(arbiter.shapes)#gives the type of objects that are colliding [chassis,line seg]
# #         print(arbiter.contact_point_set.normal)#direction of contact
# #         print(arbiter.contact_point_set.points[0].distance)#distance is the penetration distance of the two shapes. Overlapping means it will be negative. This value is calculated as dot(point2 - point1), normal) and is ignored when you set the Arbiter.contact_point_set.
# #         print(arbiter.total_impulse)#Returns the impulse that was applied this step to resolve the collision Vec2d(xImpulse, yImpulse)
# #         print(arbiter.contact_point_set.points[0].point_a)#point_a and point_b are the contact position on the surface of each shape.
# #         print(arbiter.contact_point_set.points[0].point_b)
#         return arbiter.contact_point_set
# 
# class ActionsCortex:    
#     def __init__(self, bodyRef):
#         self.actionSeq = defaultdict(list)  
#         self.actionsNetworkPresent = False
#         self.randomRates = []
#         self.angleAccuracy = 10 #degrees 
#         self.distanceAccuracy = 10 #pixels        
#         self.body = bodyRef
#         for leg in self.body.legs:
#             leg.motor.rate = 0 #keep motor still when initialized
#         #TODO: load actions network
#     
#     def generateNewActions(self, stopActionSequence):
#         if len(self.randomRates) == 0:
#             if len(self.body.legs) > 0:
#                 self.randomRates = random.sample(self.body.legs[0].motor.legRateRange, len(self.body.legs)) #sample(range,numNumbers) = sampling without replacement. Generates unique random samples within range
#                 
#         if stopActionSequence:
#             self.randomRates = []
#             for leg in self.body.legs:
#                 leg.motor.rate = 0
#         else:             
#             for i in range(0, len(self.body.legs), 1):
#                 self.body.legs[i].motor.rate = self.randomRates[i]
#                 #---find which angle quadrant and distance radius the leg tip falls wrt the rotated body position
#                 bodyAng = round(math.degrees(self.body.chassis_body.angle)%360)
#                 legTip = self.body.legs[i].getTip();  chCen = self.body.chassis_body.position
#                 dist = abs(math.sqrt((legTip[0]-chCen[0])**2 + (legTip[1]-chCen[1])**2)) / self.distanceAccuracy
#                 ang = round((math.degrees(math.atan2((legTip[1]-chCen[1]), (legTip[0]-chCen[0])))-bodyAng)/self.angleAccuracy)
#                 self.body.actionNetwork.addAction([i, dist, ang, self.body.legs[i].motor.rate]) #HASH: [legID, distance, angle, motorRate]
#         
# class Brain:        
#     def __init__(self, bodyRef):
#         self.experience = 20 #number of action iterations it can handle
#         self.trainingExperienceCounter = 0        
#         self.motorCortex = ActionsCortex(bodyRef)
#         self.tactileCortex = TactileCortex(bodyRef)        
#         
#     def getSensoryInputsAndDecideWhatToDo(self):#TODO: add visual sensory input
#         if self.motorCortex.actionsNetworkPresent:
#             senses = self.tactileCortex.getTactileInputs()
#         else:
#             if self.trainingExperienceCounter == 0:
#                 self.motorCortex.generateNewActions(self.trainingExperienceCounter==0)
#                 self.trainingExperienceCounter = self.experience
#             else:
#                 self.motorCortex.generateNewActions(self.trainingExperienceCounter==0)
#                 self.trainingExperienceCounter -= 1  

class ActionNetwork:
    def __init__(self, execLen, legs):
        self.actionFile = 'actionNetwork_'+str(execLen)+'_'+legs+'.gpickle'
        self.graph = None
        self.__loadNetwork__()
    
    def addNode(self, node):
        self.graph.add_node(tuple(node))
        
    def addEdge(self, node1, node2, wt, expe):
        self.graph.add_edge(tuple(node1), tuple(node2), weight=wt, experience=tuple(expe))
    
    def getEdge(self, node1, node2):
        return self.graph.get_edge_data(tuple(node1), tuple(node2))
    
    def getSuccessorNodes(self, currNode):#edges going out of currNode
        if self.graph.out_degree[tuple(currNode)] == 0: return None 
        else: return self.graph.successors(tuple(currNode))
    
    def displayNetwork(self):
        #plt.subplot(111)
        plt.clf(); 
        #nx.draw_kamada_kawai(self.graph)
        #spring_pos = nx.spring_layout(self.graph)
        #spring_pos = nx.planar_layout(self.graph)        
        #spring_pos = nx.circular_layout(self.graph)
        spring_pos = nx.kamada_kawai_layout(self.graph)
        nx.draw_networkx(self.graph, pos=spring_pos, arrows=True, with_labels=False, node_size=20)
        #nx.draw_circular(self.graph)
        #nx.draw(self.graph, with_labels=False, font_weight='bold')
        plt.pause(0.001)
        plt.show(block=False)   
# __all__ = ['bipartite_layout',
#            'circular_layout',
#            'kamada_kawai_layout',
#            'random_layout',
#            'rescale_layout',
#            'shell_layout',
#            'spring_layout',
#            'spectral_layout',
#            'planar_layout',
#            'fruchterman_reingold_layout']        
        
    def saveNetwork(self):
        nx.write_gpickle(self.graph, self.actionFile)
        print('Saved network to '+self.actionFile)
    
    def __loadNetwork__(self):
        try:
            self.graph = nx.read_gpickle(self.actionFile)
        except:
            print('No '+self.actionFile+' found. Creating new action network.')
            self.graph = nx.MultiDiGraph()
#         finally:
            
    
class Directions:
    UP = 1
    DOWN = 2
    LEFT = 3    
    RIGHT = 4

class LegPart:#This is one leg part. Could be part A that's connected to the chassis or part B that's connected to part A
    def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight):
        self.space = pymunk.Space()
        self.ori = Directions()
        self.legLeftOrRight = self.ori.LEFT  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
        self.prevBodyXY = 0
        self.prevBodyWd = 0  # chassis width
        self.legWd = 20 #leg thickness (width)
        self.legHt = 2 #leg height
        self.legMass = 0.5
        self.relativeAnguVel = 0
        self.leg_body = None
        self.leg_shape = None
        self.pinJoint = None
        self.motor = None        
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.prevBodyXY = prevBody.position
        self.prevBodyWd = prevBodyWidth
        self.legLeftOrRight = leftOrRight
        self.__createLegPart__()
        self.__linkLegPartWithPrevBodyPart__(prevBody)
        self.experience = []
    
    def __createLegPart__(self):
        self.leg_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd, self.legHt)))
        if self.legLeftOrRight == self.ori.LEFT:
            self.leg_body.position = self.prevBodyXY - ((self.prevBodyWd / 2) + (self.legWd / 2), 0)            
        if self.legLeftOrRight == self.ori.RIGHT:
            self.leg_body.position = self.prevBodyXY + ((self.prevBodyWd / 2) + (self.legWd / 2), 0)
        self.leg_shape = pymunk.Poly.create_box(self.leg_body, (self.legWd, self.legHt))            
        self.leg_shape.filter = self.shapeFilter
        self.leg_shape.color = 200, 200, 200  
        self.leg_shape.friction = 10.0 
        self.space.add(self.leg_shape, self.leg_body) 
        
    def delete(self):
        self.space.remove(self.leg_shape, self.leg_body, self.pinJoint, self.motor)
    
    def getTip(self):  
        v = self.leg_shape.get_vertices()    
        if self.legLeftOrRight == self.ori.LEFT:            
            v = v[2].rotated(self.leg_shape.body.angle) + self.leg_shape.body.position #TODO/BUG: This will have to be changed based on polygon shape chosen in future
        if self.legLeftOrRight == self.ori.RIGHT:
            v = v[1].rotated(self.leg_shape.body.angle) + self.leg_shape.body.position #TODO/BUG: This will have to be changed based on polygon shape chosen in future        
        return Vec2d(v)
    
    def __linkLegPartWithPrevBodyPart__(self, prevBody):
        maxMotorRate = 5
        motorRateRangePieces = (maxMotorRate * 2 + 1) * 10
        #---link left leg A with Chassis
        if self.legLeftOrRight == self.ori.LEFT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (self.legWd / 2, 0), (-self.prevBodyWd / 2, 0))
        if self.legLeftOrRight == self.ori.RIGHT:
            self.pinJoint = pymunk.PinJoint(self.leg_body, prevBody, (-self.legWd / 2, 0), (self.prevBodyWd / 2, 0))            
        self.motor = pymunk.SimpleMotor(self.leg_body, prevBody, self.relativeAnguVel) 
        self.space.add(self.pinJoint, self.motor)
        self.motor.rate = 0
        self.motor.max_force = 10000000
        self.motor.legRateRange = np.linspace(-maxMotorRate, maxMotorRate, motorRateRangePieces)         
        
    def updatePosition(self, offsetXY):
        self.leg_body.position = self.leg_body.position + offsetXY 
        
    def getLegAngle(self):
        return round(math.degrees(self.leg_body.angle)%360)        

class RobotBody:
    def __init__(self, pymunkSpace, chassisCenterPoint, legCode):
        self.legsCode = legCode
        self.ownBodyShapeFilter = pymunk.ShapeFilter(group=1) #to prevent collisions between robot body parts
        self.ori = Directions()  #leg at left or right of chassis
        self.prevBodyWd = 30 #chassis width 
        self.chassisHt = 20 #chassis height
        self.chassisMass = 10
        self.chassis_body = None #chassis body
        self.chassis_shape = None #chassis shape 
        self.legs = []
        self.currentActionNode = []#node on the action network        
#         self.brain = None        
        self.space = pymunkSpace
        self.__createBody__(chassisCenterPoint)
        #self.actionNetwork = globalActionNetwork
        #self.__activateBrain__()
        
    def __createBody__(self, chassisXY):
        self.chassis_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.prevBodyWd, self.chassisHt)))
        self.chassis_body.body_type = pymunk.Body.KINEMATIC
        self.chassis_body.position = chassisXY
        self.chassis_body.startPosition = Vec2d(self.chassis_body.position[0], self.chassis_body.position[1])
#         self.chassis_body.startAngle = self.chassis_body.angle        
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_body, (self.prevBodyWd, self.chassisHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.setNormalRobotColor() 
        self.chassis_shape.friction = 10.0
        self.space.add(self.chassis_shape, self.chassis_body) 
        self.createLegs()
        self.makeRobotDynamic()
    
    def createLegs(self):
        s = self.legsCode.split("#")
        lt = s[0]; rt = s[1]; rt = rt[::-1]#reverse string rt
        for s in lt.split(","):#number of left legs
            if len(s) == 1:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.prevBodyWd, self.ori.LEFT); self.legs.append(leftLegA)
            if len(s) == 2:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.prevBodyWd, self.ori.LEFT); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.leg_body, leftLegA.legWd, self.ori.LEFT); self.legs.append(leftLegB)                
        for s in rt.split(","):#number of right legs
            if len(s) == 1:
                rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.prevBodyWd, self.ori.RIGHT); self.legs.append(rightLegA)
            if len(s) == 2:
                rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.chassis_body, self.prevBodyWd, self.ori.RIGHT); self.legs.append(rightLegA) 
                rightLegB = LegPart(self.space, self.ownBodyShapeFilter, rightLegA.leg_body, rightLegA.legWd, self.ori.RIGHT); self.legs.append(rightLegB)                        
        
    def setExperience(self, expe):       
        for leg in self.legs:
            leg.experience[:] = []
        while len(expe) > 0:
            for leg in self.legs:
                leg.experience.append(expe.pop(0))
             
    def reinitializeWithRandomValues(self, seqLen):       
        for leg in self.legs:
            leg.experience[:] = []
            for i in range(0, seqLen, 1):
                thisRate = random.choice(leg.motor.legRateRange)
                leg.experience.append(thisRate)                                    
    
    def getValues(self):
        ex = []
        for leg in self.legs:
            ex.extend(leg.experience)
        return ex
    
    def setValuesWithClamping(self, ex, seqLen):
        j = 0
        for leg in self.legs:
            leg.experience[:] = []
            for i in range(0, seqLen, 1): 
                v = ex[j]
                if v < leg.motor.legRateRange[0] or v > leg.motor.legRateRange[-1]:
                    v = random.choice(leg.motor.legRateRange)
                leg.experience.append(v)
                j += 1
    
    def setMotorRateForSequence(self, seqId):
        for leg in self.legs:
            leg.motor.rate = leg.experience[seqId]
    
    def stopMotion(self):
        for leg in self.legs:
            leg.motor.rate = 0
                        
    def setFocusRobotColor(self):
        self.chassis_shape.color = (190, 0, 0)
        
    def setNormalRobotColor(self):
        self.chassis_shape.color = (170, 170, 170)
                
    def setImaginaryRobotColor(self):
        self.chassis_shape.color = (110, 110, 110)  
        for leg in self.legs:
            leg.leg_shape.color = (110, 110, 110)
        
    def makeRobotDynamic(self):
        self.chassis_body.body_type = pymunk.Body.DYNAMIC
        self.chassis_body.mass = self.chassisMass
        self.chassis_body.moment = pymunk.moment_for_box(self.chassisMass, (self.prevBodyWd, self.chassisHt))            
        
    def delete(self):
        self.space.remove(self.chassis_shape); self.space.remove(self.chassis_body)
        for legPart in self.legs:
            legPart.delete()
        self.legs[:] = []#clear the list
        
#     def __activateBrain__(self):
#         self.brain = Brain(self)
        
#     def brainActivity(self):
#         self.brain.getSensoryInputsAndDecideWhatToDo()
        
    def getPosition(self):
        return self.chassis_body.position
        
    def updatePosition(self, offsetXY):
        self.chassis_body.position += offsetXY 
        self.chassis_body.startPosition += offsetXY
        for leg in self.legs:
            leg.updatePosition(offsetXY)
    
#     def getFullBodyStatesAndMotorRates(self):
#         bodyStates = []; motorRates = []
#         bodyStates.append(self.chassis_body.angle)
#         bodyStates.append(self.chassis_body.position)
#         for leg in self.legs:
#             val = leg.getLegStatesAndMotorRates()
#             bodyStates.append(val[0])
#             motorRates.append(val[1])
#         return (bodyStates, motorRates)
    
    def getBodyAngle(self):
        return round(math.degrees(self.chassis_body.angle)%360)
    
    def getUniqueBodyAngles(self):
        ang = [self.roundToNearest(self.getBodyAngle())]
        for leg in self.legs:
            ang.append(self.roundToNearest(leg.getLegAngle()))
        return ang
    
    def setBodyPositionAndAngles(self, pos, angles, offset):
        p = pos.pop(0)
        self.chassis_body.position = Vec2d(p[0], p[1]) + offset
        self.chassis_body.startPosition = Vec2d(p[0], p[1]) + offset
        self.chassis_body.angle = math.radians(angles.pop(0))
        for leg in self.legs:
            p = pos.pop(0)
            leg.leg_body.position = Vec2d(p[0], p[1]) + offset
            leg.leg_body.angle = math.radians(angles.pop(0))
        return pos    
    
    def getPositions(self):
        pos = [Vec2d(self.chassis_body.position)]
        for leg in self.legs:
            pos.append(Vec2d(leg.leg_body.position))
        return pos
        
    def roundToNearest(self, num):
        roundOffPrecision = 5
        rem = num % roundOffPrecision
        if rem < roundOffPrecision / 2: num = int(num/roundOffPrecision) * roundOffPrecision
        else: num = int((num+roundOffPrecision) / roundOffPrecision) * roundOffPrecision
        return num
    
    