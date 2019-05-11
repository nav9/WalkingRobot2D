# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import math
import pymunk
import random
import hashlib
import numpy as np
import networkx as nx
from DE import Constants
from StatesAndSensors import *
import matplotlib.pyplot as plt
from pymunk import Vec2d, shapes
from pygame import _numpysurfarray
from collections import defaultdict
from WalkingRobot import Directions
from matplotlib.animation import FuncAnimation
from StatesAndSensors import TactileSensor

class LegAppendage:
    A = 0
    B = 1
    C = 2

class LegPartActionNetwork:#stores actions of each movable part as a node
    def __init__(self, legs):
        self.actionFile = 'actionNetwork_'+legs+'.gpickle'
        self.graph = None
        self.fig = plt.figure()
        self.fig.patch.set_facecolor('black')        
        plt.rcParams['axes.facecolor'] = 'black'
        plt.rcParams['patch.facecolor'] = 'black'
        self.fig.canvas.toolbar.pack_forget()#remove bottom bar        
        self.__loadNetwork__()
    
    def addNode(self, node): self.graph.add_node(tuple(node))        
    def addEdge(self, node1, node2, wt, rate, duration):
        edge = self.graph.get_edge_data(tuple(node1), tuple(node2))
        if not edge == None:
            for e in edge:
                if edge[e]['rate']==rate and edge[e]['duration']==duration:
                    return #because edge is already present 
        self.graph.add_edge(tuple(node1), tuple(node2), weight=wt, rate=rate, duration=duration)    
        
    def getEdge(self, node1, node2): return self.graph.get_edge_data(tuple(node1), tuple(node2))
    
    def getSuccessorNodes(self, currNode):#edges going out of currNode
        if self.graph.out_degree[tuple(currNode)] == 0: return None 
        else: return self.graph.successors(tuple(currNode))
    
    def displayNetwork(self):    
        plt.clf(); posType = nx.spring_layout(self.graph)
        nx.draw_networkx(self.graph, pos=posType, arrows=True, with_labels=False, node_size=20, edge_color='green', node_color='green', alpha=0.5, arrowsize=5)
        plt.pause(0.001); plt.show(block=False)  
        print('Num nodes in action network: '+str(nx.number_of_nodes(self.graph)))     
        
    def saveNetwork(self):
        nx.write_gpickle(self.graph, self.actionFile)
        self.actionNetworkProperties()
        print('Saved network to '+self.actionFile)
    
    def __loadNetwork__(self):
        try:
            self.graph = nx.read_gpickle(self.actionFile)
        except:
            print('No '+self.actionFile+' found. Creating new action network.')
            self.graph = nx.MultiDiGraph()
        if not nx.is_empty(self.graph):
            #self.displayNetwork()
            self.actionNetworkProperties()
    
    def actionNetworkProperties(self):
        print('Action network properties:')
        print('Num nodes: '+str(nx.number_of_nodes(self.graph)))
        print('Num edges: '+str(nx.number_of_edges(self.graph)))
        print('Density: '+str(nx.density(self.graph)))
        print('Num self loops: '+str(nx.number_of_selfloops(self.graph)))
    
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class LearningRobotLegPart:#This is one leg part. Could be part A that's connected to the chassis or part B that's connected to part A
    def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight):
        self.space = pymunk.Space()
        d = Directions()
        self.ori = d.getDirn()
        self.leftRight = self.ori['LEFT']  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
        self.prevBodyXY = 0
        self.chassisWd = 0  # chassis width
        self.legWd = 20 #leg thickness (width)
        self.legHt = 2 #leg height
        self.legMass = 0.1 #kg
        self.relativeAnguVel = 0
        self.obj_body = None
        self.leg_shape = None
        self.pinJoint = None
        self.motor = None        
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.prevBodyXY = prevBody.position
        self.chassisWd = prevBodyWidth
        self.leftRight = leftOrRight        
        self.__createLegPart__()
        self.__linkLegPartWithPrevBodyPart__(prevBody)
        self.state = Freeze(self)
        self.decimalPrecision = 2
    
    def __createLegPart__(self):
        self.obj_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd, self.legHt)))
        if self.leftRight == self.ori['LEFT']: self.obj_body.position = self.prevBodyXY - ((self.chassisWd / 2) + (self.legWd / 2), 0)            
        if self.leftRight == self.ori['RIGHT']: self.obj_body.position = self.prevBodyXY + ((self.chassisWd / 2) + (self.legWd / 2), 0)
        self.leg_shape = pymunk.Poly.create_box(self.obj_body, (self.legWd, self.legHt))            
        self.leg_shape.filter = self.shapeFilter
        self.leg_shape.color = 200, 200, 200  
        self.leg_shape.friction = 10.0 
        self.space.add(self.leg_shape, self.obj_body) 
        
    def delete(self): self.space.remove(self.leg_shape, self.obj_body, self.pinJoint, self.motor)
    
    def getTip(self):  
        v = self.leg_shape.get_vertices()    
        if self.leftRight == self.ori['LEFT']: v = v[2].rotated(self.leg_shape.body.angle) + self.leg_shape.body.position #TODO/BUG: This will have to be changed based on polygon shape chosen in future
        if self.leftRight == self.ori['RIGHT']: v = v[1].rotated(self.leg_shape.body.angle) + self.leg_shape.body.position #TODO/BUG: This will have to be changed based on polygon shape chosen in future        
        return Vec2d(round(v[0], self.decimalPrecision), round(v[1], self.decimalPrecision))
    
    def __linkLegPartWithPrevBodyPart__(self, prevBody):
        maxMotorRate = 5; fractionOfSec = 5; minMovtDuration = 1/fractionOfSec; maxMovtDuration = 2
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
        self.motor.legMovtDurationRange = np.linspace(minMovtDuration, maxMovtDuration, fractionOfSec)         
        
    def updatePosition(self, offsetXY): self.obj_body.position = self.obj_body.position + offsetXY
    def __getNodeUID__(self, quadrant):        
        return (self.id, self.leftRight, quadrant)         
#     def getLegAngle(self): return round(math.degrees(self.obj_body.angle)%360)        

class LearningRobot:
    def __init__(self, world, chassisCenterPoint, legCode, actionNet):
        self.world = world
        self.legsCode = legCode
        self.actions = actionNet
        self.ownBodyShapeFilter = pymunk.ShapeFilter(group=1) #to prevent collisions between robot body parts
        d = Directions()  #leg at left or right of chassis
        self.ori = d.getDirn()
        self.chassisWd = 30 #chassis width 
        self.chassisHt = 20 #chassis height
        self.chassisMass = 5 #kg
        self.obj_body = None #chassis body
        self.chassis_shape = None #chassis shape 
        self.legs = []     
        self.quadrantAccuracy = 5 #pixels
        self.__createBody__(chassisCenterPoint)
        self.state = None
        #---register sensors
        self.sensors = []
        self.sensors.append(TactileSensor(world, self))
        for leg in self.legs: self.sensors.append(TactileSensor(world, leg))
        self.sensors.append(AngleSensor())
        self.sensors.append(DisplacementSensor())
        
    def run(self):
        for sen in self.sensors:
            sen.get()
        if not self.state == None:
            self.state.run()
    
    def __createBody__(self, chassisXY):
        self.obj_body = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.chassisWd, self.chassisHt)))
        self.obj_body.body_type = pymunk.Body.KINEMATIC
        self.obj_body.position = chassisXY
        self.obj_body.startPosition = Vec2d(self.obj_body.position[0], self.obj_body.position[1])
        self.chassis_shape = pymunk.Poly.create_box(self.obj_body, (self.chassisWd, self.chassisHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.setNormalRobotColor() 
        self.chassis_shape.friction = 10.0
        self.world.space.add(self.chassis_shape, self.obj_body) 
        self.createLegs()
        #self.makeRobotDynamic()
    
    def setState(self, aState): self.state = aState
    def createLegs(self):
        s = self.legsCode.split("#")
        lt = s[0]; rt = s[1]; rt = rt[::-1]#reverse string rt

        for s in lt.split(","):#number of left legs
            if len(s) == 1:
                leftLegA = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT']); 
                leftLegA.id = LegAppendage.A; self.legs.append(leftLegA)
            if len(s) == 2:
                leftLegA = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT']); 
                leftLegA.id = LegAppendage.A; self.legs.append(leftLegA)
                leftLegB = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['LEFT']); 
                leftLegB.id = LegAppendage.B; self.legs.append(leftLegB)                
            if len(s) == 3:
                leftLegA = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT']); 
                leftLegA.id = LegAppendage.A; self.legs.append(leftLegA)
                leftLegB = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['LEFT']); 
                leftLegB.id = LegAppendage.B; self.legs.append(leftLegB)                 
                leftLegC = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, leftLegB.obj_body, leftLegB.legWd, self.ori['LEFT']); 
                leftLegC.id = LegAppendage.C; self.legs.append(leftLegC)                                 
        for s in rt.split(","):#number of right legs
            if len(s) == 1:
                rightLegA = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT']); 
                rightLegA.id = LegAppendage.A; self.legs.append(rightLegA)
            if len(s) == 2:
                rightLegA = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT']); 
                rightLegA.id = LegAppendage.A; self.legs.append(rightLegA) 
                rightLegB = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, rightLegA.obj_body, rightLegA.legWd, self.ori['RIGHT']); 
                rightLegB.id = LegAppendage.B; self.legs.append(rightLegB)                        
            if len(s) == 3:
                rightLegA = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT']); 
                rightLegA.id = LegAppendage.A; self.legs.append(rightLegA)
                rightLegB = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, rightLegA.obj_body, rightLegA.legWd, self.ori['RIGHT']); 
                rightLegB.id = LegAppendage.B; self.legs.append(rightLegB)                 
                rightLegC = LearningRobotLegPart(self.world.space, self.ownBodyShapeFilter, rightLegB.obj_body, rightLegB.legWd, self.ori['RIGHT']); 
                rightLegC.id = LegAppendage.C; self.legs.append(rightLegC)                                 
        id = 0
        for leg in self.legs:
            leg.id = id; id += 1
            
    def stopMotion(self):
        for leg in self.legs: leg.motor.rate = 0
                        
    def setFocusRobotColor(self): self.chassis_shape.color = (190, 0, 0)
    def setNormalRobotColor(self): self.chassis_shape.color = (170, 170, 170)
                
    def setImaginaryRobotColor(self):
        self.chassis_shape.color = (110, 110, 110)  
        for leg in self.legs: leg.leg_shape.color = (110, 110, 110)
        
    def makeRobotDynamic(self):
        self.obj_body.body_type = pymunk.Body.DYNAMIC
        self.obj_body.mass = self.chassisMass
        self.obj_body.moment = pymunk.moment_for_box(self.chassisMass, (self.chassisWd, self.chassisHt))            
        
    def delete(self):
        self.world.space.remove(self.chassis_shape); self.world.space.remove(self.obj_body)
        for legPart in self.legs: legPart.delete()
        self.legs[:] = [] #clear the list
        
    def getPosition(self): return self.obj_body.position
    def getBodyAngle(self): return round(math.degrees(self.obj_body.angle)%360)
    def getNodeUID(self, legRef):        
        return legRef.__getNodeUID__(self.getQuadrantForLeg(legRef))    
    
    def updatePosition(self, offsetXY):
        self.obj_body.position += offsetXY 
        self.obj_body.startPosition += offsetXY
        for leg in self.legs: leg.updatePosition(offsetXY)

    def getLegQuadrants(self):
        quads = []
        for i in range(0, len(self.legs), 1):
            quads.append(self.getQuadrantForLeg(i))
        return quads
    
    def getQuadrantForLeg(self, l): 
        if isinstance(l, int): return self.__getQuadrant__(self.legs[i].getTip())
        else: return self.__getQuadrant__(l.getTip())
    
    def __getQuadrant__(self, pt):#pt should be Vec2d or tuple
        translateOffset = -Vec2d(self.obj_body.position)
        tPt = pt + translateOffset #translate
        theta = -self.getBodyAngle() #for clockwise rotation
        tPt[0] = tPt[0] * math.cos(theta) - tPt[1] * math.sin(theta) #rotate
        tPt[1] = tPt[0] * math.sin(theta) + tPt[1] * math.cos(theta) #rotate
        return (math.floor(tPt[0]/self.quadrantAccuracy), math.floor(tPt[1]/self.quadrantAccuracy))
#                 #---find which angle quadrant and distance radius the leg tip falls wrt the rotated body position
#                 bodyAng = round(math.degrees(self.body.obj_body.angle)%360)
#                 legTip = self.body.legs[i].getTip();  chCen = self.body.obj_body.position
#                 dist = abs(math.sqrt((legTip[0]-chCen[0])**2 + (legTip[1]-chCen[1])**2)) / self.distanceAccuracy
#                 ang = round((math.degrees(math.atan2((legTip[1]-chCen[1]), (legTip[0]-chCen[0])))-bodyAng)/self.angleAccuracy)
#                 self.body.actionNetwork.addAction([i, dist, ang, self.body.legs[i].motor.rate]) #HASH: [legID, distance, angle, motorRate]
#         

    
                                    
#     def setExperience(self, expe):       
#         for leg in self.legs: leg.experience[:] = []
#         while len(expe) > 0: 
#             for leg in self.legs: leg.experience.append(expe.pop(0))
#              
#     def reinitializeWithRandomValues(self, seqLen):       
#         for leg in self.legs:
#             leg.experience[:] = []
#             for i in range(0, seqLen, 1):
#                 thisRate = random.choice(leg.motor.legRateRange)
#                 leg.experience.append(thisRate)                                    
#     
#     def getValues(self):
#         ex = []
#         for leg in self.legs: ex.extend(leg.experience)
#         return ex
#     
#     def setValuesWithClamping(self, ex, seqLen):
#         j = 0
#         for leg in self.legs:
#             leg.experience[:] = []
#             for i in range(0, seqLen, 1): 
#                 v = ex[j]
#                 if v < leg.motor.legRateRange[0] or v > leg.motor.legRateRange[-1]: v = random.choice(leg.motor.legRateRange)
#                 leg.experience.append(v)
#                 j += 1
#     
#     def setMotorRateForSequence(self, seqId):
#         for leg in self.legs: leg.motor.rate = leg.experience[seqId]   
#     def getUniqueBodyAngles(self):
#         ang = [self.roundToNearest(self.getBodyAngle())]
#         for leg in self.legs: ang.append(self.roundToNearest(leg.getLegAngle()))
#         return ang
#     
#     def setBodyPositionAndAngles(self, pos, angles, offset):
#         p = pos.pop(0)
#         self.obj_body.position = Vec2d(p[0], p[1]) + offset
#         self.obj_body.startPosition = Vec2d(p[0], p[1]) + offset
#         self.obj_body.angle = math.radians(angles.pop(0))
#         for leg in self.legs:
#             p = pos.pop(0)
#             leg.obj_body.position = Vec2d(p[0], p[1]) + offset
#             leg.obj_body.angle = math.radians(angles.pop(0))
#         return pos    
#     
#     def getPositions(self):
#         pos = [Vec2d(self.obj_body.position)]
#         for leg in self.legs: pos.append(Vec2d(leg.obj_body.position))
#         return pos
#         
#     def roundToNearest(self, num):
#         roundOffPrecision = 5
#         rem = num % roundOffPrecision
#         if rem < roundOffPrecision / 2: num = int(num/roundOffPrecision) * roundOffPrecision
#         else: num = int((num+roundOffPrecision) / roundOffPrecision) * roundOffPrecision
#         return num
     

         
