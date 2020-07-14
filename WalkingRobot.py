# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import math
import pymunk
import random
import numpy as np
#import networkx as nx
from ComputationalIntelligence import Constants
import matplotlib.pyplot as plt
from pymunk import Vec2d, shapes
from pygame import _numpysurfarray
from collections import defaultdict
from matplotlib.animation import FuncAnimation

class Directions:
    def __init__(self):
        self.dirn = {'UP':1, 'DOWN':2, 'LEFT':3, 'RIGHT':4, #'TOPRIGHT':5, 'TOPLEFT':6, 'BOTTOMRIGHT':7, 'BOTTOMLEFT':8
                     }
    def getDirn(self): 
        return self.dirn        

# class Brain:
#     def __init__(self, pos):
#         d = Directions()
#         self.ori = d.getDirn()
#         self.direction = self.ori['RIGHT']
#         #self.stuckThresh = 2 #TODO: Currently pixels. Make this a proportion of the body length
#         #self.maxStuck = 2 #max iterations before deciding to get unstuck by going in different direction
#         #self.stuck = 0 #counter 
#         #self.maxRoaming = 5 #max iterations to roam in non-main direction
#         #self.roaming = 0 #counter
#         self.prevPos = pos
#         self.const = Constants()
#         self.decimalPrecision = 2
#         
#     def movementThinking(self, currPos):
#         if self.roaming > 0:
#             self.roaming -= 1
#             if self.roaming == 0: self.setToMainDirection()#stop roaming
#         dist = abs(math.sqrt((currPos[0]-self.prevPos[0])**2 + (currPos[1]-self.prevPos[1])**2))
#         if dist <= self.stuckThresh:   
#             self.stuck += 1
#             if self.stuck == self.maxStuck: self.moveInDifferentDirection()
#         else: 
#             self.prevPos = currPos
#             self.stuck = 0
#             
#     def getFitness(self, prevPos, currPos):
#         if self.robotDirection(prevPos, currPos) == self.direction: 
#             return round(abs(math.sqrt((currPos[0]-self.prevPos[0])**2 + (currPos[1]-self.prevPos[1])**2)), self.decimalPrecision)
#         else: 
#             return self.const.NOTFIT 
#         
#     def robotDirection(self, prevPos, currPos): 
#         ang = round(math.degrees(math.atan2((currPos[1]-prevPos[1]), (currPos[0]-prevPos[0])))) % 360
#         if ang > 270 or ang <= 91: direc = self.ori['RIGHT']
#         if ang > 91 and ang <= 135: direc = self.ori['UP']
#         if ang > 135 and ang <= 225: direc = self.ori['LEFT']
#         if ang > 225 and ang <= 270: direc = self.ori['DOWN']  
#         return direc
#     
#     def moveInDifferentDirection(self):
#         d = self.ori[list(self.ori)[random.randint(0,len(self.ori)-1)]]#random direction
#         while d == self.direction:
#             d = self.ori[list(self.ori)[random.randint(0,len(self.ori)-1)]]#random direction
#         self.direction = d
#         self.roaming = self.maxRoaming
#         
#     def setToMainDirection(self): self.direction = self.ori['RIGHT']
#     def getDirection(self): return self.direction

# class ActionNetwork:
#     def __init__(self, execLen, legs):
#         self.actionFile = 'actionNetwork_'+str(execLen)+'_'+legs+'.gpickle'
#         self.graph = None
#         self.fig = plt.figure()
#         self.fig.patch.set_facecolor('black')        
#         plt.rcParams['axes.facecolor'] = 'black'
#         plt.rcParams['patch.facecolor'] = 'black'
#         self.fig.canvas.toolbar.pack_forget()#remove bottom bar        
#         self.__loadNetwork__()
#     
#     def addNode(self, node): self.graph.add_node(tuple(node))        
#     def addEdge(self, node1, node2, wt, expe, fit, dirn): self.graph.add_edge(tuple(node1), tuple(node2), weight=wt, experience=tuple(expe), fitness=fit, direction=dirn)    
#     def getEdge(self, node1, node2): return self.graph.get_edge_data(tuple(node1), tuple(node2))
#     
#     def getSuccessorNodes(self, currNode):#edges going out of currNode
#         if self.graph.out_degree[tuple(currNode)] == 0: return None 
#         else: return self.graph.successors(tuple(currNode))
    
#     def displayNetwork(self):    
#         plt.clf(); 
#         #posType = nx.planar_layout(self.graph)        
#         #posType = nx.circular_layout(self.graph)
#         #posType = nx.kamada_kawai_layout(self.graph)
#         #posType = nx.random_layout(self.graph)
#         posType = nx.spring_layout(self.graph)
#         #posType = nx.fruchterman_reingold_layout(self.graph)
#         #nx.draw_networkx(self.graph, pos=posType, arrows=True, with_labels=False, node_size=20, edge_color='green', arrowsize=1, arrowstyle='fancy')
#         nx.draw_networkx(self.graph, pos=posType, arrows=True, with_labels=False, node_size=20, edge_color='green', node_color='green', alpha=0.5, arrowsize=5)
#         #nx.draw_circular(self.graph)
#         #nx.draw(self.graph, with_labels=False, font_weight='bold')
#         plt.pause(0.001)
#         plt.show(block=False)  
#         print('Num nodes in action network: '+str(nx.number_of_nodes(self.graph)))     
        
#     def saveNetwork(self):
#         nx.write_gpickle(self.graph, self.actionFile)
#         print('Saved network to '+self.actionFile)
#     
#     def __loadNetwork__(self):
#         try:
#             self.graph = nx.read_gpickle(self.actionFile)
#         except:
#             print('No '+self.actionFile+' found. Creating new action network.')
#             self.graph = nx.MultiDiGraph()
#         if not nx.is_empty(self.graph):
#             #self.displayNetwork()
#             self.actionNetworkProperties()
#     
#     def actionNetworkProperties(self):
#         print('Action network properties:')
#         print('Num nodes: '+str(nx.number_of_nodes(self.graph)))
#         print('Num edges: '+str(nx.number_of_edges(self.graph)))
#         print('Density: '+str(nx.density(self.graph)))
#         print('Num self loops: '+str(nx.number_of_selfloops(self.graph)))

class LegPart:#This is one leg part. Could be part A that's connected to the chassis or part B that's connected to part A
    def __init__(self, pymunkSpace, ownBodyShapeFilter, prevBody, prevBodyWidth, leftOrRight):
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
        self.leg_shape.friction = 10.0 
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
        
    def updatePosition(self, offsetXY): self.obj_body.position = self.obj_body.position + offsetXY         
    def getLegAngle(self): return round(math.degrees(self.obj_body.angle) % 360)        

class RobotBody:
    def __init__(self, pymunkSpace, chassisCenterPoint, legCode):
        self.legsCode = legCode
        self.ownBodyShapeFilter = pymunk.ShapeFilter(group=1) #to prevent collisions between robot body parts
        d = Directions()  #leg at left or right of chassis
        self.ori = d.getDirn()
        self.chassisWd = 30 #chassis width 
        self.chassisHt = 20 #chassis height
        self.chassisMass = 10
        self.obj_body = None #chassis body
        self.chassis_shape = None #chassis shape 
        self.legs = []
        self.limbMotorRates = []
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
        self.chassis_shape.friction = 10.0
        self.space.add(self.chassis_shape, self.obj_body) 
        self.createLegs()
        self.makeRobotDynamic()
    
    def createLegs(self):
        s = self.legsCode.split("#")
        lt = s[0]; rt = s[1]; rt = rt[::-1]#reverse string rt
        for s in lt.split(","):#number of left legs
            if len(s) == 1:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT']); self.legs.append(leftLegA)
            if len(s) == 2:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT']); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['LEFT']); self.legs.append(leftLegB)                
            if len(s) == 3:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['LEFT']); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['LEFT']); self.legs.append(leftLegB)                 
                leftLegC = LegPart(self.space, self.ownBodyShapeFilter, leftLegB.obj_body, leftLegB.legWd, self.ori['LEFT']); self.legs.append(leftLegC)                                 
        for s in rt.split(","):#number of right legs
            if len(s) == 1:
                rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT']); self.legs.append(rightLegA)
            if len(s) == 2:
                rightLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT']); self.legs.append(rightLegA) 
                rightLegB = LegPart(self.space, self.ownBodyShapeFilter, rightLegA.obj_body, rightLegA.legWd, self.ori['RIGHT']); self.legs.append(rightLegB)                        
            if len(s) == 3:
                leftLegA = LegPart(self.space, self.ownBodyShapeFilter, self.obj_body, self.chassisWd, self.ori['RIGHT']); self.legs.append(leftLegA)
                leftLegB = LegPart(self.space, self.ownBodyShapeFilter, leftLegA.obj_body, leftLegA.legWd, self.ori['RIGHT']); self.legs.append(leftLegB)                 
                leftLegC = LegPart(self.space, self.ownBodyShapeFilter, leftLegB.obj_body, leftLegB.legWd, self.ori['RIGHT']); self.legs.append(leftLegC)                                 
        #---limbMotorRates will store rates for DE to use, but the starting and stopping of motors can be done independent of the values in limbMotorRates
        self.setRandomLegMotorRates()
        self.stopMotion()
                        
    def getFitness(self, prevPos, currPos):
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
    
#     def setExperience(self, expe):       
#         for leg in self.legs: leg.experience[:] = []
#         while len(expe) > 0: 
#             for leg in self.legs: leg.experience.append(expe.pop(0))
# 
#     def setExperienceWithClamping(self, ex, seqLen):
#         j = 0
#         for leg in self.legs:
#             leg.experience[:] = []
#             for _ in range(0, seqLen, 1): 
#                 v = ex[j]
#                 if v < leg.motor.legRateRange[0] or v > leg.motor.legRateRange[-1]: 
#                     v = random.choice(leg.motor.legRateRange)
#                 leg.experience.append(v)
#                 j += 1
    
#     def reinitializeExperienceWithRandomValues(self, seqLen):       
#         for leg in self.legs:
#             leg.experience[:] = []
#             for _ in range(0, seqLen, 1):
#                 thisRate = random.choice(leg.motor.legRateRange)
#                 leg.experience.append(thisRate)                                    
#     
#     def getMotorRatesExperience(self):
#         ex = []
#         for leg in self.legs: 
#             ex.extend(leg.experience)
#         return ex
#     
#     def getEmptyMotorRatesExperience(self):
#         ex = []
#         for leg in self.legs: 
#             ex.extend([0] * len(leg.experience))
#         return ex    
#     
#     def setMotorRateForSequence(self, seqId):
#         for leg in self.legs: 
#             leg.motor.rate = leg.experience[seqId]
    
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
    
    def setFocusRobotColor(self): self.chassis_shape.color = (190, 0, 0)
    def setNormalRobotColor(self): self.chassis_shape.color = (170, 170, 170)
                
    def setImaginaryRobotColor(self):
        self.chassis_shape.color = (110, 110, 110)  
        for leg in self.legs: 
            leg.leg_shape.color = (110, 110, 110)
        
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
        
    def updatePosition(self, offsetXY):
        self.obj_body.position += offsetXY 
        self.obj_body.startPosition += offsetXY
        for leg in self.legs: 
            leg.updatePosition(offsetXY)
    
#     def getFullBodyStatesAndMotorRates(self):
#         bodyStates = []; motorRates = []
#         bodyStates.append(self.obj_body.angle)
#         bodyStates.append(self.obj_body.position)
#         for leg in self.legs:
#             val = leg.getLegStatesAndMotorRates()
#             bodyStates.append(val[0])
#             motorRates.append(val[1])
#         return (bodyStates, motorRates)
    
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
        
