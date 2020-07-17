# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

# Note: WalkingRobot is for ImaginationTwin and LearningRobot is for ActualImagination

import math
import pymunk
import random
import hashlib
import numpy as np
#import networkx as nx
from ComputationalIntelligence import Constants
from collections import deque
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
    
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class LiveGraph:#to display sensory info of robot
    def __init__(self, numSenses):
        self.val = []; self.maxLen = 100
        for i in range(0, numSenses, 1):
            self.val.append(deque())
        self.fig = plt.figure()
        self.fig.patch.set_facecolor('white')
    def showPlot(self, sensed, names):
        i = 0
        plt.clf(); styles = ['-r','-b','-g','-c','-m','-y','-k','--r','--b','--g','--c','--m','--y','--k','-.r','-.b','-.g','-.c','-.m','-.y','-.k',':r',':b',':g',':c',':m',':y',':k']        
        for v in sensed:
            self.val[i].append(v) #add new value
            if len(self.val[i]) > self.maxLen: self.val[i].popleft() #remove oldest
            i += 1
        for i in range(0, len(self.val), 1):
            plt.plot(np.linspace(1, len(self.val[i]), len(self.val[i])), self.val[i], styles[i], label=names[i])
            print(str(self.val[i][-1])+'  '+names[i])
        plt.title('Senses'); plt.xlabel('Time', fontsize=12);plt.ylabel('impulses detected', fontsize=12)
        self.fig.canvas.toolbar.pack_forget() #remove bottom bar  
        plt.pause(0.0001); plt.legend(loc='best'); plt.show(block=False) 
        
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

class LearningRobot:#Used in the actual imagination world
    def __init__(self, world, chassisCenterPoint, legCode, showImagination=True): #def __init__(self, world, chassisCenterPoint, legCode, actionNet, showImagination=True):
        self.world = world
        self.legsCode = legCode
        #self.actions = actionNet
        self.showImagination = showImagination
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
        self.sensors.append(TactileSensor(world, self, showImagination)) #body touching objects
        for leg in self.legs: self.sensors.append(TactileSensor(world, leg, showImagination)) #leg parts touching objects
        self.sensors.append(BodyAngleSensor(world, self, showImagination))
        for leg in self.legs: self.sensors.append(LegTipQuadrantSensor(world, self, leg, showImagination))
        self.sensors.append(DisplacementSensor((self.world.worldWidth, self.world.worldHeight), self.getPosition()))  
        #---plot to show sensor data
        if self.showImagination: self.dataDisplay = LiveGraph(len(self.sensors))            
        
    def run(self):       
        #---sense
        sensed = []; names = []
        for i in range(0, len(self.sensors), 1): 
            vals = self.sensors[i].get() #get sensor info for this time instant
            name = self.sensors[i].__class__.__name__
            vals = self.getGraphReadyValues(vals, name)
            names.append(str(i+1) + name); sensed.append(vals) 
        #self.dataDisplay.showPlot(sensed, names)
        #---run behaviour
        if self.state != None:
            self.state.run()
    def getGraphReadyValues(self, vals, name):    
        if name == 'TactileSensor':
            if len(vals) > 1: vals = 1
            else: vals = 0
        if name == 'DisplacementSensor':
            if len(vals) > 0:
                vals = math.fabs((vals[0] + vals[1])/2)
        if name == 'LegTipQuadrantSensor':
            if len(vals) > 1: vals = 1
            else: vals = 0
        return vals
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
        for s in self.sensors: s.delete()
        
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
        if isinstance(l, int): return self.__getQuadrant__(self.legs[l].getTip())
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

