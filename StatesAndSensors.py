# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import math
import time
import pymunk
import random
from pymunk import Vec2d

class RunState:
    DONE = 0
    RUNNING = 1

class RandomMovement:    
    def __init__(self, leg, motorRate, duration):
        self.runState = RunState.RUNNING
        self.leg = leg
        self.leg.motor.rate = motorRate
        self.startTime = time.time()
        self.duration = duration
    def run(self):
        if self.runState == RunState.RUNNING:
            if time.time() - self.startTime > self.duration:
                self.leg.motor.rate = 0
                self.runState = RunState.DONE
    
class Retract:
    pass

class Freeze:
    def __init__(self, leg):
        self.runState = RunState.RUNNING
        self.leg = leg
    def run(self):
        if self.runState == RunState.RUNNING:
            self.leg.motor.rate = 0
            self.runState = RunState.DONE

class BrainState_RandomMovement:#used by Heaven World
    def __init__(self, robo):
        self.runState = RunState.RUNNING
        self.bodyPart = robo
        for leg in self.bodyPart.legs: 
            self.setRandomMovementState(leg)
    def setRandomMovementState(self, leg):        
        leg.currentRate = random.choice(leg.motor.legRateRange)
        leg.currentDura = random.choice(leg.motor.legMovtDurationRange)
        leg.oldNode = self.bodyPart.getNodeUID(leg)#node before leg starts moving     
        leg.state = RandomMovement(leg, leg.currentRate, leg.currentDura)                    
    def run(self):
        for leg in self.bodyPart.legs:
            leg.state.run()
            if leg.state.runState == RunState.DONE:
                newNode = self.bodyPart.getNodeUID(leg)#node after leg has moved for duration at rate
                if leg.oldNode != newNode:
                    self.bodyPart.actions.addEdge(leg.oldNode, newNode, 1, leg.currentRate, leg.currentDura)
                self.setRandomMovementState(leg)

class BrainState_WhatHappensIfITryThis:
    def __init__(self, robo):
        self.runState = RunState.RUNNING
        self.robo = robo
        for leg in self.robo.legs: 
            self.setRandomMovementState(leg)
    def setRandomMovementState(self, leg):        
        leg.currentRate = random.choice(leg.motor.legRateRange)
        leg.currentDura = random.choice(leg.motor.legMovtDurationRange)
        leg.oldNode = self.robo.getNodeUID(leg)#node before leg starts moving     
        leg.state = RandomMovement(leg, leg.currentRate, leg.currentDura)                    
    def run(self):
        for leg in self.robo.legs:
            leg.state.run()
            if leg.state.runState == RunState.DONE:
                newNode = self.robo.getNodeUID(leg)#node after leg has moved for duration at rate
                #if leg.oldNode != newNode:
                    #self.bodyPart.actions.addEdge(leg.oldNode, newNode, 1, leg.currentRate, leg.currentDura)
                self.setRandomMovementState(leg)

#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class DisplacementSensor:
    def __init__(self, worldExtent, currPos):
        self.worldExtent = worldExtent #world extent helps in normalization
        self.prevPos = currPos; 
        self.displacement = None; self.decimalAccuracy = 4
    def set(self, currPos): #value to normalize
        self.displacement[0] = (self.prevPos[0] - currPos[0]) / self.worldExtent[0]
        self.displacement[1] = (self.prevPos[1] - currPos[1]) / self.worldExtent[1]
        self.prevPos = currPos
    def get(self): return Vec2d(self.displacement)
    def delete(self):
        pass

class BodyAngleSensor:
    def __init__(self, world, robo, showImagination):
        self.world = world; self.robo = robo; self.showImagination = showImagination
        self.prevAngle = robo.getBodyAngle(); 
        self.angleChange = None; self.decimalAccuracy = 2
        self.addedObjects = []
        if showImagination:
            sz = 1; mass = 1; moment = 0; pos = self.robo.getPosition()
            self.ob_body = pymunk.Body(mass, moment)
            self.ob_body.body_type = pymunk.Body.KINEMATIC
            self.ob_body.position = Vec2d(pos[0], pos[1] + self.world.imaginaryWorldYOffset)
            ob_shape = pymunk.Poly.create_box(self.ob_body, (self.robo.chassisWd, sz)); ob_shape.color = (110, 110, 110)
            self.world.space.add(self.ob_body, ob_shape); self.addedObjects.append(self.ob_body); self.addedObjects.append(ob_shape)        
    def get(self): 
        ang = self.robo.getBodyAngle()
        self.angleChange = (self.prevAngle - ang) / 360.0 #normalizing
        self.prevAngle = ang
        if self.showImagination:
            self.ob_body.angle = math.radians(ang); pos = self.robo.getPosition()
            self.ob_body.position = Vec2d(pos[0], pos[1]+self.world.imaginaryWorldYOffset)
        return round(self.angleChange)
    def delete(self):
        for ob in self.addedObjects: 
            self.world.space.remove(ob)
        self.addedObjects[:] = []
    
class LegTipQuadrantSensor:
    def __init__(self, world, robo, leg, showImagination):
        self.world = world; self.robo = robo; self.leg = leg; self.showImagination = showImagination
        self.addedObjects = []
        if self.showImagination:
            sz = 1; mass = 1; moment = 0; pos = leg.getTip()
            self.tip_body = pymunk.Body(mass, moment); self.tip_body.body_type = pymunk.Body.KINEMATIC
            self.tip_body.position = Vec2d(pos[0], pos[1]+self.world.imaginaryWorldYOffset)
            tip_shape = pymunk.Poly.create_box(self.tip_body, (sz, sz)); tip_shape.color = (255, 0, 0)  
            self.world.space.add(self.tip_body, tip_shape); self.addedObjects.append(self.tip_body); self.addedObjects.append(tip_shape)        
    def get(self):  
        if self.showImagination:       
            pos = self.leg.getTip()
            self.tip_body.position = Vec2d(pos[0], pos[1]+self.world.imaginaryWorldYOffset)
        return self.robo.getQuadrantForLeg(self.leg)
    def delete(self):
        for ob in self.addedObjects: 
            self.world.space.remove(ob)
        self.addedObjects[:] = []
        
class TactileSensor:
    def __init__(self, world, bodyPart, showImagination):
        self.world = world; self.bodyPart = bodyPart; self.showImagination = showImagination
        self.points = set()
        self.addedObjects = []
    def get(self):
        self.bodyPart.obj_body.each_arbiter(self.contactInfo)#invoke callback fn
        for p in self.points:
            x = p[0]; y = p[1]; sz = 1; mass = 1; moment = 0
            if self.world.sensedObjects[x][y] == 0: #point is not in imagination
                self.world.sensedObjects[x][y] = 1
                if self.showImagination:
                    ob_body = pymunk.Body(mass, moment)
                    ob_body.body_type = pymunk.Body.KINEMATIC
                    ob_body.position = Vec2d(x, self.world.imaginaryWorldYOffset+y)
                    ob_shape = pymunk.Poly.create_box(ob_body, (sz, sz))
                    self.world.space.add(ob_body, ob_shape); self.addedObjects.append(ob_body); self.addedObjects.append(ob_shape)
        return self.points
    def contactInfo(self, arbiter):#callback fn  
#         print(arbiter.shapes)#gives the type of objects that are colliding [chassis,line seg]
#         print(arbiter.contact_point_set.normal)#direction of contact
#         print(arbiter.contact_point_set.points[0].distance)#distance is the penetration distance of the two shapes. Overlapping means it will be negative. This value is calculated as dot(point2 - point1), normal) and is ignored when you set the Arbiter.contact_point_set.
#         print(arbiter.total_impulse)#Returns the impulse that was applied this step to resolve the collision Vec2d(xImpulse, yImpulse)
#         print(arbiter.contact_point_set.points[0].point_a)#point_a and point_b are the contact position on the surface of each shape.
#         print(arbiter.contact_point_set.points[0].point_b)
        self.points = set()
        for p in arbiter.contact_point_set.points:
            self.points.add((round(p.point_b[0]), round(p.point_b[1])))
    def delete(self):
        for ob in self.addedObjects:
            self.world.space.remove(ob)
        self.addedObjects[:] = []
        
    