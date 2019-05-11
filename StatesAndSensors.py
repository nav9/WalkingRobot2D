# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

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

class BrainStateRandom:#used by Heaven World
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
                if leg.oldNode != newNode:
                    self.robo.actions.addEdge(leg.oldNode, newNode, 1, leg.currentRate, leg.currentDura)
                self.setRandomMovementState(leg)

class BrainStateResearch:
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
                    #self.robo.actions.addEdge(leg.oldNode, newNode, 1, leg.currentRate, leg.currentDura)
                self.setRandomMovementState(leg)

#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class DisplacementSensor:
    def __init__(self, worldExtent, currPos):
        self.worldExtent = worldExtent #world extent helps in normalization
        self.prevAngle = currPos; 
        self.displacement = None; self.decimalAccuracy = 4
    def set(self, currPos): #value to normalize
        self.displacement[0] = (self.prevAngle[0] - currPos[0]) / self.worldExtent[0]
        self.displacement[1] = (self.prevAngle[1] - currPos[1]) / self.worldExtent[1]
        self.prevAngle = currPos
    def get(self): return Vec2d(self.displacement)

class AngleSensor:
    def __init__(self, currAng):
        self.prevAngle = currAng; 
        self.angleChange = None; self.decimalAccuracy = 2
    def set(self, currAngle): #value to normalize
        self.angleChange = (self.prevAngle - currAngle) / 360.0
        self.prevAngle = currAngle
    def get(self): return self.angleChange
    
class TactileSensor:#TODO: delete function to remove objects added to space
    def __init__(self, world, robo):
        self.world = world
        self.robo = robo
        self.points = set()
    def get(self):
        self.robo.chassis_body.each_arbiter(self.contactInfo)
        for p in self.points:
            x = p[0]; y = p[1]; sz = 1; mass = 1; moment = 0
            if self.world.sensedObjects[x][y] == 0: #point is not in imagination
                self.world.sensedObjects[x][y] = 1
                ob_body = pymunk.Body(mass, moment)
                ob_body.body_type = pymunk.Body.KINEMATIC
                ob_body.position = Vec2d(x,y)
                ob_shape = pymunk.Poly.create_box(ob_body, (sz, sz))
                self.world.space.add(ob_body, ob_shape)

    def contactInfo(self, arbiter):    
#         print(arbiter.shapes)#gives the type of objects that are colliding [chassis,line seg]
#         print(arbiter.contact_point_set.normal)#direction of contact
#         print(arbiter.contact_point_set.points[0].distance)#distance is the penetration distance of the two shapes. Overlapping means it will be negative. This value is calculated as dot(point2 - point1), normal) and is ignored when you set the Arbiter.contact_point_set.
#         print(arbiter.total_impulse)#Returns the impulse that was applied this step to resolve the collision Vec2d(xImpulse, yImpulse)
#         print(arbiter.contact_point_set.points[0].point_a)#point_a and point_b are the contact position on the surface of each shape.
#         print(arbiter.contact_point_set.points[0].point_b)
        self.points = set()
        for p in arbiter.contact_point_set.points:
            self.points.add((round(p.point_b[0]), round(p.point_b[1])))
    
    