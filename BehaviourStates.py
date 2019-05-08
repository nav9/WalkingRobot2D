# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import time
import random

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

class BrainStateRandom:
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

            
            