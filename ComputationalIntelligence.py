# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

from WalkingRobot import *
from random import uniform

class RunCode:
    STOP = 0
    CONTINUE = 1
    RESET = 2
    NEXTGEN = 3
    IMAGINE = 4
    EXPERIENCE = 5
    PAUSE_AND_SWITCH_TO_IMAGINATION = 6

class Constants:
    UNDETERMINED = -1
    NOTFIT = 0
    xID = 0
    yID = 1
    mainRobotID = 0

# IMPORTANT NOTE: A motor rate that produced a certain kind of motion in one execution may not always
# produce the same kind of motion again, due to the unreliability of the friction, joint movements and
# the physics in general. Therefore, the fitness values will vary unpredictably. This is normal for 
# the PyMunk physics environment, so don't be surprised when you see weird motion.
        
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class RandomBest:#Use randomness instead of a CI algorithm  
    def __init__(self, roboList):
        self.robots = roboList
        self.infoString = ""
        self.const = Constants()
        self.motorRatesOfFittest = []
        self.reinitialize()
    def reinitialize(self):
        self.fittestRobot = self.const.UNDETERMINED
    def run(self):        
        currBestFit = 0 if self.fittestRobot == self.const.UNDETERMINED else self.robots[self.fittestRobot].getFitness()
        #---go through all robots to find if there's a new fittest one
        for i in range(0, len(self.robots)):            
            fit = self.robots[i].getFitness()
            if fit > currBestFit:#if greater than zero
                self.fittestRobot = i #if there was a previous generation's fittest, that won't get replaced, but everything gets reset when generations are reinitialized
                self.motorRatesOfFittest = self.robots[i].getLegMotorRates()
                currBestFit = fit
        for i in range(0, len(self.robots)):
            if self.fittestRobot == i:            
                continue #don't change the motor rates for this
            else:
                self.robots[i].setRandomLegMotorRates()
        self.infoString = ",  Random,  fittest robot: "+ ('-' if self.fittestRobot == self.const.UNDETERMINED else str(self.fittestRobot)) +",  fitness: "+str(self.robots[self.fittestRobot].getFitness())+",  motor rates: "+str([round(x,1) for x in self.motorRatesOfFittest])
    def getInfoString(self):
        return self.infoString
    def getFittestRobot(self):
        return self.fittestRobot
    
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

# Note: At least 4 robots are required for Differential Evolution to work
class SimpleDE:#Use randomness instead of a CI algorithm  
    def __init__(self, roboList, maxGen):
        self.robots = roboList
        self.maxGen = maxGen
        self.infoString = ""
        self.const = Constants()
        self.motorRatesOfFittest = []
        #---Differential Evolution parameters
        self.masterBeta = 2.0 #beta is real number belongs to [0 -> 2]
        self.vBetaReductionFactor = 1/self.maxGen
        self.crProba = 0.3 #crossover probability range [0 -> 1]  
        self.vBeta = self.masterBeta  
        minRate, maxRate = self.robots[0].getMinMaxLegRates()
        self.minLegMotorRate = minRate
        self.maxLegMotorRate = maxRate
        self.reinitialize()
    def reinitialize(self):
        self.fittestRobot = self.const.UNDETERMINED
        self.vBeta = self.masterBeta
    def run(self):        
        currBestFit = 0 if self.fittestRobot == self.const.UNDETERMINED else self.robots[self.fittestRobot].getFitness()
        #---go through all robots to find if there's a new fittest one
        robotIndexes = []
        for i in range(0, len(self.robots)):
            robotIndexes.append(i)            
            fit = self.robots[i].getFitness()
            if fit > currBestFit:#if greater than zero
                self.fittestRobot = i #if there was a previous generation's fittest, that won't get replaced, but everything gets reset when generations are reinitialized
                self.motorRatesOfFittest = self.robots[i].getLegMotorRates()
                currBestFit = fit        
        #---mutations for each robot
        for i in range(0, len(self.robots)):
            if self.fittestRobot == i: continue #don't change the motor rates for fittest
            else:
                tempIndexes = robotIndexes[:] #copying values instead of references
                tempIndexes.remove(i) #remove influence of current robot
                r1 = random.choice(tempIndexes); tempIndexes.remove(r1)
                r2 = random.choice(tempIndexes); tempIndexes.remove(r2)
                r3 = random.choice(tempIndexes); tempIndexes.remove(r3)
                r1 = self.robots[r1].getLegMotorRates()
                r2 = self.robots[r2].getLegMotorRates()
                r3 = self.robots[r3].getLegMotorRates()
                r = self.robots[i].getLegMotorRates() #current robot being iterated
                mutantMotorRates = []; hadMutated = False
                for ii in range(0, len(r1)):
                    mutatedRateForOneMotor = r[ii] #the original motor rate for the robot being considered
                    #print('mutated rate for one:', mutatedRateForOneMotor)
                    if uniform(0,1) <= self.crProba: #if crossover required
                        #print('mutations value:',r1[ii],self.vBeta,r2[ii], r3[ii])
                        mutatedRateForOneMotor = r1[ii] + round(self.vBeta * (r2[ii] - r3[ii])) #mutate
                    if mutatedRateForOneMotor < self.minLegMotorRate or mutatedRateForOneMotor > self.maxLegMotorRate: #bounds check
                        mutantMotorRates.append(r[ii]) #no change to motor rate instead of clamping or re-initializing
                    else:
                        mutantMotorRates.append(mutatedRateForOneMotor); hadMutated = True
                if hadMutated:
                    self.robots[i].setLegMotorRates(mutantMotorRates)
        #---beta is reduced to encourage exploitation and reduce exploration
        if self.vBeta > self.vBetaReductionFactor: 
            self.vBeta = self.vBeta - self.vBetaReductionFactor    

        self.infoString = ",  DE,  fittest robot: "+ ('-' if self.fittestRobot == self.const.UNDETERMINED else str(self.fittestRobot)) +",  fitness: "+str(self.robots[self.fittestRobot].getFitness())+",  motor rates: "+str([round(x,1) for x in self.motorRatesOfFittest])
        
    def getInfoString(self):
        return self.infoString
    def getFittestRobot(self):
        return self.fittestRobot
    
    
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#Particle Swarm Optimization
class SimplePSO:#Use randomness instead of a CI algorithm  
    def __init__(self, roboList):
        self.robots = roboList
        self.infoString = ""
        self.const = Constants()        
        self.numLegs = len(self.robots[0].legs)
        #---PSO parameters
        self.c1 = 1 #cognitive coefficient
        self.c2 = 2 #social coefficient        
        self.reinitialize() 
        minRate, maxRate = self.robots[0].getMinMaxLegRates()
        self.minLegMotorRate = minRate
        self.maxLegMotorRate = maxRate        
    def reinitialize(self):
        self.fittestRobot = self.const.UNDETERMINED
        self.motorRatesOfFittest = [0] * len(self.robots) #creates [0,0,...,0] #global best rate
        self.velocitiesPSO = self.__initializeLocalZeroArrays__(len(self.robots), self.numLegs)
        self.personalBestRates = self.__initializeLocalZeroArrays__(len(self.robots), self.numLegs)
        self.pastFitnesses = [0] * len(self.robots) #creates [0,0,...,0]         
    def run(self):        
        currBestFit = 0 if self.fittestRobot == self.const.UNDETERMINED else self.robots[self.fittestRobot].getFitness()
        #---go through all robots to find if there's a new fittest one
        robotIndexes = []
        for i in range(len(self.robots)):
            robotIndexes.append(i)            
            fit = self.robots[i].getFitness()
            if fit > currBestFit:#if greater than zero
                self.fittestRobot = i #if there was a previous generation's fittest, that won't get replaced, but everything gets reset when generations are reinitialized
                self.motorRatesOfFittest = self.robots[i].getLegMotorRates()
                currBestFit = fit        
        #---mutations for each robot
        for i in range(len(self.robots)):
            if self.fittestRobot == i: continue #don't change the motor rates for fittest
            else:
                currentRates = self.robots[i].getLegMotorRates()
                currentFitness = self.robots[i].getFitness()
                if currentFitness > self.pastFitnesses[i]:
                    self.pastFitnesses[i] = currentFitness
                    self.personalBestRates[i] = currentRates
                for ii in range(self.numLegs):
                    cognitive = self.c1 * random.random() * (self.personalBestRates[i][ii] - currentRates[ii])
                    social = self.c2 * random.random() * (self.motorRatesOfFittest[ii] - currentRates[ii])                    
                    self.velocitiesPSO[i][ii] = self.velocitiesPSO[i][ii] + cognitive + social
                    #---velocity clamping
                    if self.velocitiesPSO[i][ii] > self.maxLegMotorRate / 2: self.velocitiesPSO[i][ii] = self.maxLegMotorRate / 2
                    if self.velocitiesPSO[i][ii] < self.minLegMotorRate / 2: self.velocitiesPSO[i][ii] = self.minLegMotorRate / 2
                    #---rate update 
                    currentRates[ii] = currentRates[ii] + self.velocitiesPSO[i][ii]
                    #---rate clamping
                    if currentRates[ii] > self.maxLegMotorRate: currentRates[ii] = self.maxLegMotorRate
                    if currentRates[ii] < self.minLegMotorRate: currentRates[ii] = self.minLegMotorRate
                self.robots[i].setLegMotorRates(currentRates)

        fittestRobotString = "fittest robot: "+ ('-' if self.fittestRobot == self.const.UNDETERMINED else str(self.fittestRobot))
        fitnessString = ",  fitness: "+str(self.robots[self.fittestRobot].getFitness())
        motorRatesString = ",  motor rates: "+str([round(x,1) for x in self.motorRatesOfFittest])
        self.infoString = ",  PSO,  " + fittestRobotString + fitnessString + motorRatesString
        
    def getInfoString(self):
        return self.infoString
    def getFittestRobot(self):
        return self.fittestRobot
    def __initializeLocalZeroArrays__(self, numRobots, numLegs):
        a = []
        for _ in range(numRobots):
            a.append([0] * numLegs) #[0,0,0,0] 
        return a
    
    