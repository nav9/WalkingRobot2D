# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import math
import random
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

#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

class RandomBest:#Use randomness instead of a CI algorithm  
    def __init__(self, roboList):
        self.robots = roboList
        self.infoString = ""
        self.const = Constants()
        self.fittestRobot = None
        self.motorRatesOfFittest = []
    def reinitialize(self):
        self.fittestRobot = self.const.UNDETERMINED
    def run(self):        
        currBestFit = 0
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
        self.infoString = " Fittest robot: "+ ('-' if self.fittestRobot<0 else str(self.fittestRobot)) +", motor rates: "+str([round(x,1) for x in self.motorRatesOfFittest])
    def getInfoString(self):
        return self.infoString
            
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

# Note: At least 4 robots are required for Differential Evolution to work
class SimpleDE:#Differential Evolution  
    def __init__(self, iRobo, realRob): 
        self.infoString = "" 
        
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------

# Note: At least 4 robots are required for Differential Evolution to work
class ImaginationDifferentialEvolution:  
    def __init__(self, iRobo, realRob): 
        self.infoString = ""  
        self.decimalPrecision = 2
        self.robots = iRobo 
        self.realRobo = realRob
        self.repeatSeq = 0; 
        self.maxSeqRepetitions = 3 #how many times to repeat the sequence of seqNum's  
        self.seqNum = 0 #ordinal of the sequence of movements of an experience
        self.temp = []
        self.const = Constants()
        self.resetDE()
        #---Differential Evolution parameters
        self.masterBeta = 2.0 #beta is real number belongs to [0 -> 2]
        self.vBeta = 0 #variable beta
        self.crProba = 0.3 #crossover probability range [0 -> 1]  
        self.vBeta = self.masterBeta    
        self.fit = []
        self.startNewGen()
        self.fittestRobotsMotorRates = [0]
        
    def resetDE(self):
        self.epochFittestRobot = self.const.UNDETERMINED 
        self.epochBestFitness = self.const.NOTFIT
        self.currentBestFitness = self.const.NOTFIT
        self.currentFittestRobot = self.const.UNDETERMINED

    def calcFitness(self):     
        self.fit[:] = []   
        for r in self.robots:
            self.fit.append(self.realRobo[self.const.mainRobotID].getFitness(r.obj_body.startPosition, r.obj_body.position)) #all imaginary robots start from a certain position, so fitness measures how far they travelled from that position to current body position. If they have not moved yet, fitness is zero

            #self.fit.append(self.realRobo[self.const.mainRobotID].brain.getFitness(r.obj_body.startPosition, r.obj_body.position))
            #self.fit.append(round(r.obj_body.position[0] - r.obj_body.startPosition[0], self.decimalPrecision))
#         #---assign zero fitness to any robot that turned upside down
#         for r in range(len(self.robots)):
#             ang = self.robots[r].getBodyAngle()            
#             if ang > 90 and ang < 270:
#                 self.unfitThisFullGen[r] = True
#                 self.fit[r] = self.NOTFIT  
        self.currentBestFitness = max(self.fit)
        if self.currentBestFitness == self.const.NOTFIT: 
            self.currentFittestRobot = self.const.UNDETERMINED
        else: 
            self.currentFittestRobot = self.fit.index(self.currentBestFitness)
    
    def differentialEvolution(self, seqLen):
        self.calcFitness(); oldSel = []; sel = []; i = 0; mutant = []
        for i in range(len(self.robots)):
            oldSel.append(i)
        
#         self.currentBestFitness = max(self.fit)
#         self.currentFittestRobot = self.fit.index(self.currentBestFitness)
        if self.currentBestFitness > self.epochBestFitness:
            self.epochBestFitness = self.currentBestFitness
            self.epochFittestRobot = self.fit.index(self.epochBestFitness)
            
        #---if none are fit, re-initialize all randomly coz there's no point doing DE
        if not False in self.unfitThisFullGen:#all values have to be true for this if to evaluate to true
            for r in self.robots:
                r.reinitializeExperienceWithRandomValues(seqLen)
                self.temp[:] = []                   
            return
        #---proceed with DE 
        for i in range(len(self.robots)):
            sel[:] = []
            for s in oldSel:
                sel.append(s)

            if i == self.currentFittestRobot:#don't mess with the fittest
                continue
            del sel[i]#remove current robot from list to be able to select another 3
            #---randomly choose three others for DE
            x1 = random.choice(sel); sel.remove(x1)
            x2 = random.choice(sel); sel.remove(x2)
            x3 = random.choice(sel); sel.remove(x3)
            mutant[:] = []
            x1 = self.robots[x1].getMotorRatesExperience()
            x2 = self.robots[x2].getMotorRatesExperience()
            x3 = self.robots[x3].getMotorRatesExperience()
            curr = self.robots[i].getMotorRatesExperience()   
            for ii in range(len(x1)):
                mutant.append(x1[ii] + round(self.vBeta * (x2[ii] - x3[ii])))
            if mutant == curr:
                self.robots[i].reinitializeExperienceWithRandomValues(seqLen)
            else:#---crossover                
                for ii in range(len(curr)):
                    if uniform(0,1) <= self.crProba:
                        mutant[ii] = curr[ii]                        
                self.robots[i].setExperienceWithClamping(mutant, seqLen) 
        #---reinitialize 25% of least fit robots
        tempFit = self.fit[:] #copy
        minVal = min(self.fit)#self.fit[leastFitRobot]
        leastFitIndices = set([])
        reinitNum = math.floor(0.25*len(self.robots))
        while len(leastFitIndices) < reinitNum:
            for i in range(len(self.robots)):
                if self.fit[i] <= minVal: 
                    leastFitIndices.add(i)
                if len(leastFitIndices) >= reinitNum: break#out of for
            tempFit = [i for i in tempFit if i != minVal]#remove minVal from list
            if len(tempFit) > 0: minVal = min(tempFit)
        for i in leastFitIndices:
            self.robots[i].reinitializeExperienceWithRandomValues(seqLen)       
        
        #---beta is reduced to encourage exploitation and reduce exploration
        if self.vBeta > 1/40: 
            self.vBeta = self.vBeta - 1/40        
        
        #---store the values for next gen to use
        self.temp[:] = []
        for i in range(len(self.robots)):
            self.temp.append(self.robots[i].getMotorRatesExperience())#all legs values stored in one row
    
    def findCurrentBestFitness(self):
        self.calcFitness()
    
    def run(self, seqLen):#will return false when it's time to stop   
        self.findCurrentBestFitness()  
        
        runState = RunCode.CONTINUE
        if self.generatingSeqForThisGen:#sequence is being generated
            i = 0
            for r in self.robots:
                if len(self.temp) == 0:
                    r.reinitializeExperienceWithRandomValues(seqLen)
                else:                                        
                    v = self.temp[i]; j = 0; 
                    for leg in r.legs:#assign DE values calculated from prev gen
                        leg.experience[:] = []
                        for _ in range(0, seqLen, 1):
                            leg.experience.append(v[j])
                            j += 1
                i += 1
        #---sequence generated. Now just execute as-is for entire epoch
        for r in self.robots:
            r.setMotorRateForSequence(self.seqNum)                                   
        self.seqNum += 1  
        
#         if not False in self.unfitThisFullGen:
#             self.seqNum = seqLen
#             self.repeatSeq = self.maxSeqRepetitions#no point running this sequence any more. Proceed to next one
                        
        if self.seqNum >= seqLen:#finished one sequence experience
            self.seqNum = 0
            self.generatingSeqForThisGen = False
            self.repeatSeq += 1
        if self.repeatSeq >= self.maxSeqRepetitions:#finished one generation
            self.repeatSeq = 0
            self.differentialEvolution(seqLen)
            runState = RunCode.NEXTGEN            
        
        return runState    
    
    def storeExperienceOfFittestRobot(self):
        self.calcFitness()
        if self.currentFittestRobot == self.const.UNDETERMINED:
            anyRobot = 0
            self.fittestRobotsMotorRates = self.robots[anyRobot].getEmptyMotorRatesExperience()#there's no fittest robot, so just returning zeroes as motor rates
        else:
            self.fittestRobotsMotorRates = self.robots[self.currentFittestRobot].getMotorRatesExperience()
        #print('experience of fittest robot: ', self.fittestRobotsMotorRates, ' robot:', self.currentFittestRobot)    
    
    def startNewGen(self):#NOTE: gets called when starting new epoch too
        self.generatingSeqForThisGen = True   
        self.unfitThisFullGen = [False] * len(self.robots)
        
    def startNewEpoch(self):
        self.startNewGen()
        self.temp[:] = []   
        self.resetDE()
        
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)
        
        
        
#--------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------
        
# Note: At least 4 robots are required for Differential Evolution to work
class DifferentialEvolution:#(AbstractRobotBehaviour):    
    def __init__(self, bodyPart): 
        self.infoString = ""  
        self.decimalPrecision = 2
        self.robots = bodyPart     
        self.repeatSeq = 0; 
        self.maxSeqRepetitions = 50 #how many times to repeat the sequence of seqNum's  
        self.seqNum = 0 #ordinal of the sequence of movements of an experience
        self.temp = []
        self.const = Constants()
        self.resetDE()
        #---Differential Evolution parameters
        self.masterBeta = 2.0 #beta is real number belongs to [0 -> 2]
        self.vBeta = 0 #variable beta
        self.crProba = 0.3 #crossover probability range [0 -> 1]  
        self.vBeta = self.masterBeta    
        self.fit = []
        self.startNewGen()
        self.fittestRobotsMotorRates = [0]
        
    def resetDE(self):
        self.epochFittestRobot = self.const.UNDETERMINED
        self.epochBestFitness = self.const.NOTFIT
        self.currentBestFitness = self.const.NOTFIT
        self.currentFittestRobot = self.const.UNDETERMINED

    def calcFitness(self):     
        self.fit[:] = []   
        for r in self.robots:
            self.fit.append(round(r.obj_body.position[0] - r.obj_body.startPosition[0], self.decimalPrecision))
#         #---assign zero fitness to any robot that became ulta
#         for r in range(len(self.robots)):
#             ang = self.robots[r].getBodyAngle()            
#             if ang > 90 and ang < 270:
#                 self.unfitThisFullGen[r] = True
#             if self.unfitThisFullGen[r]:
#                 self.fit[r] = self.NOTFIT  
        self.currentBestFitness = max(self.fit)
        if self.currentBestFitness == self.const.NOTFIT: 
            self.currentFittestRobot = self.const.UNDETERMINED
        else: 
            self.currentFittestRobot = self.fit.index(self.currentBestFitness)
    
    def differentialEvolution(self, seqLen):
        self.calcFitness(); oldSel = []; sel = []; i = 0; mutant = []
        for i in range(len(self.robots)):
            oldSel.append(i)
        
#         self.currentBestFitness = max(self.fit)
#         self.currentFittestRobot = self.fit.index(self.currentBestFitness)
        if self.currentBestFitness > self.epochBestFitness:
            self.epochBestFitness = self.currentBestFitness
            self.epochFittestRobot = self.fit.index(self.epochBestFitness)
        
        #---if none are fit, re-initialize all randomly coz there's no point doing DE
        if not False in self.unfitThisFullGen:
            for r in self.robots:
                r.reinitializeExperienceWithRandomValues(seqLen)
                self.temp[:] = []                   
            return
        #---proceed with DE 
        for i in range(len(self.robots)):
            sel[:] = []
            for s in oldSel:
                sel.append(s)

            if i == self.currentFittestRobot:#don't mess with the fittest
                continue
            del sel[i]#remove current car from list to be able to select another 3
            #---randomly choose three others for DE
            x1 = random.choice(sel); sel.remove(x1)
            x2 = random.choice(sel); sel.remove(x2)
            x3 = random.choice(sel); sel.remove(x3)
            mutant[:] = []
            x1 = self.robots[x1].getMotorRatesExperience()
            x2 = self.robots[x2].getMotorRatesExperience()
            x3 = self.robots[x3].getMotorRatesExperience()
            curr = self.robots[i].getMotorRatesExperience()   
            for ii in range(len(x1)):
                mutant.append(x1[ii] + round(self.vBeta * (x2[ii] - x3[ii])))
            if mutant == curr:
                self.robots[i].reinitializeExperienceWithRandomValues(seqLen)
            else:
                #---crossover
                for ii in range(len(curr)):
                    if uniform(0,1) <= self.crProba:
                        mutant[ii] = curr[ii]                        
                self.robots[i].setExperienceWithClamping(mutant, seqLen) 
        #---reinitialize 25% of least fit robots
        tempFit = self.fit[:] #copy
        minVal = min(self.fit)#self.fit[leastFitRobot]
        leastFitIndices = set([])
        percentageOfRobotsToReinitialize = 0.25
        reinitNum = math.floor(percentageOfRobotsToReinitialize * len(self.robots))
        while len(leastFitIndices) < reinitNum:
            for i in range(len(self.robots)):
                if self.fit[i] <= minVal: 
                    leastFitIndices.add(i)
                if len(leastFitIndices) >= reinitNum: break#out of for
            tempFit = [i for i in tempFit if i != minVal]
            if len(tempFit) > 0: minVal = min(tempFit)
        for i in leastFitIndices:
            self.robots[i].reinitializeExperienceWithRandomValues(seqLen)
        
        #---beta is reduced to encourage exploitation and reduce exploration
        if self.vBeta > 1/40: 
            self.vBeta = self.vBeta - 1/40        
        
        #---store the values for next gen to use
        self.temp[:] = []
        for i in range(len(self.robots)):
            self.temp.append(self.robots[i].getMotorRatesExperience())#all legs values stored in one row
        #self.calcFitness(); self.storeExperienceOfFittestRobot() #TODO:this line should be calculated after a generation completes
    
    def findCurrentBestFitness(self):
        self.calcFitness()
        self.currentBestFitness = max(self.fit)
        self.currentFittestRobot = self.fit.index(self.currentBestFitness)
        
    def storeExperienceOfFittestRobot(self):
        if self.currentFittestRobot == self.const.UNDETERMINED:
            anyRobot = 0
            self.fittestRobotsMotorRates = self.robots[anyRobot].getEmptyMotorRatesExperience()#there's no fittest robot, so just returning zeroes as motor rates
        else:
            self.fittestRobotsMotorRates = self.robots[self.currentFittestRobot].getMotorRatesExperience()
        #print('experience of fittest robot: ', self.fittestRobotsMotorRates, ' robot:', self.currentFittestRobot)
    
    def run(self, seqLen):#will return false when it's time to stop   
        self.findCurrentBestFitness()  
                
        runState = RunCode.CONTINUE
        if self.generatingSeqForThisGen:#sequence is being generated
            i = 0
            for r in self.robots:
                if len(self.temp) == 0:
                    r.reinitializeExperienceWithRandomValues(seqLen)
                else:                                        
                    v = self.temp[i]; j = 0; 
                    for leg in r.legs:#assign DE values calculated from prev gen
                        leg.experience[:] = []
                        for _ in range(0, seqLen, 1):
                            leg.experience.append(v[j])
                            j += 1
                i += 1
        #sequence generated. Now just execute as-is for entire epoch
        for r in self.robots:
            r.setMotorRateForSequence(self.seqNum)                                   
        self.seqNum += 1  
        
        if not False in self.unfitThisFullGen:
            self.seqNum = seqLen
            self.repeatSeq = self.maxSeqRepetitions#no point running this sequence any more. Proceed to next one
                        
        if self.seqNum >= seqLen:#finished one sequence experience
            self.seqNum = 0
            self.generatingSeqForThisGen = False
            self.repeatSeq += 1
        if self.repeatSeq >= self.maxSeqRepetitions:#finished one generation
            self.repeatSeq = 0
            self.differentialEvolution(seqLen)
            runState = RunCode.NEXTGEN            
        
        return runState    
    
    def startNewGen(self):#NOTE: gets called when starting new epoch too
        self.generatingSeqForThisGen = True   
        self.unfitThisFullGen = [False] * len(self.robots)
        
    def startNewEpoch(self):
        self.startNewGen()
        self.temp[:] = []   
        self.resetDE()
        
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)        