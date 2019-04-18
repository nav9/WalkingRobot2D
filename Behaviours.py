# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import random
from random import uniform
from WalkingRobot import RobotBody

class RunCode:
    STOP = 0
    CONTINUE = 1
    RESET = 2
    NEXTGEN = 3

# Note: At least 4 robots are required for Differential Evolution to work
class DifferentialEvolution:#(AbstractRobotBehaviour):    
    def __init__(self, robo): 
        self.infoString = ""  
        self.decimalPrecision = 2
        self.robots = robo    
        self.generatingSeqForThisGen = True   
        self.repeatSeq = 0; 
        self.maxSeqRepetitions = 30 #how many times to repeat the sequence of seqNum's  
        self.seqNum = 0 #ordinal of the sequence of movements of an experience
        self.temp = []
        self.resetDE()
        #---Differential Evolution parameters
        self.masterBeta = 2.0 #beta is real number belongs to [0 -> 2]
        self.vBeta = 0 #variable beta
        self.crProba = 0.3 #crossover probability range [0 -> 1]  
        self.vBeta = self.masterBeta    
        
    def resetDE(self):
        self.fittestRobotInGen = -1
        self.bestFitnessOfGen = 0
        self.currentBestFitness = 0
        self.currentFittestRobot = -1
#         self.cameraXOffset = 0
        
#     def updateChassisBodyPositionForFitness(self, x):
#         self.cameraXOffset += x
#     
    def getFitness(self):
        fit = []
        for r in self.robots:
            fit.append(round(r.chassis_body.position[0] - r.chassis_body.startPosition[0], self.decimalPrecision))
        return fit
        
    def differentialEvolution(self, seqLen):
        fit = self.getFitness(); oldSel = []; sel = []; i = 0; mutant = []
        for i in range(len(self.robots)):
            oldSel.append(i)
        
        self.bestFitnessOfGen = max(fit)
        self.fittestRobotInGen = fit.index(self.bestFitnessOfGen)
        leastFitRobot = fit.index(min(fit)) 
        for i in range(len(self.robots)):
            sel[:] = []
            for s in oldSel:
                sel.append(s)

            if i == self.fittestRobotInGen:#don't mess with the fittest
                continue
            del sel[i]#remove current car from list to be able to select another 3
            #---randomly choose three others for DE
            x1 = random.choice(sel); sel.remove(x1)
            x2 = random.choice(sel); sel.remove(x2)
            x3 = random.choice(sel); sel.remove(x3)
            mutant[:] = []
            x1 = self.robots[x1].getValues()
            x2 = self.robots[x2].getValues()
            x3 = self.robots[x3].getValues()
            curr = self.robots[i].getValues()   
            for ii in range(len(x1)):
                mutant.append(x1[ii] + round(self.vBeta * (x2[ii] - x3[ii])))
            if mutant == curr:
                self.robots[i].reinitializeWithRandomValues(seqLen)
            else:
                #---crossover
                for ii in range(len(curr)):
                    if uniform(0,1) <= self.crProba:
                        mutant[ii] = curr[ii]                        
                self.robots[i].setValuesWithClamping(mutant, seqLen) 
            self.robots[leastFitRobot].reinitializeWithRandomValues(seqLen)
        
        #---beta is reduced to encourage exploitation and reduce exploration
        if self.vBeta > 1/40: 
            self.vBeta = self.vBeta - 1/40        
        
        #---store the values for next gen to use
        self.temp[:] = []
        for i in range(len(self.robots)):
            self.temp.append(self.robots[i].getValues())#all legs values stored in one row
            
    def findCurrentBestFitness(self):
        fit = self.getFitness()
        self.currentBestFitness = max(fit)
        try:
            self.currentFittestRobot = fit.index(self.bestFitnessOfGen)
        except ValueError:
            self.currentFittestRobot = -1
    
    def run(self, seqLen):#will return false when it's time to stop   
        self.findCurrentBestFitness()     
        runState = RunCode.CONTINUE
        if self.generatingSeqForThisGen:#sequence is being generated
            i = 0
            for r in self.robots:
                if len(self.temp) == 0:
                    r.reinitializeWithRandomValues(seqLen)
                else:                 
                       
                    v = self.temp[i]; j = 0; 
                    for leg in r.legs:#assign DE values calculated from prev gen
                        leg.experience[:] = []
                        for k in range(0, seqLen, 1):
                            leg.experience.append(v[j])
                            j += 1
                i += 1
        #sequence generated. Now just execute as-is for entire epoch
        for r in self.robots:
            r.setMotorRateForSequence(self.seqNum)                                   
        self.seqNum += 1      
        if self.seqNum == seqLen:#finished one sequence experience
            self.seqNum = 0
            self.generatingSeqForThisGen = False
            self.repeatSeq += 1
        if self.maxSeqRepetitions == self.repeatSeq:#finished one generation
            self.repeatSeq = 0
            self.differentialEvolution(seqLen)
            runState = RunCode.NEXTGEN            
        
        return runState    
    
    def startNewGen(self):
        self.generatingSeqForThisGen = True     
        
    def startNewEpoch(self):
        self.temp[:] = []   
        self.resetDE()
        
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)

        