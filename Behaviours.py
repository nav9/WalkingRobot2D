import random
from random import uniform
from WalkingRobot import RobotBody, ActionsNetwork

class RunCode:
    STOP = 0
    CONTINUE = 1
    RESET = 2
    NEXTGEN = 3

class DifferentialEvolution:#(AbstractRobotBehaviour):    
    def __init__(self, robo): 
        self.infoString = ""  
        self.robots = robo         
        self.generatingSeq = True
#         self.experienceLevel = 1; self.maxExperienceLevel = 20 #The number of seqNum times a leg is moved
#         self.epoch = 0; self.maxEpochs = 5         
        #self.firstRunForAnExperienceLevel = True    
        self.repeatSeq = 0; 
        self.maxSeqRepetitions = 2 #how many times to repeat the sequence of seqNum's  
        self.seqNum = 0 #ordinal of the sequence of movements of an experience
    
    def getFitness(self):
        fit = []
        for r in self.robots:
            fit.append(r.chassis_body.position[0])
        return fit
        
    def differentialEvolution(self):
        fit = self.getFitness(); oldSel = []; sel = []; i = 0
        for i in range(len(self.robots)):
            sel.append(i); oldSel.append(i)
        
        bestFitnessThisGen = max(fit)
        fittestCar = fit.index(bestFitnessThisGen)
        leastFitCar = fit.index(min(fit)) 
        for i in range(len(self.robots)):
            sel = []
            for s in oldSel:
                sel.append(s)
            
            if i == fittestCar:#don't mess with the fittest
                continue
            del sel[i]
            #---randomly choose three others for DE
            x1 = random.choice(sel); sel.remove(x1)
            x2 = random.choice(sel); sel.remove(x2)
            x3 = random.choice(sel); sel.remove(x3)
            mutant = []
            x1 = self.robots[x1].experience
            x2 = self.robots[x2].experience
            x3 = self.robots[x3].experience
            prev = self.robots[i].experience               
            for i in range(len(x1)):
                mutant.append(x1[i] + round(self.vBeta * (x2[i] - x3[i])))
            if mutant == prev:
                self.robots[i].reinitializeWithRandomValues()
            else:
                #---crossover
                for i in range(len(prev)):
                    if uniform(0,1) <= self.crProba:
                        mutant[i] = prev[i]
                self.robots[i].setValues(mutant) 
            self.robots[leastFitCar].reinitializeWithRandomValues()
        
        #---beta is reduced to encourage exploitation and reduce exploration
        if self.vBeta > 1/40: 
            self.vBeta = self.vBeta - 1/40        
        
    def reinitializeWithRandomValues(self):
        for r in self.robots:
            for leg in r.legs:
                thisRate = random.choice(leg.motor.legRateRange)
                r.experience.append(thisRate)                                    
                leg.motor.rate = thisRate
                            
    def generateSequence(self, seqLen):
        for i in range(0, seqLen, 1):
            for r in self.robots:
                for leg in r.legs:
                    thisRate = random.choice(leg.motor.legRateRange)
                    r.experience.append(thisRate)            

    def run(self, seqLen):#will return false when it's time to stop        
        runState = RunCode.CONTINUE
        if self.generatingSeq:#sequence is being generated
            self.generateSequence(seqLen)
        #sequence generated. Now just execute as-is for entire epoch
        for r in self.robots:
            for leg in r.legs:                                    
                leg.motor.rate = r.experience[self.seqNum]
        self.seqNum += 1      
        if self.seqNum == seqLen:
            self.seqNum = 0
            self.generatingSeq = False
            self.repeatSeq += 1
        if self.maxSeqRepetitions == self.repeatSeq:
            self.repeatSeq = 0
            runState = RunCode.NEXTGEN
            #self.differentialEvolution()
        
        return runState    
    
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)

        