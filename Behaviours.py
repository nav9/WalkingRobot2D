import random
from WalkingRobot import RobotBody, ActionsNetwork

class RunCode:
    STOP = 0
    CONTINUE = 1
    RESET = 2
    NEXTEPOCH = 3

class DifferentialEvolution:#(AbstractRobotBehaviour):    
    def __init__(self, robo): 
        self.infoString = ""  
        self.robots = robo         
        self.generatingSeq = True
#         self.experienceLevel = 1; self.maxExperienceLevel = 20 #The number of seqNum times a leg is moved
#         self.epoch = 0; self.maxEpochs = 5         
        #self.firstRunForAnExperienceLevel = True    
        self.repeatSeq = 0; self.maxSeqRepetitions = 10 #how many times to repeat the sequence of seqNum's  
        self.seqNum = 0 #ordinal of the sequence of movements of an experience 
        
    def run(self, seqLen):#will return false when it's time to stop        
        runState = RunCode.CONTINUE
        if self.generatingSeq:#sequence is being generated
            for r in self.robots:
                for leg in r.legs:
                    thisRate = random.choice(leg.motor.legRateRange)
                    r.experience.append(thisRate)                                    
                    leg.motor.rate = thisRate
        else:#sequence generated. Now just execute as-is for entire epoch
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
            runState = RunCode.NEXTEPOCH
        
        return runState    
    
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)

        