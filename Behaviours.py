import random
from WalkingRobot import RobotBody, ActionsNetwork

class RunCode:
    STOP = 0
    CONTINUE = 1
    RESET = 2

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
        self.runState = RunCode.CONTINUE
        
    def run(self, seqLen):#will return false when it's time to stop        
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
                self.runState = RunCode.RESET
        
        return self.runState
    
#         if self.runState == RunCode.RESET:
#             self.runState = RunCode.CONTINUE
#             for r in self.robots:
#                 r.experience = []
#         if self.experienceLevel == self.maxExperienceLevel:
#             self.runState = RunCode.STOP
#             return self.runState
#         else:
#             if self.epoch == self.maxEpochs:
#                 #run DE equation
#                 self.epoch = 0
#                 self.generatingSeq = False
#                 self.runState = RunCode.RESET
#                 return self.runState                
#             else:
#                 if self.repeatSeq == self.maxSeqRepetitions:
#                     self.repeatSeq = 0; self.epoch += 1
#                     #store the best fit robot's parameters
#                 if self.repeatSeq < self.maxSeqRepetitions:
#                     if self.seqNum == self.experienceLevel:
#                         self.seqNum = 0
#                         self.generatingSeq = True
#                     else:
#                         if not self.generatingSeq:
#                             #generate a new random pattern
#                             for r in self.robots:
#                                 rate = []
#                                 for leg in r.legs:
#                                     thisRate = random.choice(leg.motor.legRateRange)
#                                     rate.append(thisRate)                                    
#                                     leg.motor.rate = thisRate
#                                 r.experience.append(rate)
#                     if self.generatingSeq:
#                         for r in self.robots:
#                             motorID = 0
#                             for leg in r.legs:
#                                 leg.motor.rate = r.experience[self.seqNum][motorID]
#                                 motorID += 1
#                     self.seqNum += 1
#                 self.repeatSeq += 1
#         self.infoString = "Experience: "+str(self.experienceLevel)+"  Epoch: "+str(self.epoch)+"  Gen: "+str(self.repeatSeq)+"  Fittest: "
#                     
#         return RunCode.CONTINUE
    
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)

        