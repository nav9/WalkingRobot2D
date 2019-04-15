import random
from WalkingRobot import RobotBody, ActionsNetwork

class DifferentialEvolution:#(AbstractRobotBehaviour):    
    def __init__(self, robo): 
        self.infoString = ""  
        self.robots = robo         
        self.experienced = False
        self.experienceLevel = 1; self.maxExperienceLevel = 20 #The number of dT times a leg is moved
        self.epoch = 0; self.maxEpochs = 5         
        #self.firstRunForAnExperienceLevel = True    
        self.gen = 0; self.maxGen = 10 #how many times to repeat the sequence of dT's  
        self.dT = 0 #ordinal of the sequence of movements of an experience 
        for r in self.robots:
            r.temp = [] ; print('temp initiated'); print(r.temp)
            r.experience = []               
        
    def run(self):#will return false when it's time to stop
        continueRunning = True
        if self.experienceLevel == self.maxExperienceLevel:
            continueRunning = False
            return continueRunning
        else:
            if self.epoch == self.maxEpochs:
                #run DE equation
                self.reset()
                for r in self.robots:
                    r.experience[:] = []
                self.experienced = False                
            else:
                if self.gen == self.maxGen:
                    self.gen = 0; self.epoch += 1
                    #store the best fit robot's parameters
                else:
                    if self.dT == self.experienceLevel:
                        self.dT = 0
                        self.experienced = True
                    else:
                        if not self.experienced:
                            #generate a new random pattern
                            for r in self.robots:
                                rate = []
                                for leg in r.legs:
                                    thisRate = random.choice(leg.motor.legRateRange)
                                    rate.append(thisRate)                                    
                                    leg.motor.rate = thisRate
                                r.experience.append(rate)
                    if self.experienced:
                        for r in self.robots:
                            motorID = 0
                            for leg in r.legs:
                                leg.motor.rate = r.experience[self.dT][motorID]
                                motorID += 1
                    self.dT += 1
        self.infoString = "Experience: "+str(self.experienceLevel)+"  Epoch: "+str(self.epoch)+"  Gen: "+str(self.gen)+"  Fittest: "
                    
        return continueRunning
    
    def fixValueBounds(self):
        for ob in self.robots:
            if ob.values < ob.legs[0].motor.legRateRange[0] or ob.values > ob.legs[0].motor.legRateRange[-1]:
                ob.values = random.choice(ob.legs[0].motor.legRateRange)
        
    def reset(self):
        for ob in self.robots:
            ob.values[:] = []     
            ob.chassis_body.position = ob.chassis_body.start_position 
            ob.chassis_body.angle = ob.chassis_body.startAngle
            for leg in ob.legs:
                ob.values.append(random.choice(leg.motor.legRateRange))
                leg.leg_body.position = leg.leg_body.startPosition
                leg.leg_body.angle = leg.leg_body.startAngle
                leg.motor.rate = 0
        