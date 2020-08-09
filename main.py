# Author: Navin Ipe
# Created: April 2019. Remodified: July 2020. See Git history.
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author
 
# Installation instructions:
# It is recommended that you install Python as part of a virtual environment like pyEnv so that it does not mess up your system's Python. Python3 is preferred.
# Install dependent libraries:
# >>> pip3 install pygame
# >>> pip3 install pymunk
# >>> pip3 install numpy
# >>> pip3 install scipy
# >>> pip3 install matplotlib
# sudo apt install speech-dispatcher #if not already installed by default on Ubuntu (this is for speech notifications, and not really necessary)
# Now simply run using: 
# >>> python3 main.py

import os
import logging, traceback
from Analytics import ProgramAnalytics
from Enums import RunCI, Terrains, Run, MainProgramParameters
from Worlds import ImaginationTwin, ActualImagination, Heaven, MovementAccuracyTestWorld
    
class MainSimulator(object):
    def __init__(self, legs, simulationToRun):
        self.worlds = []
        self.worldOrdinal = -1        
        #---registration of the worlds to runWorld
        #--------------------------------------------------------
        #-------------------- SINGLE RUNS -----------------------
        #--------------------------------------------------------
        if simulationToRun == Run.IMAGINATION_TWIN: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.FLAT_GROUND, None))
        #if simulationToRun == Run.IMAGINATION_TWIN: self.worlds.append(ImaginationTwin(legs, RunCI.DE, Terrains.RANDOM_BOXES_LOW_DENSE, None))
        #if simulationToRun == Run.IMAGINATION_TWIN: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.RANDOM_SPHERES_LOW_DENSE, None))
        #if simulationToRun == Run.IMAGINATION_TWIN: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.STAIRCASE_UP_DOWN, None))
        #if simulationToRun == Run.IMAGINATION_TWIN: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.ALTERNATOR, None))
        #--------------------------------------------------------
        #--------------------- LONG RUNS ------------------------
        #--------------------------------------------------------        
        if simulationToRun == Run.IMAGINATION_TWIN_TRIAL_MULTI_RUNS:
            startTrialNum = 0
            trialRange = range(startTrialNum, MainProgramParameters.MAX_TRIALS_TO_RUN)
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.FLAT_GROUND, trialNum));
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.DE, Terrains.FLAT_GROUND, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.PSO, Terrains.FLAT_GROUND, trialNum))
              
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.RANDOM_BOXES_LOW_DENSE, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.DE, Terrains.RANDOM_BOXES_LOW_DENSE, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.PSO, Terrains.RANDOM_BOXES_LOW_DENSE, trialNum))
                
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.RANDOM_SPHERES_LOW_DENSE, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.DE, Terrains.RANDOM_SPHERES_LOW_DENSE, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.PSO, Terrains.RANDOM_SPHERES_LOW_DENSE, trialNum))
               
#             for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.STAIRCASE_UP_DOWN, trialNum))
#             for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.DE, Terrains.STAIRCASE_UP_DOWN, trialNum))
#             for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.PSO, Terrains.STAIRCASE_UP_DOWN, trialNum))
               
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.RANDOM, Terrains.ALTERNATOR, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.DE, Terrains.ALTERNATOR, trialNum))
            for trialNum in trialRange: self.worlds.append(ImaginationTwin(legs, RunCI.PSO, Terrains.ALTERNATOR, trialNum))   
    
        #--------------------------------------------------------
        #--------------------- MISC RUNS ------------------------
        #--------------------------------------------------------            
        if simulationToRun == Run.ACTUAL_IMAGINATION: self.worlds.append(Heaven(legs))
        if simulationToRun == Run.HEAVEN: self.worlds.append(ActualImagination(legs))  
        if simulationToRun == Run.MOVEMENT_ACCURACY_CHECKER: self.worlds.append(MovementAccuracyTestWorld(legs))           
    
    def nextWorld(self):
        self.worldOrdinal += 1
        if self.worldOrdinal < len(self.worlds):
            w = self.worlds[self.worldOrdinal]
            w.initialize()
            #time.sleep(5)
            w.runWorld()
            w.delete()    
        return self.worldOrdinal < len(self.worlds)#any more worlds to process?

#-----------------------------------------------
#-----------------------------------------------
#             PROGRAM STARTS HERE
#-----------------------------------------------
#-----------------------------------------------
if __name__ == '__main__':
    noErrors = True
    # - Single leg part, -- Two leg parts, # Chassis
    legs = MainProgramParameters.LEGS
    #sim = MainSimulator(legs, Run.IMAGINATION_TWIN)
    #sim = MainSimulator(legs, Run.IMAGINATION_TWIN_TRIAL_MULTI_RUNS)    
    #sim = MainSimulator(legs, Run.ACTUAL_IMAGINATION)
    #sim = MainSimulator(legs, Run.HEAVEN)
    sim = MainSimulator(legs, Run.MOVEMENT_ACCURACY_CHECKER)
    
    try:
        while sim.nextWorld():
            pass
    except BaseException as e:
        noErrors = False
        print(e); os.system('spd-say '+"oh_no_the_program_crashed")
        logging.error(traceback.format_exc(None, True))        
    
#     results = ProgramAnalytics()
#     results.loadProgramRunData()    
    print('\n\n... Program complete ...')
    if noErrors: os.system('spd-say "Program_completed_successfully"')
    else: os.system('spd-say "Exiting_with_errors"')
    
