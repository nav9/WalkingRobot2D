# Author: Navin Ipe
# Created: July 2020
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author
class TestRunMode:
    CREATING_RESULTS = 1
    VIEWING_RESULTS = 2
    
class MainProgramParameters:
    MAX_TRIALS_TO_RUN = 10
    FINISH_LINE_POSITION_FROM_END = 100 #800 for near or 100 for far
    LEGS = '--#--'  
    MAX_GENS = 30
    NUM_IMAGINARY_ROBOTS = 30 #population
    MAX_GENS_FOR_STUCK_CHECK = 10 #if stuck at same positions for more than 10 tries of main robot movement, then activate unstuck mode
    DISTANCE_FOR_ASSUMING_STUCK = 20 #pixels
    GENERATIONS_TO_PERSIST_STUCK = 5
    TEST_MODE_RUN_STATE = TestRunMode.VIEWING_RESULTS #TestRunMode.CREATING_RESULTS or TestRunMode.VIEWING_RESULTS
    
#------------------------------------------------------------------
#------------------------------------------------------------------
#------------------------------------------------------------------
        
class Run:
    IMAGINATION_TWIN = 0
    IMAGINATION_TWIN_TRIAL_MULTI_RUNS = 1
    ACTUAL_IMAGINATION = 2
    HEAVEN = 3
    MOVEMENT_ACCURACY_CHECKER = 4
    
class RunStep:
    IMAGINARY_MOTOR_EXEC = 0
    IMAGINARY_GENERATION = 1
    REAL_MOTOR_EXEC = 2
    REAL_GENERATION = 3   
    
class RunCI:
    RANDOM = 'RANDOM'
    DE = 'DE'
    PSO = 'PSO'
    
class Terrains:
    FLAT_GROUND = 'FLAT_GROUND' 
    RANDOM_BOXES_LOW_DENSE = 'RANDOM_BOXES'
    RANDOM_SPHERES_LOW_DENSE = 'RANDOM_SPHERE'
    STAIRCASE_UP_DOWN = 'STAIRCASE'
    ALTERNATOR = 'ALTERNATOR'
    
class ShapeTypes:
    RECTANGLE = 1
    CIRCLE = 2
    
class ShapeProperties:
    COL = 'COL'
    ROW = 'ROW'
    WIDTH = 'WIDTH' #Also used as radius of circle
    HEIGHT = 'HEIGHT'
    
class Directions:
    def __init__(self):
        self.dirn = {'UP':1, 'DOWN':2, 'LEFT':3, 'RIGHT':4, #'TOPRIGHT':5, 'TOPLEFT':6, 'BOTTOMRIGHT':7, 'BOTTOMLEFT':8
                     }
    def getDirn(self): 
        return self.dirn  
    
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
    
class Directories:
    PICKLE_EXTN = '.pickle' #file extension
    terrainObjectsFolder = 'terrainObjects/'
    programMetricsFolder = 'runMetrics/'
    
class ProgramMetrics:
    timeToCrossFinishLine = 'timeToCrossFinishLine'
    runWhichCI = 'runWhichCI'
    runWhichTerrain = 'runWhichTerrain'
    trialNumber = 'trialNumber'
    numImaginaryRobots = 'numImaginaryRobots'
    numGens = 'numGens'
                
    
