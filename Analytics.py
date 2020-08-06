# Author: Navin Ipe
# Created: July 2020
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import os
import glob
import pickle
import logging
import traceback
import numpy as np
import matplotlib.pyplot as plt
from Enums import Directories, ProgramMetrics, MainProgramParameters
    
class FileOperations:
    def __init__(self):
        self.dir = Directories()
    
    def savePickleFile(self, directory, filename, data):
        self.createDirectoryIfNotExisting(directory)
        with open(os.path.join(directory, filename), 'wb') as handle:
            pickle.dump(data, handle, protocol = pickle.HIGHEST_PROTOCOL)
            print('Saved ',directory, filename)
        
    def loadPickleFile(self, directory, filename):
        data = None #data will be loaded in exactly the same format it was stored in
        try:
            if directory == None: 
                with open(filename, 'rb') as handle: data = pickle.load(handle)
            else:
                self.createDirectoryIfNotExisting(directory) 
                with open(os.path.join(directory, filename), 'rb') as handle: data = pickle.load(handle)
            print('Loaded ',directory, filename)
        except FileNotFoundError:
            print("File not found: ", directory, filename)
        return data      
    
    def createDirectoryIfNotExisting(self, directory):
        if not os.path.exists(directory): 
            try: os.makedirs(directory)
            except FileExistsError:#in case there's a race condition where some other process creates the directory before makedirs is called
                pass  
            
    def checkIfFileExists(self, directory, filename):
        return os.path.isfile(os.path.join(directory, filename))
    
    def getUniqueNameForTerrainTrials(self, terrainName, trialNumber):#, numImaginaryRobots):
        return terrainName + '_tr' + str(trialNumber) + self.dir.PICKLE_EXTN

    def getUniqueNameForFinishingTime(self, numGens, nameOfCI, terrainName, trialNumber, numImaginaryRobots):
        return str(numGens) + 'Gen_' + nameOfCI +  '_' + terrainName + str(trialNumber) + '_' + str(numImaginaryRobots) + self.dir.PICKLE_EXTN    
    
    def loadAllPickleFilesFromDirectory(self, directory):
        return glob.glob(directory+'*'+self.dir.PICKLE_EXTN)

class ProgramAnalytics:
    def __init__(self):
        self.metricNames = ProgramMetrics()
        self.fileOps = FileOperations()
        
    def saveFinishingTime(self, numGens, runWhichCI, runWhichTerrain, trialNumber, numImaginaryRobots, totalTimeTaken):
        filename = self.fileOps.getUniqueNameForFinishingTime(numGens, runWhichCI, runWhichTerrain, trialNumber, numImaginaryRobots)
        programMetrics = {
            self.metricNames.numGens: numGens,
            self.metricNames.timeToCrossFinishLine: totalTimeTaken,
            self.metricNames.runWhichCI: runWhichCI,
            self.metricNames.runWhichTerrain: runWhichTerrain,
            self.metricNames.trialNumber: trialNumber,
            self.metricNames.numImaginaryRobots: numImaginaryRobots
        }
        self.fileOps.savePickleFile(self.fileOps.dir.programMetricsFolder, filename, programMetrics)
        
    def loadProgramRunData(self):
        data = []
        fileList = self.fileOps.loadAllPickleFilesFromDirectory(self.fileOps.dir.programMetricsFolder)
        #---go through all filenames and load dicts
        trialNums = set(); robotNums = set(); genNums = set()
        for filename in fileList:
            d = self.fileOps.loadPickleFile(None, filename)
            genNums.add(d[self.metricNames.numGens])
            trialNums.add(d[self.metricNames.trialNumber])
            robotNums.add(d[self.metricNames.numImaginaryRobots]) 
            data.append(d)
        print('\n----------------- Results of ',len(genNums),' maxGen types, ',len(trialNums),' trials and ', robotNums, ' robots:')#The +1 is because trials start with 0
        print('Trial, numRobots, CI, Terrain, Real robot\'s Time (s)')
        for t in trialNums:
            for g in genNums:
                for r in robotNums:
                    for d in data:
                        try:
                            if d[self.metricNames.trialNumber] == t and d[self.metricNames.numImaginaryRobots] == r and d[self.metricNames.numGens] == g:
                                print(str(t+1)+", "+str(r)+", "+d[self.metricNames.runWhichCI]+', '+d[self.metricNames.runWhichTerrain]+', '+str(float(d[self.metricNames.timeToCrossFinishLine])/MainProgramParameters.MAX_GENS))
                        except Exception as e:
                            print(e)
                            print('exception caught for trial ', t, ' gen ', g, ' numRobot ', r)
                            logging.error(traceback.format_exc(None, True))
    
class FitnessAnalytics:    
    def __init__(self):
        self.file = FileOperations()
        self.bestFitness = [] #appended to at the end of each epoch
        self.timeTaken = [] #appended to when the robot crosses the finish line

    def saveDataToDisk(self, directory, filename):
        self.file.savePickleFile(directory, filename, self.bestFitness)
        self.file.savePickleFile(directory, filename, self.timeTaken)
    
    def loadRatesPositionDataFromDisk(self, directory, filename):
        self.bestFitness = self.file.loadPickleFile(directory, filename)
        self.timeTaken = self.file.loadPickleFile(directory, filename)
        return self.bestFitness, self.timeTaken
        

class TestAnalyticsForMovementAccuracy:
    def __init__(self):
        self.file = FileOperations()

    def saveDataToDisk(self, directory, filename, data):
        self.file.savePickleFile(directory, filename, data) #saves [[motorRates], [x1,y1], [x2,y2]...] or [numTouch1, numTouch2,...4] or [[angle1, angle2...angleNumFrames], [],...]        
    
    def loadRatesPositionDataFromDisk(self, directory, filename):
        ratesAndPositions = self.file.loadPickleFile(directory, filename) #returns [[motorRates], [x1,y1], [x2,y2]...]
        gotRate = False
        rate = None; positions = []
        for pos in ratesAndPositions:
            if gotRate: positions.append(pos)
            else: rate = pos;gotRate = True
        return rate, positions

    def loadSurfaceTouchDataFromDisk(self, directory, filename):
        return self.file.loadPickleFile(directory, filename) #returns [numTouch1, numTouch2,...4]     

    def loadRobotAnglesDataFromDisk(self, directory, filename):
        return self.file.loadPickleFile(directory, filename) #[[angle1, angle2...angleNumFrames], [],...]    
    
    def generateRatesPositionFilename(self, trial):
        return "TestMotorRateAccuracy_trial"+str(trial)
    
    def generateSurfaceTouchFilename(self, trial):
        return "SurfaceTouch_trial"+str(trial)
        
    def generateRobotAnglesFilename(self, trial):
        return "ChassisAngle_trial"+str(trial)    

    def plot(self, directory, xPositions, yPositions, rates, surfaceTouch, robotAngles):#arrays will be in the form xPositions = [[1,2,5], [5,7,2,2,5], [7,2,5]...]. Same for yPositions
        self.boxPlots(directory, xPositions, yPositions)
        self.groupedBarsMotorRates(directory, rates)
        self.groupedBarsSurfaceTouch(directory, surfaceTouch)        
        self.multipleParameters(directory, robotAngles, surfaceTouch) #robotAngles = [[chAng, leg1Ang, leg2Ang, leg3Ang, leg4Ang], [], ...]     
        
    def multipleParameters(self, directory, angles, surfaceTouch):#[[[[chA1,lA1,lA2,lA3,lA4],[],...aNumFrames], [], []], []]. angles[0] gives all simulation run's for first trial = [[chA1,lA1,lA2,lA3,lA4], [], []], []]        
        #angles is numTrials=20 long.
        #angles[0] is numSimulations=100. [[[360, 6, 3, 355, 358], [359, 12, 6, 349, 356], ...
        #angles[0][0] is numFrames=50. [[360, 6, 3, 355, 358], [359, 12, 6, 349, 356], ...
        SMALL_SIZE = 8
        plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
        plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
        plt.rc('axes', labelsize=SMALL_SIZE)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
        plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
        plt.rc('figure', titlesize=SMALL_SIZE)  # fontsize of the figure title        
        x = np.arange(len(angles[0][0]))#range from 0 to numFrames
        subplotNumRows = 3; subplotNumCols = 3
        for t in range(len(angles)):#for each trial
            fig, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9)) = plt.subplots(subplotNumRows, subplotNumCols, sharex=True, constrained_layout=True)
            fig.suptitle('Trial'+str(t+1)+'. '+str(len(angles[t]))+' sims')
            #---leg and chassis angles
            for sim in angles[t]:#for each simulation. Each sim = [[chA1,lA1,lA2,lA3,lA4], [], []...numFrames]
                chassisAngles = []; leg1Angles = []; leg2Angles = []; leg3Angles = []; leg4Angles = [];  
                for frame in sim:#for each frame = [chA1,lA1,lA2,lA3,lA4]
                    chassisAngles.append(frame[0])#taking the first angle, which is the chassis angle
                    leg1Angles.append(frame[1]); leg2Angles.append(frame[2])
                    leg3Angles.append(frame[3]); leg4Angles.append(frame[4])                
                ax1.plot(x, leg1Angles, '.'); ax1.set_title('Limb1 Angle'); ax1.set(ylabel='Angle')
                ax3.plot(x, leg2Angles, '.'); ax3.set_title('Limb2 Angle'); ax3.set(ylabel='Angle')
                ax5.plot(x, leg3Angles, '.'); ax5.set_title('Limb3 Angle'); ax5.set(ylabel='Angle')
                ax7.plot(x, leg4Angles, '.'); ax7.set_title('Limb4 Angle'); ax7.set(ylabel='Angle')
                ax9.plot(x, chassisAngles, '.'); ax9.set_title("Chassis Angle", fontsize=8); ax9.set(ylabel='Angle')
            #---touch points
            for eachSim in surfaceTouch[t]:#for each simulation in trial
                touches1=[]; touches2=[]; touches3=[]; touches4=[]
                for eachFrame in eachSim:#eachFrame=[l1,l2,l3,l4]
                    touches1.append(eachFrame[0]); touches2.append(eachFrame[1])
                    touches3.append(eachFrame[2]); touches4.append(eachFrame[3])              
                ax2.plot(x, touches1, '.'); ax2.set_title('Limb1 Touch'); ax2.set(ylabel='Touch')
                ax4.plot(x, touches2, '.'); ax4.set_title('Limb2 Touch'); ax4.set(ylabel='Touch')
                ax6.plot(x, touches3, '.'); ax6.set_title('Limb3 Touch'); ax6.set(ylabel='Touch')
                ax8.plot(x, touches4, '.'); ax8.set_title('Limb4 Touch'); ax8.set(ylabel='Touch')
            
            #---motor rates
#             for ax in fig.get_axes():
#                 ax.set(xlabel='frames', ylabel='angle')
                #ax.label_outer()                              
            #plt.xlabel('frames'); plt.ylabel('chassis angle'); plt.title('Trial'+str(t+1)+'. '+str(len(angles[t]))+' simulations', loc='center', pad=None)
            plt.savefig(directory+' AnglesTouches_Trial'+str(t)+'.png')
            fig.tight_layout()
            plt.subplots_adjust(wspace=0.5, hspace=0.3)#height and width spacing between subplots
            plt.xlabel('Frames')
            #plt.show(block=False)  
            #break          

    def groupedBarsMotorRates(self, directory, rates):
        #print(rates)
        labels = []#the groups
        m1=[]; m2=[]; m3=[]; m4=[]
        i = 1
        for rateList in rates:
            m1.append(abs(rateList[0]))
            m2.append(abs(rateList[1]))
            m3.append(abs(rateList[2]))
            m4.append(abs(rateList[3]))
            labels.append(str(i)); i = i + 1
        x = np.arange(len(labels)) #label locations
        width = 0.08 #bar width
        fig, ax = plt.subplots()
        rects1 = ax.bar(x - 2*width+(width/2), m1, width, label='motor1')
        rects2 = ax.bar(x - width/2, m2, width, label='motor2')
        rects3 = ax.bar(x + width/2, m3, width, label='motor3')
        rects4 = ax.bar(x + 2*width-(width/2), m4, width, label='motor4')    
        #function within function
#         def autolabel(rects):
#             """Attach a text label above each bar in *rects*, displaying its height."""
#             for rect in rects:
#                 height = rect.get_height()
#                 ax.annotate('{}'.format(int(height)),
#                             xy=(rect.get_x() + rect.get_width() / 2, height),
#                             xytext=(0, 0),  # 3 points vertical offset
#                             textcoords="offset points",
#                             ha='center', va='bottom')
        ax.set_ylabel('abs(motor rate)'); ax.set_xlabel('Trials')
        #ax.set_title('Avg. best fitness in 2500 gen.')
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        #ax.legend(bbox_to_anchor=(1.05,1),loc='upper left')
        ax.legend(bbox_to_anchor=(0.,1.02,1.,.102),loc='lower left', mode="expand", ncol=4)    
        #autolabel(rects1); autolabel(rects2); autolabel(rects3); autolabel(rects4)    
        fig.tight_layout()
        plt.savefig(directory+'motorRatesForXYAccuracies.png')
        #plt.show(block=False)     
        
    def groupedBarsSurfaceTouch(self, directory, surfaceTouch):#surfaceTouch=#[ [ [ [l1,l2,l3,l4], [], ...numFrames ], [], [], ... numSims ], [], []...numTrials ]
        #print('surface touch: ',surfaceTouch)
        trialNum = 1
        labels = []#the groups
        leg1=[]; leg2=[]; leg3=[]; leg4=[] #each will have numTrials length   
        for trial in surfaceTouch:
            totalL1 = 0; totalL2 = 0; totalL3 = 0; totalL4 = 0;
            for eachSim in trial:
                for eachFrame in eachSim:#eachFrame=[l1,l2,l3,l4]
                    totalL1 += eachFrame[0]; totalL2 += eachFrame[1]
                    totalL3 += eachFrame[2]; totalL4 += eachFrame[3]  
             
            leg1.append(totalL1); leg2.append(totalL2)
            leg3.append(totalL3); leg4.append(totalL4)
            labels.append(str(trialNum)); trialNum += 1
        x = np.arange(len(labels)) #label locations
        width = 0.08 #bar width
        fig, ax = plt.subplots()
        rects1 = ax.bar(x - 2*width+(width/2), leg1, width, label='limb1')
        rects2 = ax.bar(x - width/2, leg2, width, label='limb2')
        rects3 = ax.bar(x + width/2, leg3, width, label='limb3')
        rects4 = ax.bar(x + 2*width-(width/2), leg4, width, label='limb4')    
        #function within function
#         def autolabel(rects):
#             """Attach a text label above each bar in *rects*, displaying its height."""
#             for rect in rects:
#                 height = rect.get_height()
#                 ax.annotate('{}'.format(int(height)),
#                             xy=(rect.get_x() + rect.get_width() / 2, height),
#                             xytext=(0, 0),  # 3 points vertical offset
#                             textcoords="offset points",
#                             ha='center', va='bottom')
        ax.set_ylabel('limb contact'); ax.set_xlabel('Trials')
        #ax.set_title('Avg. best fitness in 2500 gen.')
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        #ax.legend(bbox_to_anchor=(1.05,1),loc='upper left')
        ax.legend(bbox_to_anchor=(0.,1.02,1.,.102),loc='lower left', mode="expand", ncol=4)    
        #autolabel(rects1); autolabel(rects2); autolabel(rects3); autolabel(rects4)    
        fig.tight_layout()
        plt.savefig(directory+'surfaceTouchesForXYAccuracies.png')
        plt.show(block=False)         
    
    def boxPlots(self, directory, xPositions, yPositions):
        ticks = []
        for i in range(len(xPositions)):
            ticks.append(str(i+1))
        plt.figure()
        
        rangeAndOffsetsForLeftBox = np.array(range(len(xPositions))) * 2+ 1
        rangeAndOffsetsForRightBox = np.array(range(len(yPositions))) * 2 + 1.7
        bpLeft = plt.boxplot(xPositions, positions = rangeAndOffsetsForLeftBox + 0.4, sym="")
        bpRight = plt.boxplot(yPositions, positions = rangeAndOffsetsForRightBox + 0.4, sym="")
        red = '#D7191C'; black = '#000000'; #blue = '#2C7BB6' #http://colorbrewer2.org/ 
        self.changeBoxColor(bpLeft, red)
        self.changeBoxColor(bpRight, black)
        plt.xlabel('Trials'); plt.ylabel('pixels')
        plt.plot([], c=red, label='dx'); plt.plot([], c=black, label='dy') #temporary lines for legend
        plt.legend()
        
        plt.xticks(rangeAndOffsetsForLeftBox , ticks)
        plt.tight_layout()
        plt.savefig(directory+'xyAccuracies.png')
        #plt.show(block=False) 
        
    def changeBoxColor(self, boxPlot, c):
        plt.setp(boxPlot['boxes'], color=c)
        plt.setp(boxPlot['whiskers'], color=c)
        plt.setp(boxPlot['caps'], color=c)
        plt.setp(boxPlot['medians'], color=c)

