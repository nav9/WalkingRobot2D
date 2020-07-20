# Author: Navin Ipe
# Created: July 2020
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

class FileOperations:
    def __init__(self):
        pass
    
    def savePickleFile(self, directory, filename, data):
        self.createDirectoryIfNotExisting(directory)
        with open(os.path.join(directory, filename), 'wb') as handle:
            pickle.dump(data, handle, protocol = pickle.HIGHEST_PROTOCOL)
            print('Saved ',directory, filename)
        
    def loadPickleFile(self, directory, filename):
        data = None #data will be loaded in exactly the same format it was stored in
        try:
            self.createDirectoryIfNotExisting(directory) 
            with open(os.path.join(directory, filename), 'rb') as handle:
                data = pickle.load(handle)
                print('Loaded ',directory, filename)
        except FileNotFoundError:
            print("File not found: ", directory, filename)
        return data      
    
    def createDirectoryIfNotExisting(self, folder):
        if not os.path.exists(folder): 
            try: os.makedirs(folder)
            except FileExistsError:#in case there's a race condition where some other process creates the directory before makedirs is called
                pass      

class FitnessAnalytics:    
    def __init__(self):
        self.file = FileOperations()
        self.bestFitness = [] #appended to at the end of each epoch
        self.timeTaken = [] #appended to when the robot crosses the finish line

    def saveDataToDisk(self, directory, filename):
        self.file.savePickleFile(directory, filename, self.bestFitness)
        self.file.savePickleFile(directory, filename, self.timeTaken)
    
    def loadDataFromDisk(self, directory, filename):
        self.bestFitness = self.file.loadPickleFile(directory, filename)
        self.timeTaken = self.file.loadPickleFile(directory, filename)
        return self.bestFitness, self.timeTaken
        

class TestAnalyticsForMovementAccuracy:
    def __init__(self):
        self.file = FileOperations()

    def saveDataToDisk(self, directory, filename, data):
        self.file.savePickleFile(directory, filename, data) #saves [[motorRates], [x1,y1], [x2,y2]...]
    
    def loadDataFromDisk(self, directory, filename):
        ratesAndPositions = self.file.loadPickleFile(directory, filename) #returns [[motorRates], [x1,y1], [x2,y2]...]
        gotRate = False
        rate = None; positions = []
        for pos in ratesAndPositions:
            if gotRate: positions.append(pos)
            else: rate = pos;gotRate = True
        return rate, positions
    
    def generateFilename(self, trial):
        return "TestMotorRateAccuracy_trial"+str(trial)

    def plot(self, xPositions, yPositions):#arrays will be in the form xPositions = [[1,2,5], [5,7,2,2,5], [7,2,5]...]. Same for yPositions
        ticks = []
        for i in range(len(xPositions)):
            ticks.append(str(i+1))
        plt.figure()
        
        rangeAndOffsetsForLeftBox = np.array(range(len(xPositions))) * 2+ 1
        rangeAndOffsetsForRightBox = np.array(range(len(yPositions))) * 2 + 1.7
        bpLeft = plt.boxplot(xPositions, positions = rangeAndOffsetsForLeftBox + 0.4, sym="")
        bpRight = plt.boxplot(yPositions, positions = rangeAndOffsetsForRightBox + 0.4, sym="")
        red = '#D7191C'; black = '#000000'; blue = '#2C7BB6' #http://colorbrewer2.org/ 
        self.changeBoxColor(bpLeft, red)
        self.changeBoxColor(bpRight, black)
        
        plt.plot([], c=red, label='dx'); plt.plot([], c=black, label='dy') #temporary lines for legend
        plt.legend()
        
        plt.xticks(rangeAndOffsetsForLeftBox , ticks)
        plt.tight_layout()
        plt.savefig('xyAccuracies.png')
        plt.show()
        
    def changeBoxColor(self, boxPlot, c):
        plt.setp(boxPlot['boxes'], color=c)
        plt.setp(boxPlot['whiskers'], color=c)
        plt.setp(boxPlot['caps'], color=c)
        plt.setp(boxPlot['medians'], color=c)

