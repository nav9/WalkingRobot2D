# Author: Navin Ipe
# Created: July 2020
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import os
import pickle

class FileOperations:
    def __init__(self):
        pass
    
    def savePickleFile(self, directory, filename, data):
        direc = self.dataDir + directory 
        self.createDirectoryIfNotExisting(direc)
        with open(os.path.join(direc, filename), 'wb') as handle:
            pickle.dump(data, handle, protocol = pickle.HIGHEST_PROTOCOL)
            print('Saved ',direc, filename)
        
    def loadPickleFile(self, directory, filename):
        data = None #data will be loaded in exactly the same format it was stored in
        try:
            direc = self.dataDir + directory
            self.createDirectoryIfNotExisting(direc) 
            with open(os.path.join(direc, filename), 'rb') as handle:
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
    
    def generateFilename(self, trial, sim):
        return "TestMotorRateAccuracy_trial"+str(trial)+"_sim"+str(sim) 

