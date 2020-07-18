# Author: Navin Ipe
# Created: July 2020
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author

import os
import pickle

class ResultStore:
    def __init__(self):
        self.bestFitness = [] #appended to at the end of each epoch
        self.timeTaken = [] #appended to when the robot crosses the finish line

    def saveDataToDisk(self, directory, filename):
        self.__savePickleFile__(directory, filename, self.bestFitness)
        self.__savePickleFile__(directory, filename, self.timeTaken)
    
    def loadDataFromDisk(self, directory, filename):
        self.bestFitness = self.__loadPickleFile__(directory, filename)
        self.timeTaken = self.__loadPickleFile__(directory, filename)
        return self.bestFitness, self.timeTaken
        
    def __savePickleFile__(self, directory, filename, data):
        direc = self.dataDir + directory 
        self.createDirectoryIfNotExisting(direc)
        with open(os.path.join(direc, filename), 'wb') as handle:
            pickle.dump(data, handle, protocol = pickle.HIGHEST_PROTOCOL)
            print('Saved ',direc,filename)
        
    def __loadPickleFile__(self, directory, filename):
        data = None
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
            
class Analytics:
    pass


