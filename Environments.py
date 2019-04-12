# Author: Navin Ipe
# Created: April 2019
# License: Proprietary. No part of this code may be copied or used in any form without the permission of the author
import pymunk
from pymunk import Vec2d

class TrainingEnvironments:
    boundaryObjects = []
    space = None  
    envX = 0
    envY = 0  
    envWidth = 1000
    envHeight = 300
    wallThickness = 5
    boundaryColor = 170,170,170

    def initializeTrainingBoundary(self, space):
        self.space = space
        #---top boundary        
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX, self.envY+self.envHeight)    
        shape = pymunk.Poly.create_box(body, (self.envWidth, self.wallThickness)); shape.color = self.boundaryColor; shape.friction = 0.0
        self.space.add(shape); self.boundaryObjects.append(shape);
        #---bottom boundary
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX, self.envY+self.wallThickness)    
        shape = pymunk.Poly.create_box(body, (self.envWidth, self.wallThickness)); shape.color = self.boundaryColor; shape.friction = 0.0
        self.space.add(shape); self.boundaryObjects.append(shape);
    def deleteTrainingBoundary(self):
        for ob in self.boundaryObjects:
            self.space.remove(ob)
        self.boundaryObjects[:] = []#clear the list
    def updatePosition(self, cameraXY):
        for ob in self.boundaryObjects:
            ob.body.position += cameraXY        
        
class TrainingWorld1(TrainingEnvironments):#inherits
    groundObjects = []
    
#     def __init__(self):
#         TrainingEnvironments.__init__(self)
    def initializeTrainingObjects(self):        
        elevFromBottomWall = 50; groundX = self.envX+self.wallThickness; groundLen = self.envWidth-2*self.wallThickness; groundY = elevFromBottomWall
        ground_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); groundStart = Vec2d(groundX, groundY); groundPosition = Vec2d(groundX+groundLen, groundY)
        ground_shape = pymunk.Segment(ground_body, groundStart, groundPosition, 1.0); ground_shape.friction = 1.0        
        self.space.add(ground_shape); self.groundObjects.append(ground_shape)                
    def deleteTrainingObjects(self):
        for ob in self.groundObjects:
            self.space.remove(ob)
        self.groundObjects[:] = []        
    def updatePosition(self, cameraXY):
        for ob in self.boundaryObjects:
            ob.body.position += cameraXY
        for ob in self.groundObjects:
            ob.body.position += cameraXY            