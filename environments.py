import pymunk
from pymunk import Vec2d

class TrainingEnvironments:
    boundaryObjects = []
    space = None  
    envX = None
    envY = None  
    envWidth = None
    envHeight = None
    wallThickness = 5

    def initializeTrainingBoundary(self, space, x, y, width, height):
        self.envX = x; self.envY = y; self.envWidth = width; self.envHeight = height; self.space = space
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC); body.position = Vec2d(self.envX, self.envY)    
        shape = pymunk.Poly.create_box(body, (self.envWidth, self.wallThickness)); shape.color = 170, 170, 170; shape.friction = 0.0
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