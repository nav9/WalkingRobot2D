import pymunk
from pymunk import Vec2d


class Directions:
    UP = 1
    DOWN = 2
    LEFT = 3    
    RIGHT = 4


class RobotLeg:
    space = pymunk.Space()
    shapeFilter = []
    ori = Directions()
    legOrientation = ori.LEFT  # default. Will be overridden in ctor. Whether the leg is at the right of the chassis or the left
    chassisXY = 0
    chWd = 0  # chassis width
    chHt = 0  # chassis Height
    legWd_a = 50
    legHt_a = 5
    legWd_b = 100
    legHt_b = 5
    legMass = 1
    relativeAnguVel = 0
    legA_body = []
    legA_shape = []
    legB_body = []
    legB_shape = []
    pinJ_LegBLegA = []
    motor_LegBLegA = []
    pinJ_LegAChassis = []
    motor_LegAChassis = []      
    
    def __init__(self, pymunkSpace, ownBodyShapeFilter, chassisBody, chassisCenterPosition, chassisWidth, chassisHeight, leftOrRight):
        self.space = pymunkSpace
        self.shapeFilter = ownBodyShapeFilter
        self.chassisXY = chassisCenterPosition
        self.chWd = chassisWidth
        self.chHt = chassisHeight
        self.legOrientation = leftOrRight
        self.__createLegA__()
        self.__createLegB__()
        self.__linkLegsAndChassis__(chassisBody)
    
    def __createLegA__(self):
        self.legA_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd_a, self.legHt_a)))
        if self.legOrientation == self.ori.LEFT:
            self.legA_body.position = self.chassisXY - ((self.chWd / 2) + (self.legWd_a / 2), 0)
        if self.legOrientation == self.ori.RIGHT:
            self.legA_body.position = self.chassisXY + ((self.chWd / 2) + (self.legWd_a / 2), 0)
        self.legA_shape = pymunk.Poly.create_box(self.legA_body, (self.legWd_a, self.legHt_a))        
        self.legA_shape.filter = self.shapeFilter
        self.legA_shape.color = 255, 0, 0   
        self.space.add(self.legA_body, self.legA_shape)
        
    def __createLegB__(self):
        self.legB_body = pymunk.Body(self.legMass, pymunk.moment_for_box(self.legMass, (self.legWd_b, self.legHt_b)))
        if self.legOrientation == self.ori.LEFT:
            self.legB_body.position = self.legA_body.position - ((self.legWd_a / 2) + (self.legWd_b / 2), 0)
        if self.legOrientation == self.ori.RIGHT:
            self.legB_body.position = self.legA_body.position + ((self.legWd_a / 2) + (self.legWd_b / 2), 0)
        self.legB_shape = pymunk.Poly.create_box(self.legB_body, (self.legWd_b, self.legHt_b))
        self.legB_shape.filter = self.shapeFilter        
        self.legB_shape.color = 0, 255, 0  
        self.space.add(self.legB_body, self.legB_shape)
    
    def __linkLegsAndChassis__(self, chassisBody):
        #---link left leg B with left leg A       
        if self.legOrientation == self.ori.LEFT:
            self.pinJ_LegBLegA = pymunk.PinJoint(self.legB_body, self.legA_body, (self.legWd_b / 2, 0), (-self.legWd_a / 2, 0))  # anchor point coordinates are wrt the body; not the space
        if self.legOrientation == self.ori.RIGHT:
            self.pinJ_LegBLegA = pymunk.PinJoint(self.legB_body, self.legA_body, (-self.legWd_b / 2, 0), (self.legWd_a / 2, 0))  # anchor point coordinates are wrt the body; not the space                        
        self.motor_LegBLegA = pymunk.SimpleMotor(self.legB_body, self.legA_body, self.relativeAnguVel)
        self.space.add(self.pinJ_LegBLegA, self.motor_LegBLegA)
        self.motor_LegBLegA.rate = 2
        #---link left leg A with Chassis
        if self.legOrientation == self.ori.LEFT:
            self.pinJ_LegAChassis = pymunk.PinJoint(self.legA_body, chassisBody, (self.legWd_a / 2, 0), (-self.chWd / 2, 0))
        if self.legOrientation == self.ori.RIGHT:
            self.pinJ_LegAChassis = pymunk.PinJoint(self.legA_body, chassisBody, (-self.legWd_a / 2, 0), (self.chWd / 2, 0))            
        self.motor_LegAChassis = pymunk.SimpleMotor(self.legA_body, chassisBody, self.relativeAnguVel)        
        self.space.add(self.pinJ_LegAChassis, self.motor_LegAChassis)
        self.motor_LegAChassis.rate = 2
    
class RobotBody:
    space = []
    ownBodyShapeFilter = pymunk.ShapeFilter(group=1)#to prevent collisions between robot body parts
    ori = Directions()  # leg orientation
    chassisXY = Vec2d(0, 0)  # Vec2d(self.display_size[0]/2, self.ground_y+100)
    chWd = 70; chHt = 50  # chassis width and height
    chassisMass = 10
    chassis_b = []  # chassis body
    chassis_shape = []  # chassis shape
    legs = []

    def __init__(self, pymunkSpace, chassisCenterPoint):
        self.space = pymunkSpace
        self.chassisXY = chassisCenterPoint
        self.__createBody__()
        
    def __createBody__(self):
        self.chassis_b = pymunk.Body(self.chassisMass, pymunk.moment_for_box(self.chassisMass, (self.chWd, self.chHt)))
        self.chassis_b.position = self.chassisXY
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_b, (self.chWd, self.chHt))
        self.chassis_shape.filter = self.ownBodyShapeFilter
        self.chassis_shape.color = 200, 200, 200
        self.space.add(self.chassis_b, self.chassis_shape)
        # print("chassis position");print(self.chassis_b.position)
        self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_b, self.chassisXY, self.chWd, self.chHt, self.ori.LEFT))
        self.legs.append(RobotLeg(self.space, self.ownBodyShapeFilter, self.chassis_b, self.chassisXY, self.chWd, self.chHt, self.ori.RIGHT))
        
