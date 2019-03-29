import sys

import pygame
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT
from pygame.color import THECOLORS

import pymunk
from pymunk import Vec2d
import pymunk.pygame_util

class Simulator(object):

    def __init__(self):
        self.display_flags = 0
        self.display_size = (600, 600)

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        #self.space.damping = 0.999 # to prevent it from blowing up.

        # Pymunk physics coordinates start from the lower right-hand corner of the screen.
        self.ground_y = 100
        ground = pymunk.Segment(self.space.static_body, (5, self.ground_y), (595, self.ground_y), 1.0)
        ground.friction = 1.0
        self.space.add(ground)

        self.screen = None

        self.draw_options = None

    def reset_bodies(self):
        for body in self.space.bodies:
            if not hasattr(body, 'start_position'):
                continue
            body.position = Vec2d(body.start_position)
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.start_angle

    def draw(self):        
        self.screen.fill(THECOLORS["white"])### Clear the screen        
        self.space.debug_draw(self.draw_options)### Draw space        
        pygame.display.flip()### All done, lets flip the display

    def main(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.display_size, self.display_flags)
        width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)

        def to_pygame(p):            
            return int(p.x), int(-p.y+height) #Small hack to convert pymunk to pygame coordinates
        def from_pygame(p):
            return to_pygame(p)

        clock = pygame.time.Clock()
        running = True
        font = pygame.font.Font(None, 16)

        # Create the spider
        chassisXY = Vec2d(self.display_size[0]/2, self.ground_y+100)
        chWd = 70; chHt = 50
        chassisMass = 10
        
        legWd_a = 50; legHt_a = 5
        legWd_b = 100; legHt_b = 5
        legMass = 1
        relativeAnguVel = 0

        #---chassis
        chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        chassis_b.position = chassisXY
        chassis_shape = pymunk.Poly.create_box(chassis_b, (chWd, chHt))
        chassis_shape.color = 200, 200, 200
        print("chassis position");print(chassis_b.position)

        #---first left leg a
        leftLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        leftLeg_1a_body.position = chassisXY - ((chWd/2)+(legWd_a/2), 0)
        leftLeg_1a_shape = pymunk.Poly.create_box(leftLeg_1a_body, (legWd_a, legHt_a))        
        leftLeg_1a_shape.color = 255, 0, 0
        
        #---first left leg b
        leftLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        leftLeg_1b_body.position = leftLeg_1a_body.position - ((legWd_a/2)+(legWd_b/2), 0)
        leftLeg_1b_shape = pymunk.Poly.create_box(leftLeg_1b_body, (legWd_b, legHt_b))        
        leftLeg_1b_shape.color = 0, 255, 0        
        
        #---first right leg a
        rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        rightLeg_1a_body.position = chassisXY + ((chWd/2)+(legWd_a/2), 0)
        rightLeg_1a_shape = pymunk.Poly.create_box(rightLeg_1a_body, (legWd_a, legHt_a))        
        rightLeg_1a_shape.color = 255, 0, 0        
        
        #---first right leg b
        rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        rightLeg_1b_body.position = rightLeg_1a_body.position + ((legWd_a/2)+(legWd_b/2), 0)
        rightLeg_1b_shape = pymunk.Poly.create_box(rightLeg_1b_body, (legWd_b, legHt_b))        
        rightLeg_1b_shape.color = 0, 255, 0     
        
        #---link left leg b with left leg a       
        pj_ba1left = pymunk.PinJoint(leftLeg_1b_body, leftLeg_1a_body, (legWd_b/2,0), (-legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        motor_ba1Left = pymunk.SimpleMotor(leftLeg_1b_body, leftLeg_1a_body, relativeAnguVel)
        #---link left leg a with chassis
        pj_ac1left = pymunk.PinJoint(leftLeg_1a_body, chassis_b, (legWd_a/2,0), (-chWd/2, 0))
        motor_ac1Left = pymunk.SimpleMotor(leftLeg_1a_body, chassis_b, relativeAnguVel)
        #---link right leg b with right leg a       
        pj_ba1Right = pymunk.PinJoint(rightLeg_1b_body, rightLeg_1a_body, (-legWd_b/2,0), (legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        motor_ba1Right = pymunk.SimpleMotor(rightLeg_1b_body, rightLeg_1a_body, relativeAnguVel)
        #---link right leg a with chassis
        pj_ac1Right = pymunk.PinJoint(rightLeg_1a_body, chassis_b, (-legWd_a/2,0), (chWd/2, 0))
        motor_ac1Right = pymunk.SimpleMotor(rightLeg_1a_body, chassis_b, relativeAnguVel)              
        
        self.space.add(chassis_b, chassis_shape) 
        self.space.add(leftLeg_1a_body, leftLeg_1a_shape, rightLeg_1a_body, rightLeg_1a_shape) 
        self.space.add(leftLeg_1b_body, leftLeg_1b_shape, rightLeg_1b_body, rightLeg_1b_shape) 
        self.space.add(pj_ba1left, motor_ba1Left, pj_ac1left, motor_ac1Left)  
        self.space.add(pj_ba1Right, motor_ba1Right, pj_ac1Right, motor_ac1Right)      
        
        #---prevent collisions with ShapeFilter
        shape_filter = pymunk.ShapeFilter(group=1)
        chassis_shape.filter = shape_filter
        leftLeg_1a_shape.filter = shape_filter
        rightLeg_1a_shape.filter = shape_filter
        leftLeg_1b_shape.filter = shape_filter
        rightLeg_1b_shape.filter = shape_filter        
                    

        simulate = False
        rotationRate = 2
        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    #running = False
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_s:
                    # Start/stop simulation.
                    simulate = not simulate
                elif event.type == KEYDOWN and event.key == K_r:
                    # Reset.
                    # simulate = False
                    self.reset_bodies()
                elif event.type == KEYDOWN and event.key == K_UP:
                    motor_ba1Left.rate = rotationRate
                elif event.type == KEYDOWN and event.key == K_DOWN:
                    motor_ba1Left.rate = -rotationRate
                elif event.type == KEYDOWN and event.key == K_LEFT:
                    motor_ac1Left.rate = rotationRate
                elif event.type == KEYDOWN and event.key == K_RIGHT:
                    motor_ac1Left.rate = -rotationRate                    
                elif event.type == KEYUP:
                    motor_ba1Left.rate = 0
                    motor_ac1Left.rate = 0

            self.draw()

            ### Update physics
            fps = 50
            iterations = 25
            dt = 1.0/float(fps)/float(iterations)
            if simulate:
                for x in range(iterations): # 10 iterations to get a more stable simulation
                    self.space.step(dt)

            pygame.display.flip()
            clock.tick(fps)

if __name__ == '__main__':
    sim = Simulator()
    sim.main()