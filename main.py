import sys

from WalkingRobot import RobotBody
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
        self.space.gravity = (0.0, -900.0)
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
            if not hasattr(body, 'startPosition'):
                continue
            body.position = Vec2d(body.startPosition)
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.startAngle

    def draw(self):        
        self.screen.fill(THECOLORS["white"])### Clear the screen        
        self.space.debug_draw(self.draw_options)### Draw space        
        pygame.display.flip()### All done, lets flip the display

    def main(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.display_size, self.display_flags)
        width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 200,200,200, 50
        

        def to_pygame(p):            
            return int(p.x), int(-p.y+height) #Small hack to convert pymunk to pygame coordinates
        def from_pygame(p):
            return to_pygame(p)

        clock = pygame.time.Clock()
        running = True
        font = pygame.font.Font(None, 16)

        # Create the spider
        chassisXY = Vec2d(self.display_size[0]/2, self.ground_y+100)
        robot1 = RobotBody(self.space, chassisXY)        
                    

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
#                 elif event.type == KEYDOWN and event.key == K_UP:
#                     motor_ba1Left.rate = rotationRate
#                 elif event.type == KEYDOWN and event.key == K_DOWN:
#                     motor_ba1Left.rate = -rotationRate
#                 elif event.type == KEYDOWN and event.key == K_LEFT:
#                     motor_ac1Left.rate = rotationRate
#                 elif event.type == KEYDOWN and event.key == K_RIGHT:
#                     motor_ac1Left.rate = -rotationRate                    
#                 elif event.type == KEYUP:
#                     motor_ba1Left.rate = 0
#                     motor_ac1Left.rate = 0

            for body in self.space.bodies:
                if not hasattr(body, 'startPosition'):
                    continue
                #body.position = Vec2d(body.startPosition)
                body.force = 0, 0
                body.torque = 0
                body.velocity = 0, 0
                body.angular_velocity = 0
                body.angle = body.startAngle
                #print(body.position)
                break
            #print('--------')
            
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