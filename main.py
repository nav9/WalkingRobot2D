import sys

from WalkingRobot import RobotBody
import pygame
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT
from pygame.color import THECOLORS

import pymunk
from pymunk import Vec2d
import pymunk.pygame_util
from IPython.nbformat import current

class World:
    def __init__(self, space):
        # Pymunk physics coordinates start from the lower right-hand corner of the screen.
        groundX = 10; groundLen = 600
        groundY = 200#increasing this value makes it ground get positioned higher
        ground = pymunk.Segment(space.static_body, (groundX, groundY), (groundX+groundLen, groundY), 1.0)
        ground.friction = 1.0
        space.add(ground)        
        

class Simulator(object):
    focusRobotXY = Vec2d(0,0)#will be overridden below
    robots = []
    numRobots = 5

    def __init__(self):
        self.screenWidth = 800; self.screenHeight = 640
        self.display_flags = 0
        self.minViewX = 50
        self.maxViewX = self.screenWidth - self.minViewX
        self.minViewY = 100
        self.maxViewY = self.screenHeight - 50

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.fps = 50
        self.iterations = 10        
        #self.space.damping = 0.999 

#         # Pymunk physics coordinates start from the lower right-hand corner of the screen.
#         self.ground_y = 200#increasing this value makes it ground get positioned higher
#         ground = pymunk.Segment(self.space.static_body, (5, self.ground_y), (595, self.ground_y), 1.0)
#         ground.friction = 1.0
#         self.space.add(ground)

        self.world = World(self.space)
        self.screen = None
        self.draw_options = None
        self.focusRobotID = 0#the first robot created will be the focus robot. ie: The screen moves with this robot. Focus robot id can be changed dynamically

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

    def getUpdateBy(self, currentXY):
        updateBy = Vec2d(0,0)
        if currentXY[0] < self.minViewX or currentXY[0] > self.maxViewX or currentXY[1] < self.minViewY or currentXY[1] > self.maxViewY:
            updateBy = -1 * Vec2d(currentXY - self.focusRobotXY) 
        return updateBy
    
    def main(self):
        pygame.init()
        pygame.mixer.quit()#disable sound output that causes annoying sound effects if any other external music player is playing
        self.screen = pygame.display.set_mode((self.screenWidth, self.screenHeight), self.display_flags)
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

        # Create the spider robot
        self.focusRobotXY = Vec2d(self.screenWidth/2, self.screenHeight/2)
        for i in range(1,self.numRobots,1):
            self.robots.append(RobotBody(self.space, self.focusRobotXY))
            
        
        #simulate = False
        rotationRate = 2
        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    #running = False
                    sys.exit(0)
#                 elif event.type == KEYDOWN and event.key == K_s:
#                     # Start/stop simulation.
#                     simulate = not simulate
#                 elif event.type == KEYDOWN and event.key == K_r:
#                     # Reset.
#                     # simulate = False
#                     self.reset_bodies()
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

#             for body in self.space.bodies:
#                 if not hasattr(body, 'startPosition'):
#                     continue
#                 #body.position = Vec2d(body.startPosition)
#                 body.force = 0, 0
#                 body.torque = 0
#                 body.velocity = 0, 0
#                 body.angular_velocity = 0
#                 body.angle = body.startAngle
#                 #print(body.position)
#                 break

            ### Update physics
            dt = 1.0/float(self.fps)/float(self.iterations)
            for x in range(self.iterations): # 10 iterations to get a more stable simulation
                self.space.step(dt)
            updateBy = self.getUpdateBy(self.robots[self.focusRobotID].chassis_body.position)
            if updateBy != (0, 0):
                self.draw()
            else:
                self.draw()
            clock.tick(self.fps)

if __name__ == '__main__':
    sim = Simulator()
    sim.main()