""" 
This file defines an environment for the bus simulator. 
"""
import Box2D as b2 
import numpy as np
import cv2
import pygame
from framework import Framework 
from gps.agent.box2d.settings import fwSettings
from gps.agent.box2d.traffic import Traffic 
from gps.proto.gps_pb2 import END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES
from gps.agent.box2d.cvx import *

STEERINGSCALE = 0.01
BUS_LENGTH = 6 # 5.
BUS_WIDTH = 3 # 1.

# For traffic map
# macros
DISP_SCALE = 1  #10          # display pixels per meter
SCREEN_BORDER = 0.3      # border to keep bus in the screen
STEERING_RATIO = 12      # steering wheel to steering angle ratio

SPEED_DECAY = 0.05       # bus speed decrease each timestep
COLL_SPEED = 0.1         # percentage speed remaining after collision

MAP_CLEARANCE = 20       # clearance between route and map borders (meters)
CAR_DIST_MAX = 250       # max distance between car and ego bus before respawning


SHOW_TILES = 20          # show the next 20 tiles
LANE_DEVIATION = 1       # random cars deviation from center of lane
OBS_SCALE = 1            # observation pixels per meter
OBS_CLEARANCE = 0.25     # clearance between bus and observation

MAX_EP_TIME = 100        # maximum no of seconds per episode
WAIT_TIME = 10           # maximum waiting time (seconds) for bus to cover another tile before terminating episode

IMAGES = ['red', 'orange', 'yellow', 'green', 'blue', 'violet', 'purple', 'aqua', 'magenta', 'white']

# state values
SELF = 255
UNCOVERED = 200
ROAD = 150
OUT = 50
OBSTACLE = 0

# pygame colors
BLACK = (0,0,0)
WHITE = (255,255,255)
GREEN = (0,255,0)
RED = (255,0,0)
BLUE = (0,0,255)
BACKGROUND = (50,50,50)
TEXT = (200,200,200)

STATE_W = 48
STATE_H = 48
SCREEN_W = 1000
SCREEN_H = 1000
THROTTLE_SCALE = 0.01    
STEERING_SCALE = 0.02    
STEER_ANGLE = 20        

GROUND_EDGE = 50
BOX_SCALE = 10

class BusWorld(Framework):
    """ This class defines the bus and its environment"""
    name = "Bus"
    def __init__(self, x0, target, render, map_size, polygons=None,map_state=None, route=None, display_center=None):
        # TODO: try to pass the initial point as shift so that it will not affect the original code
        self.render = render
        # print('map_size', map_size)
        map_width = map_size[0]
        map_height = map_size[1]
        if self.render:
            super(BusWorld, self).__init__()
        else: 
            self.world = b2.b2World(gravity=(0, -10), doSleep=True)
        self.world.gravity = (0.0, 0.0)

        # TODO: may need resizing later, map to screen
        
        # self.initial_position = (x0[0]-display_center[0], display_center[1]-x0[1])
        # self.target_position = (target[0]-display_center[0], display_center[1]-target[1])
        self.initial_position = (x0[0], x0[1])
        self.target_position = (target[0], target[1])
        self.initial_angle = x0[2]
        self.initial_linear_velocity = (x0[3], x0[4])
        self.initial_angular_velocity = x0[5]
        self.wheelbase = BUS_LENGTH
        self.lr = BUS_LENGTH/2

        # create a ground (the outer layer bounding box) based on the map_size 
        # ground = self.world.CreateBody(position=[0, 0])
        
        # ground.CreateEdgeChain(
        #     [(-map_width/2, -map_height/2),
        #      (-map_width/2, map_height/2),
        #      (map_width/2, map_height/2),
        #      (map_width/2, -map_height/2),
        #      (-map_width/2, -map_height/2),]
        #      )

        # Initialize the rectangular bus
        rectangle_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(BUS_LENGTH/2, BUS_WIDTH/2)),
            density=1.5,
            friction=1.,
        )
        small_rectangle_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(2,2)),
            density=1.5,
            friction=1.,
        )
        square_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(5, 5)),
            density=10,
            friction=5.,
        )

        target_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(BUS_LENGTH/2, BUS_WIDTH/2)),
            isSensor=True,
        )

        if route:
            for waypoint in route:
                print("roadmark", waypoint)
                # position = (body[0] - map_width/2, map_height - body[1] - map_height/2)
                position = (waypoint[0]- map_width/2, map_height - waypoint[1]- map_height/2)
                roadmark = self.world.CreateStaticBody(
                    position = position,
                    angle = waypoint[2],
                    fixtures= rectangle_fixture,
                )
                roadmark.active = False
                # TODO; for test_action, need to modify the coordinate system
                display_center = [map_size[1]/2, map_size[0]/2]

        self.body = self.world.CreateDynamicBody(
            position=self.initial_position,
            angle=self.initial_angle,
            linearVelocity=self.initial_linear_velocity,
            angularVelocity=self.initial_angular_velocity,
            fixtures=rectangle_fixture,
        )
        
        self.target = self.world.CreateStaticBody(
            position=self.target_position,
            angle=target[2],
            fixtures = target_fixture,
        )
        self.target.active = False
        self.reach = False

        if polygons is not None:
            # draw roads in box2d environment as polygons
            convex_polygons = []
            for poly in polygons:
                new_poly = []
                for item in poly:
                    # the poly has a redundant layer of square bracket, strip it off
                    new_item = [item[0][0], item[0][1]]
                    new_poly.append(new_item)

                convex_polys,img = to_convex(new_poly)
                # print(convex_polys)
                for convex_poly in convex_polys:
                    convex_polygon = []
                    for pt in convex_poly:
                        # use y = map_height - y to convert opencv_coord to map_coord
                        new_pt = (pt[0], map_height-pt[1])
                        convex_polygon.append(new_pt)
                    convex_polygons.append(convex_polygon)

            # # render the convex polygons
            # for convex_polygon in convex_polygons:
            #     vertices = []
            #     for pt in convex_polygon:
            #         # use float to avoid datatype error
            #         vertices.append(b2.b2Vec2(float(pt[0]), float(pt[1])))
            #     obstacle = self.world.CreateStaticBody(
            #         position=[-map_width/2,-map_height/2],
            #         # position=[0,0],
            #         angle=0, 
            #         shapes= b2.b2PolygonShape(vertices=vertices)
            #     )
    
    # (deprecated) introduce_roads is replaced by drawing polygons
    def introduce_roads():
        # deprecated
        # Introduce traffic map 
        self.traffic = Traffic()
        self.map_scale = OBS_SCALE   # map px per meters. set it to OBS_SCALE so no resizing necessary when getting observation

        contours = self.loadMap()
        num_contour = len(contours)
        print("num", num_contour)
        obstacles = []

        for contour in contours:
            vertices = []
            for item in contour:
                new_vec = b2.b2Vec2(float(item[0][0]/BOX_SCALE), float(item[0][1]/BOX_SCALE))
                vertices.append(new_vec)
            print("vertices")
            print(vertices)
            contour_shape = b2.b2PolygonShape(vertices=vertices)
            obstacle = self.world.CreateStaticBody(position=(0,0), shapes=contour_shape)
            obstacles.append(obstacle)

    def check_reach_target(self):
        # compare the coordinal difference of body and target
        diff0 = self.body.position[0] - self.target.position[0] 
        diff1 = self.body.position[1] - self.target.position[1]
        diff2 = self.body.angle - self.target.angle
        
        # return (abs(diff0*diff1) < (0.1 * BUS_LENGTH * BUS_WIDTH) and abs(diff2) < 0.05)
        # if (abs(diff0) < 0.1 * BUS_WIDTH) and (abs(diff1) < 0.1 * BUS_LENGTH) and (abs(diff2) < 0.05):
        # if (abs(diff0) < 2) and (abs(diff1) < 2) and (abs(diff2) < 0.3):  # 0.2 works till index 60, cannot reach index 61
        if (abs(diff0) < 2) and (abs(diff1) < 2): 
            # what if we don't check yaw at all.
            # print("diff", diff0, diff1, diff2)
            # print(self.body.position[0], self.target.position[0], self.body.position[1], self.target.position[1] )
            return True
        else:
            return False
        
    def run(self):
        """
        Initiate the first time step
        """
        if self.render:
            # use the implementation in pygame_framework.py
            # which runs SimulationLoop and flip the pygame display
            # the pygame_framework.SimulationLoop links to frameworkbase.step
            super(BusWorld, self).run()
        else:
            self.run_next(None)

    def run_next(self, action):
        """
        Move one step forward. Call the render if applicable
        """
        dt = 1.0/fwSettings.hz
        if self.render:
            super(BusWorld, self).run_next(action)
        else:
            if action is not None:
                #update the velocity based on bus dynamics
                vel_action = self.convert_action(action)
                self.body.linearVelocity = (vel_action[0], vel_action[1])
                self.body.angularVelocity = vel_action[2]
            #velocityIterations and positionIterations are the constraints to be considered
            self.world.Step(dt, fwSettings.velocityIterations,
                            fwSettings.positionIterations)
        self.reach = self.check_reach_target()

    def run_test(self, actions):
        self.run()
        for action in actions:
            self.run_next(action)
            # print(self.get_state())

    def convert_action(self, action):
        dt = 1.0/fwSettings.hz
        
        beta = np.arctan( self.lr*np.tan(action[1]) / self.wheelbase)
        speed = np.sqrt(self.body.linearVelocity[0]**2 + self.body.linearVelocity[1]**2) + action[0]*dt

        vel_action = [0., 0., 0.]
        vel_action[0] = speed * np.cos(self.body.angle+beta)
        vel_action[1] = speed * np.sin(self.body.angle+beta)
        vel_action[2] = speed * np.cos(beta) / self.wheelbase * np.tan(action[1])
        return vel_action

    def Step(self, settings, action):
        """
        Called upon every step
        update the agent states based on action 
        and call the world to update
        """
        dt = 1.0/fwSettings.hz
        vel_action = self.convert_action(action)
        self.body.linearVelocity = (vel_action[0], vel_action[1])  
        self.body.angularVelocity = vel_action[2]

        super(BusWorld, self).Step(settings)


    def reset_world(self):
        self.world.ClearForces()
        self.body.position = self.initial_position
        self.body.angle = self.initial_angle
        self.body.linearVelocity = self.initial_linear_velocity
        self.body.angularVelocity = self.initial_angular_velocity
    
    def get_state(self):
        # to return an angle within (-pi, pi)
        self.body.angle = np.arctan2(np.sin(self.body.angle), np.cos(self.body.angle))
        state = {END_EFFECTOR_POINTS: np.append(np.array(self.body.position), self.body.angle),
                 END_EFFECTOR_POINT_VELOCITIES: np.append(np.array(self.body.linearVelocity), self.body.angularVelocity)}

        return state
    
    def BeginContact(self, contact):
        # deprecated since it is not working well and checking with box2d collision/overlapping cannot be generalized to non-render cases
        # if contact.touching:
        #     fixtureA, fixtureB = contact.fixtures
        #     if fixtureA == target_fixture or fixtureB == target_fixture:
        #         print("fixture A B", fixtureA, fixtureB)
        #         self.reach = True
        pass
            
    def EndContact(self, contact):
        pass
    
    def PreSolve(self, contact, old_manifold):
        pass
                
    
    def PostSolve(self, contact, old_manifold):
        pass
