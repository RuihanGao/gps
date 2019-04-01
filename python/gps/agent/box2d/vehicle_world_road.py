""" 
This file defines an environment for the vehicle simulator. 
"""

import Box2D as b2 
import numpy as np
import cv2
import pygame
from framework import Framework 
from gps.agent.box2d.settings import fwSettings
from gps.agent.box2d.traffic import Traffic 
from gps.proto.gps_pb2 import END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES

# macros
# used 
# map and display
SCREEN_SCALE = 1  #10    # screen pixels per meter
OBS_SCALE = 1            # observation pixels per meter
MAP_SACLE = 1            # map pixels per meter
MAP_CLEARANCE = 20       # clearance between route and map borders (meters)
LANE_WIDTH = 8           # width of lane in meters
COVER_TILE = 5           # distance per tile of road to cover
TASK_DIST = 1000         # meters from starting point to goal
# bus
BUS_LENGTH = 5.
BUS_WIDTH = 1.
FRICTION = 1.
# state values
ROAD = 150
OUT = 50




# # # unused
# STEERINGSCALE = 0.01
# SCREEN_BORDER = 0.3      # border to keep vehicle in the screen
# STEERING_RATIO = 12      # steering wheel to steering angle ratio
# SPEED_DECAY = 0.05       # vehicle speed decrease each timestep
# COLL_SPEED = 0.1         # percentage speed remaining after collision
# CAR_DIST_MAX = 250       # max distance between car and ego vehicle before respawning
# SHOW_TILES = 20          # show the next 20 tiles
# LANE_DEVIATION = 1       # random cars deviation from center of lane
# OBS_CLEARANCE = 0.25     # clearance between vehicle and observation
# MAX_EP_TIME = 100        # maximum no of seconds per episode
# WAIT_TIME = 10           # maximum waiting time (seconds) for vehicle to cover another tile before terminating episode
# IMAGES = ['red', 'orange', 'yellow', 'green', 'blue', 'violet', 'purple', 'aqua', 'magenta', 'white']
# SELF = 255
# UNCOVERED = 200
# OBSTACLE = 0
# # pygame colors
# BLACK = (0,0,0)
# WHITE = (255,255,255)
# GREEN = (0,255,0)
# RED = (255,0,0)
# BLUE = (0,0,255)
# BACKGROUND = (50,50,50)
# TEXT = (200,200,200)
# STATE_W = 48
# STATE_H = 48
# SCREEN_W = 1000
# SCREEN_H = 1000
# THROTTLE_SCALE = 0.01    
# STEERING_SCALE = 0.02    
# STEER_ANGLE = 20        
# GROUND_EDGE = 50
# BOX_SCALE = 10


class VehicleWorld(Framework):
    """ This class defines the vehicle and its environment"""
    name = "Vehicle"
    def __init__(self, x0, target, render):
        # initialize traffic, load map and divide it into tiles
        self.traffic = Traffic()
        self.loadMap()
        # replace the x0 from hyperparameters by x0 from the divideRoute
        x0_route = self.divideRoute()
        x0[0] = x0_route[0]
        x0[1] = x0_route[1]
        x0[2] = x0_route[2]
        # TODO: try x0[0:3] = self.divideRoute() to see if it works
        print("x0", x0)

        self.render = render
        if self.render:
            super(VehicleWorld, self).__init__()
        else: 
            self.world = b2.b2World(gravity=(0, -10), doSleep=True)
        self.world.gravity = (0.0, 0.0)
        self.initial_position = (x0[0], x0[1])
        self.initial_angle = x0[2]
        self.initial_linear_velocity = (x0[3], x0[4])
        self.initial_angular_velocity = x0[5]
        # ?? how to assign the parameter setting to dynamic body itself?  
        self.wheelbase = BUS_LENGTH
        self.lr = BUS_LENGTH/2

        ground = self.world.CreateBody(position=(0,GROUND_EDGE)) # set the initial position of the body
        ground.CreateEdgeChain(
            [(-GROUND_EDGE, -GROUND_EDGE),
             (-GROUND_EDGE, GROUND_EDGE),
             (GROUND_EDGE, GROUND_EDGE),
             (GROUND_EDGE, -GROUND_EDGE),
             (-GROUND_EDGE, -GROUND_EDGE)]
             )
        # ground.CreateEdgeChain(
        #     [(0, 0), (0, GROUND_EDGE), (GROUND_EDGE, GROUND_EDGE), (GROUND_EDGE, 0),(0, 0)])

        # self.introduce_roads()

        # Initialize the rectangular bus
        rectangle_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(BUS_LENGTH*self.screen_scale/2, BUS_WIDTH*self.screen_scale/2)),
            density=1.5,
            friction=FRICTION,
        )
        square_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(0.5, 0.5)),
            density=10,
            friction=5.,
        )

        self.bus = self.world.CreateDynamicBody(
            position=self.initial_position,
            angle=self.initial_angle,
            linearVelocity=self.initial_linear_velocity,
            angularVelocity=self.initial_angular_velocity,
            fixtures=rectangle_fixture,
        )

        self.target = self.world.CreateStaticBody(
            position=target[:2],
            # angle=target[2],
            # linearVelocity=target[3:5],
            # angularVelocity=target[5],
            angle=self.initial_angle,
            linearVelocity=self.initial_linear_velocity,
            angularVelocity=self.initial_angular_velocity,
            fixtures = rectangle_fixture,
        )
        self.target.active = False
        # self.introduce_obstacles()
    
    def introduce_obstacles():
        self.obstacle1 = self.world.CreateStaticBody(
            position = [16, 15],
            angle=0,
            fixtures=square_fixture
        )

        self.obstacle2 = self.world.CreateStaticBody(
            position = [13, 10],
            angle=0,
            fixtures=square_fixture
        )

        self.obstacle3 = self.world.CreateStaticBody(
            position = [15, 25],
            angle=0,
            fixtures=square_fixture
        )
    
    def introduce_roads():
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

    def run(self):
        """
        Initiate the first time step
        """
        if self.render:
            # use the implementation in pygame_framework.py
            # which runs SimulationLoop and flip the pygame display
            # the pygame_framework.SimulationLoop links to frameworkbase.step
            super(VehicleWorld, self).run()
        else:
            self.run_next(None)

    def run_next(self, action):
        """
        Move one step forward. Call the render if applicable
        """
        dt = 1.0/fwSettings.hz
        if self.render:
            super(VehicleWorld, self).run_next(action)
        else:
            if action is not None:
                #update the velocity based on vehicle dynamics
                vel_action = self.convert_action(action)
                self.body.linearVelocity = (vel_action[0], vel_action[1])
                self.body.angularVelocity = vel_action[2]
                # # update position/state
                # pos = self.pos.copy()
                # pos[0] += self.vel[0] * dt
                # pos[1] += self.vel[1] * dt
                # pos[2] += self.vel[2] * dt
            #TODO: figure out what velocityIterations and positionIterations represent
            self.world.Step(dt, fwSettings.velocityIterations,
                            fwSettings.positionIterations)

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
        #?? where to update the states [x, y, yaw],  i.e. where to put dynamics
        dt = 1.0/fwSettings.hz
        # self.body.linearVelocity += action[0]*dt
        # ?? relationship between step and run_next?
        # beta = np.arctan( self.lr*np.tan(action[1]) / self.wheelbase)
        # speed = np.sqrt(self.body.linearVelocity[0]**2 + self.body.linearVelocity[1]**2)
        # self.body.linearVelocity[0] = speed * np.cos(self.body.angle+beta)
        # self.body.linearVelocity[1] = speed * np.sin(self.body.angle+beta)
        # self.body.angularVelocity = speed * np.cos(beta) / self.wheelbase * np.tan(action[1])      
        vel_action = self.convert_action(action)
        self.body.linearVelocity = (vel_action[0], vel_action[1])  
        self.body.angularVelocity = vel_action[2]

        super(VehicleWorld, self).Step(settings)

    def reset_world(self):
        self.world.ClearForces()
        # Introduce traffic map
        self.loadMap()
        self.displayMap()

        self.body.position = self.initial_position
        self.body.angle = self.initial_angle
        self.body.linearVelocity = self.initial_linear_velocity
        self.body.angularVelocity = self.initial_angular_velocity
    

    def get_state(self):
        state = {END_EFFECTOR_POINTS: np.append(np.array(self.body.position), self.body.angle),
                 END_EFFECTOR_POINT_VELOCITIES: np.append(np.array(self.body.linearVelocity), self.body.angularVelocity)}

        return state



    # Private Methods for transformations #
    def _utmToIdx(self, utm):
        i = self.map_size[0]-1-(utm[1]-self.map_borders[2])*self.map_scale
        j = (utm[0]-self.map_borders[0])*self.map_scale
        return ( int(i), int(j) )
    
    def _idxToUtm(self, idx):
        x = idx[1] / self.map_scale + self.map_borders[0]
        y = ( self.map_size[0]-1 - idx[0] ) / self.map_scale + self.map_borders[2]
        return ( x, y )
    
    def _utmToCoords(self, utm):
        x = utm[0] - self.map_borders[0] - self.origin_on_map[0]/self.map_scale
        y = utm[1] - self.map_borders[2] - (self.map_size[0]-1-self.origin_on_map[1])/self.map_scale
        return ( x, y )
    
    def _coordsToUtm(self, coords):
        x = coords[0] + self.origin_on_map[0]/self.map_scale + self.map_borders[0]
        y = coords[1] + (self.map_size[0]-1-self.origin_on_map[1])/self.map_scale + self.map_borders[2]
        return ( x, y )

    def _coordsToIdx(self, coords):
        j = self.origin_on_map[0] + coords[0]*self.map_scale
        i = self.origin_on_map[1] - coords[1]*self.map_scale
        # clip to ensure within map
        if i < 0 or i >= self.map_size[0] or j < 0 or j >= self.map_size[1]:
            i = min(max(i,0),self.map_size[0]-1)
            j = min(max(j,0),self.map_size[1]-1)
        return ( int(i), int(j) )

    def _idxToCoords(self, idx):
        # modified by ruihan, include the yaw information
        x = (idx[1] - self.origin_on_map[0]) / self.map_scale
        y = (idx[0] - self.origin_on_map[1]) / self.map_scale * -1
        return ( x, y)

    def _idxToScreen(self, idx):
        x = ( idx[1] - self.origin_on_map[0] - self.screen_center[0] + self.scaled_screen_size[0]/2 ) * self.screen_scale
        y = ( idx[0] - self.origin_on_map[1] - self.screen_center[1] + self.scaled_screen_size[1]/2 ) * self.screen_scale
        return ( int(x), int(y) )

    def _coordsToScreen(self, coords):
        x = ( coords[0]    * self.map_scale - self.screen_center[0] + self.scaled_screen_size[0]/2 ) * self.screen_scale
        y = ( coords[1]*-1 * self.map_scale - self.screen_center[1] + self.scaled_screen_size[1]/2 ) * self.screen_scale
        return ( int(x), int(y) )
    
    def _screenToCoords(self, screen):
        x = ( screen[0] / self.screen_scale - self.scaled_screen_size[0]/2 + self.screen_center[0] ) / self.map_scale
        y = ( screen[1] / self.screen_scale - self.scaled_screen_size[1]/2 + self.screen_center[0] ) / self.map_scale * -1
        return ( x, y )

    def _coordsWithinMap(self, coords):
        j = self.origin_on_map[0] + coords[0]*self.map_scale
        i = self.origin_on_map[1] - coords[1]*self.map_scale
        if i < 0 or i >= self.map_size[0] or j < 0 or j >= self.map_size[1]: return False
        else: return True
    
    def _setScreenCenter(self, coords):
        self.screen_center = [ coords[0]*self.map_scale , -1*coords[1]*self.map_scale ]
        self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], self.origin_on_map[1] + self.screen_center[1])

    def _updateScreenCenter(self, coords):
        pos_on_screen = self._coordsToScreen(coords)
        if pos_on_screen[0] < SCREEN_BORDER*self.screen_size[0]:
            self.screen_center[0] -= ( SCREEN_BORDER*self.screen_size[0] - pos_on_screen[0] ) / self.screen_scale
        elif pos_on_screen[0] > (1-SCREEN_BORDER)*self.screen_size[0]:
            self.screen_center[0] += ( pos_on_screen[0] - (1-SCREEN_BORDER)*self.screen_size[0] ) / self.screen_scale
        if pos_on_screen[1] < SCREEN_BORDER*self.screen_size[1]:
            self.screen_center[1] -= ( SCREEN_BORDER*self.screen_size[1] - pos_on_screen[1] ) / self.screen_scale
        elif pos_on_screen[1] > (1-SCREEN_BORDER)*self.screen_size[1]:

            self.screen_center[1] += ( pos_on_screen[1] - (1-SCREEN_BORDER)*self.screen_size[1] ) / self.screen_scale
        self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], self.origin_on_map[1] + self.screen_center[1])
    
# Calculation Functions #
def _angleDiff(th1, th2):
    diff = th2 - th1
    return np.arctan2( np.sin(diff), np.cos(diff) )

def _euclidDist(pos1, pos2):
    diff = np.array(pos1[0:2]) - np.array(pos2[0:2])
    return np.linalg.norm(diff)

def _intersectionPoint(vec1, vec2):
    # if vec1[2] == vec2[2]: return float('Inf')
    a = np.array( [ [np.sin(vec1[2]), -np.cos(vec1[2])], [np.sin(vec2[2]), -np.cos(vec2[2])] ] )
    b = np.array( [ vec1[0]*np.sin(vec1[2]) - vec1[1]*np.cos(vec1[2]), vec2[0]*np.sin(vec2[2]) - vec2[1]*np.cos(vec2[2]) ] )
    x, y = np.linalg.solve(a,b)
    return (x, y)

