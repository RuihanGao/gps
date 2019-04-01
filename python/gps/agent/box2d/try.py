import sys
# Add the ptdraft folder path to the sys.path list
sys.path.append('/home/ruihan/gps/')

import Box2D as b2 
import numpy as np
from numpy import float32
import cv2
import pygame
# from framework import Framework 
# from gps.agent.box2d.settings import fwSettings
# from gps.agent.box2d.traffic import Traffic
# from traffic import Traffic
# from gps.proto.gps_pb2 import END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES

STEERINGSCALE = 0.01
BUS_LENGTH = 5.
BUS_WIDTH = 1.

# For traffic map
# macros
SCREEN_SCALE = 10        # display pixels per meter, previously DISPLAY_SCALE
SCREEN_BORDER = 0.3      # border to keep vehicle in the screen
STEERING_RATIO = 12      # steering wheel to steering angle ratio

SPEED_DECAY = 0.05       # vehicle speed decrease each timestep
COLL_SPEED = 0.1         # percentage speed remaining after collision

MAP_CLEARANCE = 20       # clearance between route and map borders (meters)
CAR_DIST_MAX = 250       # max distance between car and ego vehicle before respawning

TASK_DIST = 1000         # meters from starting point to goal
COVER_TILE = 5           # distance per tile of road to cover
SHOW_TILES = 20          # show the next 20 tiles
LANE_WIDTH = 8           # width of lane in meters
LANE_DEVIATION = 1       # random cars deviation from center of lane
OBS_SCALE = 1            # observation pixels per meter
OBS_CLEARANCE = 0.25     # clearance between vehicle and observation

MAX_EP_TIME = 100        # maximum no of seconds per episode
WAIT_TIME = 10           # maximum waiting time (seconds) for vehicle to cover another tile before terminating episode

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

GROUND_EDGE = 20
OBS_SCALE = 1




class World(object):
    def __init__(self):
        # initiate a 2dbox world
        self.world = b2.b2World(gravity=(0, -10), doSleep=True)
        # self.world.contactListener = self
        # # Introduce traffic map 
        
        # self.map_scale = OBS_SCALE   # map px per meters. set it to OBS_SCALE so no resizing necessary when getting observation

        # contours = self.loadMap()
        # num_contour = len(contours)
        # print("num", num_contour)
        # obstacles = []

        # for contour in contours:
        #     vertices = []
        #     for item in contour:
        #         new_vec = b2.b2Vec2(float(item[0][0]), float(item[0][1]))
        #         vertices.append(new_vec)
        #     print("vertices")
        #     print(vertices)
        #     contour_shape = b2.b2PolygonShape(vertices=vertices)
        #     obstacle = self.world.CreateStaticBody(position=(0,0), shapes=contour_shape)
        #     obstacles.append(obstacle)

        ground = self.world.CreateBody(position=(0,0))
        ground.CreateEdgeChain(
            [(-GROUND_EDGE, -GROUND_EDGE),
             (-GROUND_EDGE, GROUND_EDGE),
             (GROUND_EDGE, GROUND_EDGE),
             (GROUND_EDGE, -GROUND_EDGE),
             (-GROUND_EDGE, -GROUND_EDGE)]
             )

        '''s
        contours is a Python list of all the contours in the image. 
        Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
        '''

        # self.loadMap()
        # # get route in coordinates from starting position to goal, divide it to tiles
        # self.route = []
        # for i in range(len(self.env.route)-1):
        #     pt0 = self.env._idxToCoords( self.env.route[i] )
        #     pt1 = self.env._idxToCoords( self.env.route[i+1] )
        #     dist = _euclidDist(pt0, pt1)
        #     for tile_dist in np.arange(0, dist, COVER_TILE):
        #         pos = [ pt0[0] + tile_dist*np.cos(self.env.route[i][2]) , pt0[1] + tile_dist*np.sin(self.env.route[i][2]) , self.env.route[i][2] ]
        #         self.route.append( pos )
        #     self.route.append( [ pt1[0], pt1[1], self.env.route[i][2] ] )

        # # put vehicle in starting position
        # self.pos = [ self.route[0][0] + LANE_WIDTH/2*np.cos(self.route[0][2]+np.pi/2) , self.route[0][1] + LANE_WIDTH/2*np.sin(self.route[0][2]+np.pi/2) , self.route[0][2] ]

        # # delete first tile since it will always be covered at the start
        # self.route.pop(0)

        # # update center of screen
        # if self.env.render:
        #     self.env._setScreenCenter(coords=self.pos[0:2])


    
    # def reset(self):
    #     # Introduce traffic map
    #     self.loadMap()
    #     # self.displayMap()
        

    # # what I want to do: given route points by randomRoute, use cv.line to draw road with thinkness
    # # get the polygon representation of the road, and draw on 2D box canvas


    # ##  Introduce traffic map
    # def loadMap_new(self):
    #     # get random route
    #     route = self.traffic.randomRoute(dist=TASK_DIST, np_random=np.random.RandomState(seed=123))
    #     # crop map
    #     self.map_borders = [
    #         np.amin(np.array(route)[:,0]) - MAP_CLEARANCE,
    #         np.amax(np.array(route)[:,0]) + MAP_CLEARANCE,
    #         np.amin(np.array(route)[:,1]) - MAP_CLEARANCE,
    #         np.amax(np.array(route)[:,1]) + MAP_CLEARANCE
    #     ]
    #     map_center = ( (self.map_borders[0]+self.map_borders[1])/2, (self.map_borders[2]+self.map_borders[3])/2 )
    #     radius = max( map_center[0] - self.map_borders[0], map_center[1] - self.map_borders[2] )
    #     # self.traffic.cropMap(center=map_center, radius=radius)
    #     # uncomment above to show all roads instead of only the ones going to be travelled through by agent

    #     # create image
    #     self.img_width = int(self.map_scale*(self.map_borders[1]-self.map_borders[0]))
    #     self.img_height = int(self.map_scale*(self.map_borders[3]-self.map_borders[2]))
    #     self.map_state = np.zeros((self.img_height, self.img_width), dtype=np.uint8)
    #     # self.map_disp = np.zeros((img_height, img_width, 3), dtype=np.uint8)
    #     self.map_size = (self.img_height, self.img_width)

    #     for i in range(self.img_height):
    #         for j in range(self.img_width):
    #             self.map_state[i,j] = OUT
    #             # self.map_disp[i,j,:] = [0,OUT,0]

    #     # set origin on map
    #     self.origin_on_map = (self.map_size[1]//2, self.map_size[0]//2)         # (0,0) on map
    #     # print("how many roads", len(self.traffic.cropped_roads))  # the road is in pieces and trying to connect them together
    #     # add the angle information based on two adjacent points
    #     single_route = []
   
    #     for i in range(len(route)-1):
    #         pt0 = route[i]
    #         pt1 = route[i+1]
    #         angle = np.arctan2(pt1[1]-pt0[1],pt1[0]-pt0[0])
    #         idx = self._utmToIdx(pt0)
    #         # coord = self._idxToCoords(idx)
    #         # print("idx", idx, "coord", coord)
    #         single_route.append( [ idx[0], idx[1], angle ] )
    #     # final point
    #     idx = self._utmToIdx(pt1)
    #     single_route.append( [ idx[0], idx[1], angle ] )
    #     # self.route_list.append(single_route)
            
    #     # add road on image
    #     for i in range(len(single_route)-1):
    #         pt0 = single_route[i]
    #         pt1 = single_route[i+1]
    #         cv2.line(self.map_state, (pt0[1], pt0[0]), (pt1[1], pt1[0]), ROAD, int(LANE_WIDTH*2*self.map_scale))
    #         # cv2.line(self.map_disp, (pt0[1], pt0[0]), (pt1[1], pt1[0]), (ROAD, ROAD, ROAD), int(LANE_WIDTH*2*self.map_scale))


    #     # add borders around map
    #     for i in range(self.map_state.shape[0]):
    #         self.map_state[i][0] = OUT
    #         self.map_state[i][-1] = OUT
    #         # self.map_disp[i][0] = [0,0,0]
    #         # self.map_disp[i][-1] = [0,0,0]
    #     for j in range(self.map_state.shape[1]):
    #         self.map_state[-1][j] = OUT
    #         self.map_state[0][j] = OUT
    #         # self.map_disp[0][j] = [0,0,0]
    #         # self.map_disp[-1][j] = [0,0,0]

    #     # store the img
    #     cv2.imwrite("new_map.jpg", self.map_state)
    #     # return the smoothened / approximated road map
    #     approx = self.approx_map()



    #     # calculate other scales
    #     # self.screen_scale = int(SCREEN_SCALE/self.map_scale)  # scale when displaying to screen to give the right display size
    #     # self.obs_scale = int(OBS_SCALE/self.map_scale)      # convert obs_px per meters to obs_px per map_px

    #     # create copies of maps, to be shared among vehicles and updated with each vehicle occupancy per timestep
    #     # self.map_state_shared = self.map_state.copy()
    #     # self.map_disp_shared = self.map_disp.copy()

    #     # set origin on map
    #     # self.origin_on_map = (self.map_size[1]//2, self.map_size[0]//2)         # (0,0) on map

    #     # self.observation_size = (48, 48)

    #     # if self.render:
    #     #     self.screen_size = self.screen.get_size()
    #     #     self.scaled_screen_size = (self.screen_size[0]//self.screen_scale,self.screen_size[1]//self.screen_scale)

    #     #     # create surfaces
    #     #     self.screen_surf = pygame.Surface(self.scaled_screen_size)
    #     #     self.map_surf = pygame.surfarray.make_surface(np.transpose(self.map_disp, (1, 0, 2))) # pygame axis sequence x,y,ch
    #     #     self.state_surf = pygame.Surface((0.2*self.screen_size[0],self.observation_size[1]/self.observation_size[0]*0.2*self.screen_size[0]))

    #     #     # set screen center
    #     #     self._setScreenCenter((0,0))                                            # set origin as screen center
    #     #     self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], \
    #     #                         self.origin_on_map[1] + self.screen_center[1])      # screen center on map
    #     return approx

    # def displayMap(self):
    #     self.screen = pygame.display.set_mode((SCREEN_W,SCREEN_H))
    #     pygame.display.set_caption('Map simulator')
    #     self.screen_surf.fill(BACKGROUND)
    #     self.screen_surf.blit(self.map_surf, (0,0), \
    #                           (self.center_on_map[0]-self.scaled_screen_size[0]//2, self.center_on_map[1]-self.scaled_screen_size[1]//2, \
    #                            self.scaled_screen_size[0], self.scaled_screen_size[1]) )
    #     self.screen.blit(pygame.transform.scale(self.screen_surf, self.screen_size), (0,0))
    #     pygame.time.wait(5000)
    #     pygame.display.flip()


    # def loadMap(self):
    #     # get random route
    #     route = self.traffic.randomRoute(dist=TASK_DIST, np_random=self.np_random)

    #     # crop map
    #     self.map_borders = [
    #         np.amin(np.array(route)[:,0]) - MAP_CLEARANCE,
    #         np.amax(np.array(route)[:,0]) + MAP_CLEARANCE,
    #         np.amin(np.array(route)[:,1]) - MAP_CLEARANCE,
    #         np.amax(np.array(route)[:,1]) + MAP_CLEARANCE
    #     ]
    #     map_center = ( (self.map_borders[0]+self.map_borders[1])/2, (self.map_borders[2]+self.map_borders[3])/2 )
    #     radius = max( map_center[0] - self.map_borders[0], map_center[1] - self.map_borders[2] )
    #     self.traffic.cropMap(center=map_center, radius=radius)
    #     # uncomment above to show all roads instead of only the ones going to be travelled through by agent

    #     # create image
    #     img_width = int(self.map_scale*(self.map_borders[1]-self.map_borders[0]))
    #     img_height = int(self.map_scale*(self.map_borders[3]-self.map_borders[2]))
    #     self.map_state = np.zeros((img_height, img_width), dtype=np.uint8)
    #     self.map_disp = np.zeros((img_height, img_width, 3), dtype=np.uint8)
    #     self.map_size = (img_height, img_width)

    #     for i in range(img_height):
    #         for j in range(img_width):
    #             self.map_state[i,j] = OUT
    #             self.map_disp[i,j,:] = [0,OUT,0]

    #     # add roads on image
    #     for road in self.traffic.cropped_roads: 
    #         for i in range(len(road)-1):
    #             pt0 = self._utmToIdx(road[i])
    #             pt1 = self._utmToIdx(road[i+1])
    #             self.map_state = cv2.line(self.map_state, (pt0[1], pt0[0]), (pt1[1], pt1[0]), ROAD, int(LANE_WIDTH*2*self.map_scale))
    #             self.map_disp = cv2.line(self.map_disp, (pt0[1], pt0[0]), (pt1[1], pt1[0]), (ROAD, ROAD, ROAD), int(LANE_WIDTH*2*self.map_scale))

    #     # add borders around map
    #     for i in range(self.map_state.shape[0]):
    #         self.map_state[i][0] = OUT
    #         self.map_state[i][-1] = OUT
    #         self.map_disp[i][0] = [0,0,0]
    #         self.map_disp[i][-1] = [0,0,0]
    #     for j in range(self.map_state.shape[1]):
    #         self.map_state[0][j] = OUT
    #         self.map_state[-1][j] = OUT
    #         self.map_disp[0][j] = [0,0,0]
    #         self.map_disp[-1][j] = [0,0,0]
        
    #     # convert route from utm to index on map
    #     self.route = []
    #     for i in range(len(route)-1):
    #         pt0 = route[i]
    #         pt1 = route[i+1]
    #         angle = np.arctan2(pt1[1]-pt0[1],pt1[0]-pt0[0])
    #         idx = self._utmToIdx(pt0)
    #         self.route.append( [ idx[0], idx[1], angle ] )
    #     idx = self._utmToIdx(pt1)
    #     self.route.append( [ idx[0], idx[1], angle ] )

    #     # calculate other scales
    #     self.screen_scale = int(SCREEN_SCALE/self.map_scale)  # scale when displaying to screen to give the right display size
    #     self.obs_scale = int(OBS_SCALE/self.map_scale)      # convert obs_px per meters to obs_px per map_px

    #     # create copies of maps, to be shared among vehicles and updated with each vehicle occupancy per timestep
    #     self.map_state_shared = self.map_state.copy()
    #     self.map_disp_shared = self.map_disp.copy()

    #     # set origin on map
    #     self.origin_on_map = (self.map_size[1]//2, self.map_size[0]//2)         # (0,0) on map

    #     if self.render:
    #         self.screen_size = self.screen.get_size()
    #         self.scaled_screen_size = (self.screen_size[0]//self.screen_scale,self.screen_size[1]//self.screen_scale)

    #         # create surfaces
    #         self.screen_surf = pygame.Surface(self.scaled_screen_size)
    #         self.map_surf = pygame.surfarray.make_surface(np.transpose(self.map_disp, (1, 0, 2))) # pygame axis sequence x,y,ch
    #         self.state_surf = pygame.Surface((0.2*self.screen_size[0],self.observation_size[1]/self.observation_size[0]*0.2*self.screen_size[0]))

    #         # set screen center
    #         self._setScreenCenter((0,0))                                            # set origin as screen center
    #         self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], \
    #                             self.origin_on_map[1] + self.screen_center[1])      # screen center on map

    # # Private Methods for transformations #
    # def _utmToIdx(self, utm):
    #     i = self.map_size[0]-1-(utm[1]-self.map_borders[2])*self.map_scale
    #     j = (utm[0]-self.map_borders[0])*self.map_scale
    #     return ( int(i), int(j) )
    
    # def _idxToUtm(self, idx):
    #     x = idx[1] / self.map_scale + self.map_borders[0]
    #     y = ( self.map_size[0]-1 - idx[0] ) / self.map_scale + self.map_borders[2]
    #     return ( x, y )
    
    # def _utmToCoords(self, utm):
    #     x = utm[0] - self.map_borders[0] - self.origin_on_map[0]/self.map_scale
    #     y = utm[1] - self.map_borders[2] - (self.map_size[0]-1-self.origin_on_map[1])/self.map_scale
    #     return ( x, y )
    
    # def _coordsToUtm(self, coords):
    #     x = coords[0] + self.origin_on_map[0]/self.map_scale + self.map_borders[0]
    #     y = coords[1] + (self.map_size[0]-1-self.origin_on_map[1])/self.map_scale + self.map_borders[2]
    #     return ( x, y )

    # def _coordsToIdx(self, coords):
    #     j = self.origin_on_map[0] + coords[0]*self.map_scale
    #     i = self.origin_on_map[1] - coords[1]*self.map_scale
    #     # clip to ensure within map
    #     if i < 0 or i >= self.map_size[0] or j < 0 or j >= self.map_size[1]:
    #         i = min(max(i,0),self.map_size[0]-1)
    #         j = min(max(j,0),self.map_size[1]-1)
    #     return ( int(i), int(j) )

    # def _idxToCoords(self, idx):
    #     x = (idx[1] - self.origin_on_map[0]) / self.map_scale
    #     y = (idx[0] - self.origin_on_map[1]) / self.map_scale * -1
    #     return ( x, y )

    # def _idxToScreen(self, idx):
    #     x = ( idx[1] - self.origin_on_map[0] - self.screen_center[0] + self.scaled_screen_size[0]/2 ) * self.screen_scale
    #     y = ( idx[0] - self.origin_on_map[1] - self.screen_center[1] + self.scaled_screen_size[1]/2 ) * self.screen_scale
    #     return ( int(x), int(y) )

    # def _coordsToScreen(self, coords):
    #     x = ( coords[0]    * self.map_scale - self.screen_center[0] + self.scaled_screen_size[0]/2 ) * self.screen_scale
    #     y = ( coords[1]*-1 * self.map_scale - self.screen_center[1] + self.scaled_screen_size[1]/2 ) * self.screen_scale
    #     return ( int(x), int(y) )
    
    # def _screenToCoords(self, screen):
    #     x = ( screen[0] / self.screen_scale - self.scaled_screen_size[0]/2 + self.screen_center[0] ) / self.map_scale
    #     y = ( screen[1] / self.screen_scale - self.scaled_screen_size[1]/2 + self.screen_center[0] ) / self.map_scale * -1
    #     return ( x, y )

    # def _coordsWithinMap(self, coords):
    #     j = self.origin_on_map[0] + coords[0]*self.map_scale
    #     i = self.origin_on_map[1] - coords[1]*self.map_scale
    #     if i < 0 or i >= self.map_size[0] or j < 0 or j >= self.map_size[1]: return False
    #     else: return True
    
    # def _setScreenCenter(self, coords):
    #     self.screen_center = [ coords[0]*self.map_scale , -1*coords[1]*self.map_scale ]
    #     self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], self.origin_on_map[1] + self.screen_center[1])

    # def _updateScreenCenter(self, coords):
    #     pos_on_screen = self._coordsToScreen(coords)
    #     if pos_on_screen[0] < SCREEN_BORDER*self.screen_size[0]:
    #         self.screen_center[0] -= ( SCREEN_BORDER*self.screen_size[0] - pos_on_screen[0] ) / self.screen_scale
    #     elif pos_on_screen[0] > (1-SCREEN_BORDER)*self.screen_size[0]:
    #         self.screen_center[0] += ( pos_on_screen[0] - (1-SCREEN_BORDER)*self.screen_size[0] ) / self.screen_scale
    #     if pos_on_screen[1] < SCREEN_BORDER*self.screen_size[1]:
    #         self.screen_center[1] -= ( SCREEN_BORDER*self.screen_size[1] - pos_on_screen[1] ) / self.screen_scale
    #     elif pos_on_screen[1] > (1-SCREEN_BORDER)*self.screen_size[1]:

    #         self.screen_center[1] += ( pos_on_screen[1] - (1-SCREEN_BORDER)*self.screen_size[1] ) / self.screen_scale
    #     self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], self.origin_on_map[1] + self.screen_center[1])


if __name__ == '__main__':

    world = World()
    # world.reset()
