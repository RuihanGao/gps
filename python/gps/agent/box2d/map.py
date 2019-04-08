import Box2D as b2 
import numpy as np
import cv2
import pygame
from framework import Framework 
from gps.agent.box2d.settings import fwSettings
from gps.agent.box2d.traffic import Traffic 

# map and display
SCREEN_SCALE = 1  #10    # screen pixels per meter
OBS_SCALE = 1            # observation pixels per meter
MAP_SCALE = 1            # map pixels per meter
MAP_CLEARANCE = 20       # clearance between route and map borders (meters)
LANE_WIDTH = 12 #8        # width of lane in meters
COVER_TILE = 20 #10          # distance per tile of road to cover
TASK_DIST = 1000         # meters from starting point to goal
TARGET_IDX = 5
# state values
# keep consistent with emily's one 
ROAD = 150
OUT = 50
OBSTACLE = 0
BUS = 255
TARGET = 200  # same as UNCOVERED in emily's

SUB_MAP_SIZE = 200
SUB_MAP_CLEARANCE = 20

class Map():
    def __init__(self, img_name, crop_img):
        """
        Modified from loadMap()
        Generate a random route [x, y, yaw] from the traffic
        create road on self.map_state and self.map_disp (in Idx)
        store the map named in img_name
        """
        self.traffic = Traffic()
        # get a random route from traffic
        route = self.traffic.randomRoute(dist=TASK_DIST, np_random=np.random.RandomState(seed=13))

        # crop map
        self.map_borders = [
            np.amin(np.array(route)[:,0]) - MAP_CLEARANCE,
            np.amax(np.array(route)[:,0]) + MAP_CLEARANCE,
            np.amin(np.array(route)[:,1]) - MAP_CLEARANCE,
            np.amax(np.array(route)[:,1]) + MAP_CLEARANCE,
        ] # wrt utm coords
        
        map_center = ( (self.map_borders[0]+self.map_borders[1])/2, (self.map_borders[2]+self.map_borders[3])/2 )
        # radius = max( map_center[0] - self.map_borders[0], map_center[1] - self.map_borders[2] )
        # self.traffic.cropMap(center=map_center, radius=radius)
        # uncomment above to show all roads instead of only the ones going to be travelled through by agent

        # calculate scales
        self.map_scale = MAP_SCALE
        self.screen_scale = int(SCREEN_SCALE/self.map_scale)  # scale when displaying to screen to give the right display size
        self.obs_scale = int(OBS_SCALE/self.map_scale)      # convert obs_px per meters to obs_px per map_px

        # create an image of the map
        map_height = int(self.map_scale*(self.map_borders[1]-self.map_borders[0]))
        map_width = int(self.map_scale*(self.map_borders[3]-self.map_borders[2]))
        self.map_size = (map_width, map_height) 
        # initialize a grey scale one for training and a RGB one for colorful display
        self.map_state = np.zeros((map_height, map_width), dtype=np.uint8)
        self.map_disp = np.zeros((map_height, map_width, 3), dtype=np.uint8)
        # self.map_local = np.zeros((SUB_MAP_SIZE, SUB_MAP_SIZE), dtype=np.uint8)
        
        # draw the background of the map
        for i in range(map_height):
            for j in range(map_width):
                self.map_state[i,j] = OUT
                self.map_disp[i,j,:] = [0,OUT,0]


        # add roads on map, the roads are cropped and added in randomRoute method
        self.road = []
        for road in self.traffic.cropped_roads: 
            for i in range(len(road)-1):          
                pt0 = self._utmToIdx(road[i])
                pt1 = self._utmToIdx(road[i+1])
                self.road.append(pt0)
                # cv2.line(self.map_state, (pt0[1], pt0[0]), (pt1[1], pt1[0]), ROAD, int(LANE_WIDTH*2*self.map_scale))
                cv2.line(self.map_state, (pt0[0], pt0[1]), (pt1[0], pt1[1]), ROAD, int(LANE_WIDTH*2*self.map_scale))
                # cv2.line(self.map_disp, (pt0[1], pt0[0]), (pt1[1], pt1[0]), (ROAD, ROAD, ROAD), int(LANE_WIDTH*2*self.map_scale))
                cv2.line(self.map_disp, (pt0[0], pt0[1]), (pt1[0], pt1[1]), (ROAD, ROAD, ROAD), int(LANE_WIDTH*2*self.map_scale))
            self.road.append(pt1)
        # print("road", self.road)

        # add borders around map
        for i in range(self.map_state.shape[0]):
            self.map_state[i][0] = OUT
            self.map_state[i][-1] = OUT
            self.map_disp[i][0] = [0,0,0]
            self.map_disp[i][-1] = [0,0,0]
        for j in range(self.map_state.shape[1]):
            self.map_state[0][j] = OUT
            self.map_state[-1][j] = OUT
            self.map_disp[0][j] = [0,0,0]
            self.map_disp[-1][j] = [0,0,0]
        
        # convert route from utm to index on map, and include yaw information by calculating the angle
        self.route = []
        idx_route = []
        for pt in route:
            idx_route.append(self._utmToIdx(pt))

        # for i in range(len(route)-1):
        #     pt0 = route[i]
        #     pt1 = route[i+1]
        #     angle = np.arctan2(pt1[1]-pt0[1],pt1[0]-pt0[0])      
        #     idx = self._utmToIdx(pt0)
        #     print(idx, self._utmToIdx(pt1), angle)
        #     # self.route.append( [ idx[0], idx[1], angle ] )
        #     # self.route.append( [ idx[0], idx[1], -angle-np.pi/2])
        #     # to adjust the angle into (-pi, pi)
        #     angle = np.arctan2(np.sin(-angle), np.cos(-angle))
        #     self.route.append([ idx[0], idx[1], angle])           
        # idx = self._utmToIdx(pt1)
        # self.route.append( [ idx[0], idx[1], angle ] )


        for i in range(len(idx_route)-1):
            pt0 = idx_route[i]
            pt1 = idx_route[i+1]
            angle = -np.arctan2(pt1[1]-pt0[1],pt1[0]-pt0[0])
            # print(pt0, pt1, angle)
            self.route.append([ pt0[0], pt0[1], angle])           
        self.route.append( [ pt1[0], pt1[1], angle] )

        # take the lane information for each waypoint in the route
        for i in range(len(route)):
            self.route[i] = [ self.route[i][0] + LANE_WIDTH/2*np.cos(self.route[i][2]+np.pi/2) , self.route[i][1] + LANE_WIDTH/2*np.sin(self.route[i][2]+np.pi/2) , self.route[i][2] ]
        
        self.insertroute()
        # comment below after cleaning up the route
        # self.route = self.full_route




        # route_borders = [
        # np.amin(np.array(self.route)[:,0]),
        # np.amax(np.array(self.route)[:,0]),
        # np.amin(np.array(self.route)[:,1]),
        # np.amax(np.array(self.route)[:,1])]
        # print("route_borders", route_borders)

        # self.test_road_route(route)
        [self.xmin, self.ymin] = self._utmToIdx([self.map_borders[0], self.map_borders[2]])
        [self.xmax, self.ymax] = self._utmToIdx([self.map_borders[1], self.map_borders[3]])
       
        # calculate scales
        self.map_scale = MAP_SCALE
        self.screen_scale = int(SCREEN_SCALE/self.map_scale)  # scale when displaying to screen to give the right display size
        self.obs_scale = int(OBS_SCALE/self.map_scale)      # convert obs_px per meters to obs_px per map_px

        # create copies of maps, to be shared among vehicles and updated with each vehicle occupancy per timestep
        self.map_state_shared = self.map_state.copy()
        self.map_disp_shared = self.map_disp.copy()

        # set origin on map
        self.origin_on_map = (self.map_size[1]//2, self.map_size[0]//2)         # (0,0) on map, et origin at the center
        # store the img
        self.img_name = img_name
        self.crop_img = crop_img
        cv2.imwrite(self.img_name, self.map_state)
        # print("route", self.route)
        print("route length", len(self.route))
        # # initialize x0 and target
        # self.x0 = [ self.route[0][0] + LANE_WIDTH/2*np.cos(self.route[0][2]+np.pi/2) , self.route[0][1] + LANE_WIDTH/2*np.sin(self.route[0][2]+np.pi/2) , self.route[0][2] ]
        # self.target = [ self.route[TARGET_IDX][0] + LANE_WIDTH/2*np.cos(self.route[TARGET_IDX][2]+np.pi/2) , self.route[TARGET_IDX][1] + LANE_WIDTH/2*np.sin(self.route[TARGET_IDX][2]+np.pi/2) , self.route[TARGET_IDX][2] ]
        # # if self.render:
        #     self.screen_size = self.screen.get_size()
        #     self.scaled_screen_size = (self.screen_size[0]//self.screen_scale,self.screen_size[1]//self.screen_scale)

        #     # create surfaces
        #     self.screen_surf = pygame.Surface(self.scaled_screen_size)
        #     self.map_surf = pygame.surfarray.make_surface(np.transpose(self.map_disp, (1, 0, 2))) # pygame axis sequence x,y,ch
        #     self.state_surf = pygame.Surface((0.2*self.screen_size[0],self.observation_size[1]/self.observation_size[0]*0.2*self.screen_size[0]))

        # set screen center
        self._setScreenCenter((0,0))                                            # set origin as screen center
        # self.center_on_map = (self.origin_on_map[0] + self.screen_center[0], \
        #                     self.origin_on_map[1] + self.screen_center[1])      # screen center on map
    

    def divideRoute(self):
        # temporarily deprecated 
        """
        convert larger piece-wise route from Idx to coordinates, from starting position to goal
        divide it to shorter tiles based on COVER_TILE, and add the yaw information
        return a div_route that has been divided into tiles (smaller scales) 
        """
        self.div_route = []
        for i in range(len(self.route)-1):
            pt0 = self._idxToCoords( self.route[i] )
            pt1 = self._idxToCoords( self.route[i+1] )
            dist = euclidDist(pt0, pt1)
            for tile_dist in np.arange(0, dist, COVER_TILE):
                pos = [ pt0[0] + tile_dist*np.cos(self.route[i][2]) , pt0[1] + tile_dist*np.sin(self.route[i][2]) , self.route[i][2] ]
                self.div_route.append( pos )
            self.div_route.append( [ pt1[0], pt1[1], self.route[i][2] ] )
        # record the starting point
        starting_point = self.div_route[0]
        print("starting point", starting_point)
        for i in range(len(self.div_route)):
            self.div_route[i] = [self.div_route[i][0] - starting_point[0], self.div_route[i][1] - starting_point[1], self.div_route[i][2] - starting_point[2]]
        # put vehicle in starting position
        self.x0 = self.div_route[0]
        # x0 = [ self.div_route[0][0] + LANE_WIDTH/2*np.cos(self.div_route[0][2]+np.pi/2) , self.div_route[0][1] + LANE_WIDTH/2*np.sin(self.div_route[0][2]+np.pi/2) , self.div_route[0][2] ]

        # delete first tile since it will always be covered at the start
        self.div_route.pop(0)

        # TODO: put this at reset box2d screen. update the center of screen
        # self.screen._setScreenCenter(coords=starting_point[0:2])
        # return x0
    
    def insertroute(self):
        # # save the script in case you want to display the original route
        # self.full_route = self.route

        self.full_route = []
        for i in range(len(self.route)-1):
            pt0 = self.route[i]
            pt1 = self.route[i+1]
            dist = euclidDist(pt0, pt1)
            if abs(pt1[2]-pt0[2])<1:
                if abs(pt0[2])<0.1:
                    if np.sign(pt1[2]):
                        pt0[2] = np.sign(pt1[2])*0.1
                    else:
                        # consecutive zeros, just anyhow assign
                        pt0[2] = 0.1
                self.full_route.append(pt0)
                if abs(pt1[2]-pt0[2])>0.4 and dist<COVER_TILE:
                    # insert a midway point for abrupt yaw change
                    pos = [ (pt0[0]+pt1[0])/2, (pt0[1]+pt1[1])/2 , (pt0[2]+pt1[2])/2]
                    self.full_route.append(pos)
            if dist>COVER_TILE:
                for tile_dist in np.arange(0, dist, COVER_TILE):
                    pos = [ pt0[0] + tile_dist*np.cos(-self.route[i][2]), pt0[1] + tile_dist*np.sin(-self.route[i][2]) , self.route[i][2] ]
                    # print(self.route[i][2], pos)
                    self.full_route.append(pos)
        self.full_route.append( [ pt1[0], pt1[1], self.route[i][2] ] )
        # self.route = self.full_route

        # Modified on March 20, trying to clean the route
        # not valid though
        self.route = []
        pt0 = self.full_route[0]
        pt1 = self.full_route[1]
        if abs(pt0[2]-pt1[2]) < 1.5:
            self.route.append(pt0)
        for i in range(1, len(self.full_route)):
            pt1 = self.full_route[i]
            v1 = [np.cos(pt0[2]), np.sin(pt0[2])] # current direction of the bus
            v2 = [np.cos(pt1[2]), np.sin(pt1[2])] # next direction of the bus
            v_sum = np.add(v1, v2)
            v_sum = v_sum/np.linalg.norm(v_sum)
            v0 = [pt1[0]-pt0[0], -pt1[1]+pt0[1]] # direction of route
            if not np.linalg.norm(v0):
                continue
            v0 = v0/np.linalg.norm(v0)
            dot_prod = np.dot(v_sum, v0)
            
            # print(dot_prod)
            if np.isnan(dot_prod) or dot_prod < 0.3: 
                # the wierd point near destination has a dot_prod of 0.21, so using 0.3 as threshold works. 
                # But since it is hard code, may need checking later 
                #  print("skip")
                continue
            self.route.append(pt1)
            pt0 = pt1
   
        


    # def check_reach_target(self):
    #     # compare the coordinal difference of body and target
    #     diff0 = self.body.position[0] - self.target.position[0] 
    #     diff1 = self.body.position[1] - self.target.position[1]
    #     diff2 = self.body.angle - self.target.angle
    #     # return (abs(diff0*diff1) < (0.1 * BUS_LENGTH * BUS_WIDTH) and abs(diff2) < 0.05)
    #     # if (abs(diff0) < 0.1 * BUS_WIDTH) and (abs(diff1) < 0.1 * BUS_LENGTH) and (abs(diff2) < 0.05):
    #     if (abs(diff0) < 2) and (abs(diff1) < 2) and (abs(diff2) < 0.05):
    #         # print("diff", diff0, diff1, diff2)
    #         # print(self.body.position[0], self.target.position[0], self.body.position[1], self.target.position[1] )
    #         return True
    #     else:
    #         return False


 
    
    def test_road_route(self, route):
        print("ROAD")
        for road in self.traffic.cropped_roads: 
            for i in range(len(road)-1):
                pt0 = self._utmToIdx(road[i])
                print(pt0)               
        print("ROUTE")
        print(self.new_route)

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

def euclidDist(pos1, pos2):
    diff = np.array(pos1[0:2]) - np.array(pos2[0:2])
    return np.linalg.norm(diff)

def _intersectionPoint(vec1, vec2):
    # if vec1[2] == vec2[2]: return float('Inf')
    a = np.array( [ [np.sin(vec1[2]), -np.cos(vec1[2])], [np.sin(vec2[2]), -np.cos(vec2[2])] ] )
    b = np.array( [ vec1[0]*np.sin(vec1[2]) - vec1[1]*np.cos(vec1[2]), vec2[0]*np.sin(vec2[2]) - vec2[1]*np.cos(vec2[2]) ] )
    x, y = np.linalg.solve(a,b)
    return (x, y)

def _angle_2pi(angle):
    while angle>np.pi:
        angle = angle -2*np.pi
    while angle< -np.pi:
        angle = angle + 2*np.pi
    return angle

def approx_polygon(img_in, img_out):

    img = cv2.imread(img_in)
    img_height, img_width, img_channels = img.shape
    # Try to find the contours, line -> polygon
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray map in approx_polygon', imgray)
    # cv2.waitKey(3000)
    ret, thresh = cv2.threshold(imgray, 127, 255, cv2.THRESH_BINARY_INV) # for ROAD 150, OUT 50, use inverse so that it draws obstacle polygon
    # ret, thresh = cv2.threshold(imgray, 127, 255, cv2.THRESH_BINARY) # for ROAD 50, OUT 150
    print("thresh", thresh)
    # # Opencv2 returns 2 objects
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Opencv3 returns 3 objects
    im, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    num_contour = len(contours)
    # print("num", num_contour)
    # Draw the approximated contour on a clear canvas
    thresh_copy = np.zeros((img_height, img_width))
    polygons=[]
    for contour in contours:              
        epsilon = 0.01*cv2.arcLength(contour,True)  # 0.002 # 0.004 # if want to reduce the number of endpoints, increase epsilon
        # print("epsilon", epsilon)
        polygon = cv2.approxPolyDP(contour,epsilon,True)
        polygons.append(polygon)
        cv2.drawContours(thresh_copy, [polygon], -1, (255, 0, 0), 2)
        # cv2.imshow('polygon', thresh_copy)
        # cv2.waitKey(2000)
    # cv2.imshow('polygon', thresh_copy)
    cv2.imwrite(img_out, thresh_copy)
    # print("polygons", polygons)
    return polygons

def draw_rot_rectangle(image, centre, theta, width, height, pixel=BUS, line_width=1):
    # theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    # print(R)
    # print ("center of rotated rect", centre)
    p1 = [ + width / 2,  + height / 2]
    p2 = [- width / 2,  + height / 2]
    p3 = [ - width / 2, - height / 2]
    p4 = [ + width / 2,  - height / 2]
    p1_new = np.dot(p1, R)+ centre
    p2_new = np.dot(p2, R)+ centre
    p3_new = np.dot(p3, R)+ centre
    p4_new = np.dot(p4, R)+ centre
    p1 = np.add(p1, centre)
    cv2.line(image, (int(p1_new[0, 0]), int(p1_new[0, 1])), (int(p2_new[0, 0]), int(p2_new[0, 1])), pixel, line_width)
    cv2.line(image, (int(p2_new[0, 0]), int(p2_new[0, 1])), (int(p3_new[0, 0]), int(p3_new[0, 1])), pixel, line_width)
    cv2.line(image, (int(p3_new[0, 0]), int(p3_new[0, 1])), (int(p4_new[0, 0]), int(p4_new[0, 1])), pixel, line_width)
    cv2.line(image, (int(p4_new[0, 0]), int(p4_new[0, 1])), (int(p1_new[0, 0]), int(p1_new[0, 1])), pixel, line_width)

    return image

def draw_rot_fill_polygon(image, centre, theta, width, height, pixel=BUS, line_width=1):
    # theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    # print(R)
    # print ("center of rotated rect", centre)
    p1 = [ + width / 2,  + height / 2]
    p2 = [ - width / 2,  + height / 2]
    p3 = [ - width / 2,  - height / 2]
    p4 = [ + width / 2,  - height / 2]
    p1_new = np.dot(p1, R)+ centre
    p2_new = np.dot(p2, R)+ centre
    p3_new = np.dot(p3, R)+ centre
    p4_new = np.dot(p4, R)+ centre
    p1 = np.add(p1, centre)
    # print("p1", p1_new)
    poly = np.array( [ [ p1_new[0, 0], p1_new[0, 1] ], \
                       [ p2_new[0, 0] ,p2_new[0, 1] ], \
                       [ p3_new[0, 0] ,p3_new[0, 1] ], \
                       [ p4_new[0, 0] ,p4_new[0, 1] ], ] , np.int32)
    cv2.fillConvexPoly(image,poly,pixel)
    return image