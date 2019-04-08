""" This file defines an agent for the bus simulator. """
from copy import deepcopy
import numpy as np
from gps.agent.agent import Agent
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_BUS
from gps.proto.gps_pb2 import ACTION
from gps.sample.sample import Sample

class AgentBusPol(Agent):
	"""
	All communication between the algorithms and Vehicle is done through
	this class.
	"""
	def __init__(self, hyperparams):
		config = deepcopy(AGENT_BUS)
		config.update(hyperparams)
		Agent.__init__(self, config)
		self._setup_conditions()
		self.reach_start = None
		self.reach_end = None
		self.finishing = None
		self.finishing_time = None
		self._setup_world(self._hyperparams["world"],
						  self._hyperparams["target_state"],
						  self._hyperparams["render"],
						  self._hyperparams["polygons"],
						  self._hyperparams["map_size"],
						  self._hyperparams["map_state"],
						  self._hyperparams["display_center"],)
		

	def _setup_conditions(self):
		"""
		Helper method for setting some hyperparameters that may vary by
		condition.
		"""
		conds = self._hyperparams['conditions']
		# for field in ('x0', 'x0var', 'pos_body_idx', 'pos_body_offset',
        # 	'noisy_body_idx', 'noisy_body_var', 'filename'):
        #     self._hyperparams[field] = setup(self._hyperparams[field], conds)
		self._hyperparams['x0'] = setup(self._hyperparams['x0'], conds)


	def _setup_world(self, world, target_state, render, polygons, map_size, map_state, display_center):
		"""
		Helper method for handling setup of the Box2D world.
		"""
		self.x0 = self._hyperparams["x0"]  # initial state
		self._worlds = [world(self.x0[i], target_state, render, map_size, polygons=polygons, map_state=map_state, display_center=display_center)
						for i in range(self._hyperparams['conditions'])]
		
	def sample(self, policy, condition, verbose=False, save=True, noisy=True):
		"""
		Runs a trial and constructs a new sample containing information
		about the trial.

		Args:
			policy: Policy to to used in the trial.
			condition (int): Which condition setup to run.
			verbose (boolean): Whether or not to plot the trial (not used here).
			save (boolean): Whether or not to store the trial into the samples.
			noisy (boolean): Whether or not to use noise during sampling.
		"""
        # Modified on April 2, referring to agent_mjc
		# reset the world and assign the initialized state to new_sample
		# self._worlds[condition].run()
		# self._worlds[condition].reset_world()
        feature_fn = None
        if 'get_features' in dir(policy):
            feature_fn = policy.get_features
		b2d_X = self._worlds[condition].get_state()
		new_sample = self._init_sample(b2d_X)
		# initialize a dummy action sequence
		U = np.zeros([self.T, self.dU])		
		self.reach_start = None
		self.reach_end = None	

		if noisy:
			noise = generate_noise(self.T, self.dU, self._hyperparams)
		else:
			noise = np.zeros((self.T, self.dU))
		for t in range(self.T):
			X_t = new_sample.get_X(t=t)
			obs_t = new_sample.get_obs(t=t)
			U[t, :] = policy.act(X_t, obs_t, t, noise[t, :])
			# print(U[t])
			if (t+1) < self.T:
				for _ in range(self._hyperparams['substeps']):
					self._worlds[condition].run_next(U[t, :])
				if self._worlds[condition].reach:
					self._worlds[condition].reach = False
					if self.reach_start == None:
						self.reach_start = t
						# print("reach_start", t)
					elif self.reach_end == None or t>self.reach_end:
						self.reach_end = t
					if t==self.T-2:
						# continue reaching till the end of series
						# print("reach_end", self.reach_end)
						period = self.reach_end - self.reach_start
						# print("reach period", period)
						if period > 3:
							self.finishing = True
							# self.finishing_time = self.reach_end
							self.finishing_time = self.reach_start
				elif self.reach_end == t-1 :
					# just leave
					# print("reach_end", self.reach_end)
					period = self.reach_end - self.reach_start
					# print("reach period", period)
					if period > 1:
						self.finishing = True
						# self.finishing_time = self.reach_end
						self.finishing_time = self.reach_start
						if self.finishing_time == 0:
							self.finishing_time = 1				
				b2d_X = self._worlds[condition].get_state()
				self._set_sample(new_sample, b2d_X, t)			
		
		new_sample.set(ACTION, U)
		if save:
			self._samples[condition].append(new_sample)
		# if self.finishing:
			# print("agent_bus t= ", self.finishing_time)
		return new_sample

	def _init_sample(self, b2d_X):
		"""
		Construct a new sample and fill in the first time step.
		"""
		sample = Sample(self)
		self._set_sample(sample, b2d_X, -1)
		return sample

	def _set_sample(self, sample, b2d_X, t):
		for sensor in b2d_X.keys():
			sample.set(sensor, np.array(b2d_X[sensor]), t=t+1)
	
	def get_image_from obs_(self):

    # def _get_image_from_obs(self, obs):
    #     # used to capture and store iamges, not in use yet
    #     imstart = 0
    #     imend = 0
    #     image_channels = self._hyp  erparams['image_channels']
    #     image_width = self._hyperparams['image_width']
    #     image_height = self._hyperparams['image_height']
    #     for sensor in self._hyperparams['obs_include']:
    #         # Assumes only one of RGB_IMAGE or CONTEXT_IMAGE is present
    #         if sensor == RGB_IMAGE or sensor == CONTEXT_IMAGE:
    #             imend = imstart + self._hyperparams['sensor_dims'][sensor]
    #             break
    #         else:
    #             imstart += self._hyperparams['sensor_dims'][sensor]
    #     img = obs[imstart:imend]
    #     img = img.reshape((image_width, image_height, image_channels))
    #     return img


	def update(self):
        # update route if another tile covered
        cover = self._checkOccupancy(self.pos, self.env.map_state_shared, [UNCOVERED])
        while cover != 0:
            self.dist_count += 1
            self.last_move = 0
            idxs = [ self.env._coordsToIdx( self.route[0] ) , self.env._coordsToIdx( self.route[0+1] ) ]
            poly = np.array( [ [ idxs[0][1] ,idxs[0][0] ], \
                               [ idxs[0][1]+LANE_WIDTH*self.env.map_scale*np.cos(self.route[0][2]+np.pi/2),idxs[0][0]-LANE_WIDTH*self.env.map_scale*np.sin(self.route[0][2]+np.pi/2)], \
                               [ idxs[1][1]+LANE_WIDTH*self.env.map_scale*np.cos(self.route[0][2]+np.pi/2),idxs[1][0]-LANE_WIDTH*self.env.map_scale*np.sin(self.route[0][2]+np.pi/2)], \
                               [ idxs[1][1] ,idxs[1][0] ]  ] , np.int32)
            self.env.map_state_shared = cv2.fillConvexPoly(self.env.map_state_shared,poly,ROAD)
            cover = self._checkOccupancy(self.pos, self.env.map_state_shared, [UNCOVERED])
            self.route.pop(0)
        if len(self.route) <= 1: self.reach_goal = True

        # renew shared map, remove other cars
        self.env.map_state_shared = self.env.map_state.copy()

        # add uncovered tiles
        for i in range(min(SHOW_TILES,len(self.route)-1)):
            idxs = [ self.env._coordsToIdx( self.route[i] ) , self.env._coordsToIdx( self.route[i+1] ) ]
            poly = np.array( [ [ idxs[0][1] ,idxs[0][0] ], \
                               [ idxs[0][1]+LANE_WIDTH*self.env.map_scale*np.cos(self.route[i][2]+np.pi/2),idxs[0][0]-LANE_WIDTH*self.env.map_scale*np.sin(self.route[i][2]+np.pi/2)], \
                               [ idxs[1][1]+LANE_WIDTH*self.env.map_scale*np.cos(self.route[i][2]+np.pi/2),idxs[1][0]-LANE_WIDTH*self.env.map_scale*np.sin(self.route[i][2]+np.pi/2)], \
                               [ idxs[1][1] ,idxs[1][0] ]  ] , np.int32)
            self.env.map_state_shared = cv2.fillConvexPoly(self.env.map_state_shared,poly,UNCOVERED)
            self.env.map_disp_shared = cv2.fillConvexPoly(self.env.map_disp_shared,poly,(UNCOVERED, UNCOVERED, UNCOVERED))
            
        # add self, draw rectangle, meanning the pos is updated 
        c = np.cos(self.pos[2])
        s = np.sin(self.pos[2])
        coords = []
        for i in np.arange(0,self.wheelbase/2,0.8/self.env.map_scale):
            for j in np.arange(0,self.trackwidth/2,0.8/self.env.map_scale):
                coords.append( (self.pos[0] + i*c - j*s, self.pos[1] + i*s + j*c ) )
                coords.append( (self.pos[0] - i*c - j*s, self.pos[1] - i*s + j*c) )
                coords.append( (self.pos[0] + i*c + j*s, self.pos[1] + i*s - j*c) )
                coords.append( (self.pos[0] - i*c + j*s, self.pos[1] - i*s - j*c) )
        for coord in coords:
            idx = self.env._coordsToIdx(coord)
            self.env.map_state_shared[idx[0]][idx[1]] = SELF
        self.env.map_disp_shared = self.env.map_disp.copy()
        idx = self.env._coordsToIdx(self.pos)
        # rot_mat = cv2.getRotationMatrix2D( (self.img_overlay.shape[0]//2,self.img_overlay.shape[1]//2), self.pos[2]*180/np.pi, 1)
        rot_mat = cv2.getRotationMatrix2D( (self.img_overlay.shape[0]//2,self.img_overlay.shape[1]//2), 0, 1)
        overlay = cv2.warpAffine(self.img_overlay, rot_mat, (self.img_overlay.shape[0],self.img_overlay.shape[1]), borderMode=cv2.BORDER_CONSTANT, borderValue=[255,255,255] )
        for i in range(overlay.shape[0]):
            for j in range(overlay.shape[1]):
                if not np.all(np.greater(overlay[i,j,:],[150,150,150])):
                    self.env.map_disp_shared[int(idx[0]-overlay.shape[0]/2)+i,int(idx[1]-overlay.shape[1]/2)+j,:] = overlay[i,j,:]

        # update timer
        if self.env.tstep is not None:
            self.last_move += 1
            self.elapsed_time += 1
            self.no_move = self.last_move > WAIT_TIME/self.env.tstep
            self.episode_terminate = self.elapsed_time > MAX_EP_TIME/self.env.tstep
        else:
            tick = self.step_timer.tick()
            self.last_move += tick
            self.elapsed_time += tick
            self.no_move = self.last_move > WAIT_TIME * 1000
            self.episode_terminate = self.elapsed_time > MAX_EP_TIME * 1000

        # update screen center to ensure ego vehicle within border
        if self.env.render: self.env._updateScreenCenter(self.pos)
    
	def _checkOccupancy(self, pos, occ_grid, occ_state, rad=None):
        coords = []

        if rad is None:
            l = self.wheelbase // 2
            w = self.trackwidth // 2
            c = np.cos(pos[2])
            s = np.sin(pos[2])
            for i in np.arange(-w,w,0.8*self.env.map_scale):
                coords.append( (pos[0] + l*c - i*s, pos[1] + l*s + i*c) )
                coords.append( (pos[0] - l*c - i*s, pos[1] - l*s + i*c) )
            for i in np.arange(-l,l,0.8*self.env.map_scale):
                coords.append( (pos[0] + i*c - w*s, pos[1] + i*s + w*c) )
                coords.append( (pos[0] + i*c + w*s, pos[1] + i*s - w*c) )
        elif type(rad) == int or type(rad) == float:
            for i in np.arange(-rad,rad,0.8*self.env.map_scale):
                for j in np.arange(-rad,rad,0.8*self.env.map_scale):
                    coords.append( (pos[0]+i, pos[1]+j) )
        elif type(rad) == list or type(rad) == tuple:
            w = rad[0]/2
            h = rad[1]/2
            c = np.cos(pos[2])
            s = np.sin(pos[2])
            for i in np.arange(-h,h,0.8*self.env.map_scale):
                coords.append( (pos[0] + w*c - i*s, pos[1] + w*s + i*c) )
                coords.append( (pos[0] - w*c - i*s, pos[1] - w*s + i*c) )
            for i in np.arange(-w,w,0.8*self.env.map_scale):
                coords.append( (pos[0] + i*c - h*s, pos[1] + i*s + h*c) )
                coords.append( (pos[0] + i*c + h*s, pos[1] + i*s - h*c) )
        else:
            print('Unrecognized rad type:',str(type(rad)))
            return

        for coord in coords:
            idx = self.env._coordsToIdx(coord)
            if occ_grid[idx[0]][idx[1]] in occ_state: return idx
        return 0