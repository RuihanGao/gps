""" This file defines an agent for the gym environment """
import copy
import time
import numpy as np 

import gym

from gps.agent.agent import Agent
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_GYM
from gps.proto.gps_pb2 import ACTION 
from gps.sample.sample import Sample

class AgentGym(Agent):
    """
    All communication between the algorithms and gym is done through this class
    """
    def __init__(self, hyperparams):
        config = deepcopy(AGENT_GYM):
        config.update(hyperparams)
        Agent.__init__(self, config)

		self.reach_start = None
		self.reach_end = None
		self.finishing = None
		self.finishing_time = None   

        self._setup_conditions()
		self._setup_env(self._hyperparams["world"], # TODO: check later, whether necessary to use "world" instead  of "env" in order to be consistent with GPS
						  self._hyperparams["target_state"],
						  self._hyperparams["render"],
						  self._hyperparams["polygons"],
						  self._hyperparams["map_size"],
						  self._hyperparams['bodies'])

    def _setup_conditions(self):
        """
        Helper method for setting some hyperparameters that may
        vary by condition.
        """
        # TODO: modify the cond fields
        conds = self._hyperparams['conditions']
        self._hyperparams['x0'] = setup(self._hyperparams['x0'], conds)

 	def _setup_env(self, env, target_state, render, polygons, map_size, bodies):
		"""
		Helper method for handling setup of the Box2D world.
		"""
		self.x0 = self._hyperparams["x0"]  # initial state
		self._envs = [env(self.x0[i], target_state, render, polygons, map_size, bodies)
						for i in range(self._hyperparams['conditions'])]
    
    def sample(self, policy, condition, verbose=False, save=True, noisy=True):
        """
        Runs a trial and constructs a new sample containing information
        about the trial.
        Args:
            policy: Policy to to used in the trial.
            condition: Which condition setup to run.
            verbose: Whether or not to plot the trial.
            save: Whether or not to store the trial into the samples.
            noisy: Whether or not to use noise during sampling.
        """
        # reset the environment and assign the initialized state to new_sample
        self._envs[condition].run()
        self._envs[condition].reset()
        gym_X = self._envs[condition].get_state()
        new_sample = self._init_sample(gym_X)

        # initialize a dummy action sequence
        U = np.zeros([self.T, self.dU])
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))
        for t in range(self.T):
            X_t = new_sample.get_X(t=t)
            obs_t = new_sample.get_obs(t=t)
            U[t, :] = policy.act(X_t, obs_t, t, noise[t, :])
 			if (t+1) < self.T:
				for _ in range(self._hyperparams['substeps']):
					self._envs[condition].run_next(U[t, :])
				if self._envs[condition].reach:
					if self.reach_start == None:
						self.reach_start = t
					elif self.reach_end == None or t>self.reach_end:
						self.reach_end = t
					self._envs[condition].reach = False
				elif self.reach_end == t-1:
					# just leave
					period = self.reach_end - self.reach_start
					if period > 3:
						self.finishing = True
						self.finishing_time = self.reach_end
				
				gym_X = self._envs[condition].get_state()
				self._set_sample(new_sample, gym_X, t)	
		new_sample.set(ACTION, U)
		if save:
			self._samples[condition].append(new_sample)
		if self.finishing:
			print("agent_bus t= ", self.finishing_time)
		return new_sample
    
    def _init_sample(self, gym_X):
        """
        Construct a new sample and fill in the first time step.
        """
        sample = Sample(self)
        self._set_sample(sample, gym_X, -1)
        return sample

    def _set_sample(self, sample, gym_X, t):
        for sensor in gym_X.keys():
            sample.set(sensor, np.array(gym_X[sensor]), t=t+1)
