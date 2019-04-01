""" This file defines the main object that runs experiments. """

import matplotlib as mpl
mpl.use('Qt4Agg')

import logging
import imp
import os
import os.path
import sys
import copy
import argparse
import threading
import time
import traceback
import cv2

# Add gps/python to path so that imports work.
sys.path.append('/'.join(str.split(__file__, '/')[:-2]))
from gps.gui.gps_training_gui import GPSTrainingGUI
from gps.utility.data_logger import DataLogger
from gps.sample.sample_list import SampleList
from gps.agent.box2d.map import *
from gps.agent.box2d.agent_bus import AgentBus
from gps.agent.box2d.bus_world import BusWorld

from gps.agent.box2d.bus_world import BUS_LENGTH, BUS_WIDTH

LANE_WIDTH = 20  #8           # width of lane in meters
INIT_VEL = 1.
CUT_MAP_SIZE = (48, 48)

class GPSMain(object):
	""" Main class to run algorithms and experiments. """
	def __init__(self, config, quit_on_end=False):
		"""
		Initialize GPSMain
		Args:
			config: Hyperparameters for experiment
			quit_on_end: When true, quit automatically on completion
		"""
		self._quit_on_end = quit_on_end
		self._hyperparams = config
		# print(config)
		self._conditions = config['common']['conditions']
		if 'train_conditions' in config['common']:
			self._train_idx = config['common']['train_conditions']
			self._test_idx = config['common']['test_conditions']
		else:
			self._train_idx = range(self._conditions)
			config['common']['train_conditions'] = config['common']['conditions']
			self._hyperparams=config
			self._test_idx = self._train_idx

		self._data_files_dir = config['common']['data_files_dir']
		self.agent = config['agent']['type'](config['agent'])
		self.data_logger = DataLogger()
		self.gui = GPSTrainingGUI(config['common']) if config['gui_on'] else None

		config['algorithm']['agent'] = self.agent
		# hard code to pass the map_state and target_state
		config['algorithm']['cost']['costs'][1]['data_types'][3]['target_state'] = config['agent']['target_state']
		config['algorithm']['cost']['costs'][1]['data_types'][3]['map_size'] = config['agent']['map_size']
		# config['algorithm']['cost']['costs'][1]['data_types'][3]['map_size'] = CUT_MAP_SIZE

		if len(config['algorithm']['cost']['costs'])>2:
			# temporarily deprecated, not considering collision cost
			# including cost_collision
			config['algorithm']['cost']['costs'][2]['data_types'][3]['target_state'] = config['agent']['target_state']
			config['algorithm']['cost']['costs'][2]['data_types'][3]['map_size'] = config['agent']['map_size']
			config['algorithm']['cost']['costs'][2]['data_types'][3]['map_state'] = config['agent']['map_state']
		# print(config['algorithm'])
		self.algorithm = config['algorithm']['type'](config['algorithm'])
		
		# Modified by RH
		self.finishing_time = None
		self.U = None
		self.final_pos = None
		self.samples = []
		self.quick_sample = None
		# self.map_size = config['agent']['map_size']
		self.map_size = CUT_MAP_SIZE
		self.display_center = config['agent']['display_center']

	def run(self, itr_load=None):
		"""
		Run training by iteratively sampling and taking an iteration.
		Args:
			itr_load: If specified, loads algorithm state from that
				iteration, and resumes training at the next iteration.
		Returns: None
		"""
		try:
			itr_start = self._initialize(itr_load)

			for itr in range(itr_start, self._hyperparams['iterations']):
				print("iter_num", itr)
				for cond in self._train_idx:
					for i in range(self._hyperparams['num_samples']):
						self._take_sample(itr, cond, i)

				traj_sample_lists = [
					self.agent.get_samples(cond, -self._hyperparams['num_samples'])
					for cond in self._train_idx
				]
				for cond in self._train_idx:
					self.samples.append(traj_sample_lists[cond].get_samples())
				# from sample_list .get_X() return one sample if given index, else return all 
				# Clear agent samples.
				self.agent.clear_samples()
				# The inner loop is done in _take_iteration (self.algorithm.iteration())
				# take iteration is like training
				self._take_iteration(itr, traj_sample_lists)
				# pol_sample_list is valid only for testing
				pol_sample_lists = self._take_policy_samples()
				self._log_data(itr, traj_sample_lists, pol_sample_lists) 

				if self.finishing_time:
					break
					# TODO: try to save and compare to find the minimal speed later after all iterations
					
				
			if not self.finishing_time:
				print("sorry, not hit")
				# TODO: find a relatively proper (nearest) sample, and return the final_pos by get_X(index)
				# assume one condition first
				# if collect samples then can only get last iteration
				self.samples = np.concatenate(self.samples)

		except Exception as e:
			traceback.print_exception(*sys.exc_info())
		finally:
			self._end()

	def test_policy(self, itr, N):
		"""
		Take N policy samples of the algorithm state at iteration itr,
		for testing the policy to see how it is behaving.
		(Called directly from the command line --policy flag).
		Args:
			itr: the iteration from which to take policy samples
			N: the number of policy samples to take
		Returns: None
		"""

		# modified by RH
		# originally, algo_itr, traj_itr, policy_itr are all the same
		# algo_itr = itr
		# traj_itr = itr
		# polilcy_itr = itr

		algo_itr = 18
		traj_itr = 1
		polilcy_itr = 18
		
		algorithm_file = self._data_files_dir + 'algorithm_itr_%02d.pkl' % itr
		self.algorithm = self.data_logger.unpickle(algorithm_file)
		if self.algorithm is None:
			print("Error: cannot find '%s.'" % algorithm_file)
			os._exit(1) # called instead of sys.exit(), since t
		traj_sample_lists = self.data_logger.unpickle(self._data_files_dir +
			('traj_sample_itr_%02d.pkl' % itr))
		print("algorithm")
		print(self.algorithm)
		print("traj_sample_lists")
		print(traj_sample_lists)
		print("N", N)
		pol_sample_lists = self._take_policy_samples(N)
		print("pol_sample_lists")
		print(pol_sample_lists)
		self.data_logger.pickle(
			self._data_files_dir + ('pol_sample_itr_%02d.pkl' % itr),
			copy.copy(pol_sample_lists)
		)

		if self.gui:
			self.gui.update(itr, self.algorithm, self.agent,
				traj_sample_lists, pol_sample_lists)
			self.gui.set_status_text(('Took %d policy sample(s) from ' +
				'algorithm state at iteration %d.\n' +
				'Saved to: data_files/pol_sample_itr_%02d.pkl.\n') % (N, itr, itr))

	def _initialize(self, itr_load):
		"""
		Initialize from the specified iteration.
		Args:
			itr_load: If specified, loads algorithm state from that
				iteration, and resumes training at the next iteration.
		Returns:
			itr_start: Iteration to start from.
		"""
		if itr_load is None:
			if self.gui:
				self.gui.set_status_text('Press \'go\' to begin.')
			return 0
		else:
			algorithm_file = self._data_files_dir + 'algorithm_itr_%02d.pkl' % itr_load
			self.algorithm = self.data_logger.unpickle(algorithm_file)
			if self.algorithm is None:
				print("Error: cannot find '%s.'" % algorithm_file)
				os._exit(1) # called instead of sys.exit(), since this is in a thread

			if self.gui:
				traj_sample_lists = self.data_logger.unpickle(self._data_files_dir +
					('traj_sample_itr_%02d.pkl' % itr_load))
				if self.algorithm.cur[0].pol_info:
					pol_sample_lists = self.data_logger.unpickle(self._data_files_dir +
						('pol_sample_itr_%02d.pkl' % itr_load))
				else:
					pol_sample_lists = None
				self.gui.set_status_text(
					('Resuming training from algorithm state at iteration %d.\n' +
					'Press \'go\' to begin.') % itr_load)
			return itr_load + 1

	def _take_sample(self, itr, cond, i):
		"""
		Collect a sample from the agent.
		Args:
			itr: Iteration number.
			cond: Condition number.
			i: Sample number.
		Returns: None
		"""
		if self.algorithm._hyperparams['sample_on_policy'] \
				and self.algorithm.iteration_count > 0:
			pol = self.algorithm.policy_opt.policy
		else:
			pol = self.algorithm.cur[cond].traj_distr
		if self.gui:
			self.gui.set_image_overlays(cond)   # Must call for each new cond.
			redo = True
			while redo:
				while self.gui.mode in ('wait', 'request', 'process'):
					if self.gui.mode in ('wait', 'process'):
						time.sleep(0.01)
						continue
					# 'request' mode.
					if self.gui.request == 'reset':
						try:
							self.agent.reset(cond)
						except NotImplementedError:
							self.gui.err_msg = 'Agent reset unimplemented.'
					elif self.gui.request == 'fail':
						self.gui.err_msg = 'Cannot fail before sampling.'
					self.gui.process_mode()  # Complete request.

				self.gui.set_status_text(
					'Sampling: iteration %d, condition %d, sample %d.' %
					(itr, cond, i)
				)
				# self.agent.sample(
				#     pol, cond,
				#     verbose=(i < self._hyperparams['verbose_trials'])
				# )
				new_sample = self.agent.sample(
					pol, cond,
					verbose=(i < self._hyperparams['verbose_trials'])
				)

				if self.gui.mode == 'request' and self.gui.request == 'fail':
					redo = True
					self.gui.process_mode()
					self.agent.delete_last_sample(cond)
				else:
					redo = False
		else:
			# self.agent.sample(
			#     pol, cond,
			#     verbose=(i < self._hyperparams['verbose_trials'])
			# )
			new_sample = self.agent.sample(
				pol, cond,
				verbose=(i < self._hyperparams['verbose_trials'])
			)
		if type(self.agent) == AgentBus and self.agent.finishing_time:
			# save the sample
			if (self.finishing_time is None) or (self.agent.finishing_time < self.finishing_time):
				self.finishing_time = self.agent.finishing_time
				print("agent_bus t= ", self.finishing_time)
				# for sample class, get_X\get_U methods return current and future ttimesteps
				self.U = new_sample.get_U()[:self.finishing_time]
				self.X = new_sample.get_X()[:self.finishing_time]
				self.final_pos = new_sample.get_X()[self.finishing_time]
				# TODO: pass map_size if necessary
				# print("return final_pos", [self.final_pos[0]+self.display_center[0], self.display_center[1]-self.final_pos[1], self.final_pos[2]])

	def _take_iteration(self, itr, sample_lists):
		"""
		Take an iteration of the algorithm.
		Args:
			itr: Iteration number.
		Returns: None
		"""
		if self.gui:
			self.gui.set_status_text('Calculating.')
			self.gui.start_display_calculating()
		self.algorithm.iteration(sample_lists)
		if self.gui:
			self.gui.stop_display_calculating()

	def _take_policy_samples(self, N=None):
		"""
		Take samples from the policy to see how it's doing.
		Args:
			N  : number of policy samples to take per condition
		Returns: None
		"""
		if 'verbose_policy_trials' not in self._hyperparams:
			# AlgorithmTrajOpt
			raise ValueError("Verbose absent")
			return None
		verbose = self._hyperparams['verbose_policy_trials']
		if self.gui:
			self.gui.set_status_text('Taking policy samples.')
		pol_samples = [[None] for _ in range(len(self._test_idx))]
		# Since this isn't noisy, just take one sample.
		# TODO: Make this noisy? Add hyperparam?
		# TODO: Take at all conditions for GUI?
		print("any sample in _take_policy_samples?")
		for cond in range(len(self._test_idx)):
			# for different cond
			# original code
			pol_samples[cond][0] = self.agent.sample(
			self.algorithm.policy_opt.policy, self._test_idx[cond],
			verbose=verbose, save=False, noisy=False)
			print(pol_samples[cond][0])
		return [SampleList(samples) for samples in pol_samples]

	def _log_data(self, itr, traj_sample_lists, pol_sample_lists=None):
		"""
		Log data and algorithm, and update the GUI.
		Args:
			itr: Iteration number.
			traj_sample_lists: trajectory samples as SampleList object
			pol_sample_lists: policy samples as SampleList object
		Returns: None
		"""
		if self.gui:
			self.gui.set_status_text('Logging data and updating GUI.')
			self.gui.update(itr, self.algorithm, self.agent,
				traj_sample_lists, pol_sample_lists)
			self.gui.save_figure(
				self._data_files_dir + ('figure_itr_%02d.png' % itr)
			)
		if 'no_sample_logging' in self._hyperparams['common']:
			return
		self.data_logger.pickle(
			self._data_files_dir + ('algorithm_itr_%02d.pkl' % itr),
			copy.copy(self.algorithm)
		)
		self.data_logger.pickle(
			self._data_files_dir + ('traj_sample_itr_%02d.pkl' % itr),
			copy.copy(traj_sample_lists)
		)
		if pol_sample_lists:
			self.data_logger.pickle(
				self._data_files_dir + ('pol_sample_itr_%02d.pkl' % itr),
				copy.copy(pol_sample_lists)
			)

	def _end(self):
		""" Finish running and exit. """
		if self.gui:
			self.gui.set_status_text('Training complete.')
			self.gui.end_mode()
			if self._quit_on_end:
				# Quit automatically (for running sequential expts)
				os._exit(1)			
		else:
			return
	
def get_policy(idx, map_size, x0, target, map_state=None, polygons=None, timestep=None, display_center=None):
	""" 
	Input: map_size, local map size, of the format (height, width)
		x0: current state at current idx, current timestep
		target: current target, can be the final one for the one for current idx, current timestep
		map_state: the state map of map_size
		polygons: previously used for passing the polygons for the whole route, now temporarily deprecated  (Mar 25)
			For local map, the polygons are approximated slightly below
	"""

	parser = argparse.ArgumentParser(description='Run the Guided Policy Search algorithm.')
	parser.add_argument('experiment', type=str,
						help='experiment name')
	parser.add_argument('-r', '--resume', metavar='N', type=int,
						help='resume training from iter N')
	parser.add_argument('-p', '--policy', metavar='N', type=int,
						help='take N policy samples (for BADMM/MDGPS only)')
	parser.add_argument('-s', '--silent', action='store_true',
						help='silent debug print outs')
	parser.add_argument('-q', '--quit', action='store_true',
						help='quit GUI automatically when finished')
	args = parser.parse_args()

	exp_name = args.experiment
	resume_training_itr = args.resume
	test_policy_N = args.policy

	from gps import __file__ as gps_filepath
	gps_filepath = os.path.abspath(gps_filepath)
	gps_dir = '/'.join(str.split(gps_filepath, '/')[:-3]) + '/'
	exp_dir = gps_dir + 'experiments/' + exp_name + '/'
	hyperparams_file = exp_dir + 'hyperparams.py'  # connect with a specific experiment directory

	if args.silent:
		logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
	else:
		logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

	if not os.path.exists(hyperparams_file):
		sys.exit("Experiment '%s' does not exist.\nDid you create '%s'?" %
				 (exp_name, hyperparams_file))
	# Python 2.7  'imp'
	# load the hyperparams setting from source file to an accesible module object
	hyperparams = imp.load_source('hyperparams', hyperparams_file)  # load new/existing file according to exp_name and then hyperparams_file

	min_idx = None

	import random
	import numpy as np
	import matplotlib.pyplot as plt
	
	seed = hyperparams.config.get('random_seed', 0)
	random.seed(seed)
	np.random.seed(seed)

	# save the image for one frame only, return the 48*48 state image for RL training
	# TODO: add map_number as a variable, should be consistent with random seed in map.py and data_files_dir in hyperparams.py
	data_files_dir = hyperparams.config['common']['data_files_dir']
	# Modify the hyperparameters according to the route, before passing the configuration to box2d world
	if map_state.any():
		hyperparams.config['agent']['map_state'] = map_state
	hyperparams.config['agent']['x0'] = np.asarray(x0)
	hyperparams.config['agent']['target_state'] = target
	hyperparams.config['agent']['polygons'] = polygons
	hyperparams.config['agent']['map_size'] = map_size
	hyperparams.config['agent']['display_center'] = display_center

	if test_policy_N:	
		print("test_policy_"+ str(test_policy_N))	
		data_filenames = os.listdir(data_files_dir)
		algorithm_prefix = 'algorithm_itr_'
		algorithm_filenames = [f for f in data_filenames if f.startswith(algorithm_prefix)]
		current_algorithm = sorted(algorithm_filenames, reverse=True)[0]
		current_itr = int(current_algorithm[len(algorithm_prefix):len(algorithm_prefix)+2]) # last two digits indicate the number of iterations
		# Manually set a number for testing, temporarily
		# current_itr = 12
		print("current_itr", current_itr)
		gps = GPSMain(hyperparams.config)
		if hyperparams.config['gui_on']:
			# test_policy = threading.Thread(
			# 	target=lambda: gps.test_policy(itr=current_itr, N=test_policy_N)
			# )
			# test_policy.daemon = True
			gps.test_policy(itr=current_itr, N=test_policy_N)
			# test_policy.start()
			# comment the following so that the program will not end and stuck after one round of GPS
			# plt.ioff()
			# plt.show()
		else:
			gps.test_policy(itr=current_itr, N=test_policy_N)
		
		if not gps.finishing_time:
			print("not hitting")

	
	else:
		min_target_dist = None
		
		gps = GPSMain(hyperparams.config, args.quit)
		if hyperparams.config['gui_on']:
			# Try threading, continue using if there's no big problem
			run_gps = threading.Thread(
			    target=lambda: gps.run(itr_load=resume_training_itr)
			)
			run_gps.daemon = True
			gps.run(itr_load=resume_training_itr)
			# comment the following so that the program will not end and stuck after one round of GPS
			# plt.ioff()
			# plt.show()
		else:
			gps.run(itr_load=resume_training_itr)

		if not gps.finishing_time:
			# did not reach
			for sample in gps.samples:
				X = sample.get_X()
				for i in range(len(X)-1,-1,-1):
					# TODO: want to loop over each point in each trajectory, see whether increase the weight of yaw error works or not first
					yaw_diff = min(500, 500*abs(X[i][2]-target[2]))
					target_dist = (X[i][0]-target[0])**2 + (X[i][1]-target[1])**2 + yaw_diff
					if target_dist< 1000 and ((min_target_dist is None) or (target_dist < min_target_dist)):
						# keep the resonable and smallest record 
						# print("target_dist", target_dist)
						min_target_dist = target_dist
						min_idx = i
						min_sample = sample
			if min_idx:
				# There's a reasonable small distance, though not reached.
				gps.finishing_time = min_idx
				gps.U = min_sample.get_U()[:gps.finishing_time]
				gps.X = min_sample.get_X()[:gps.finishing_time]
				gps.final_pos = min_sample.get_X()[gps.finishing_time]
				# print("return final_pos", [gps.final_pos[0]+display_center[0], display_center[1]-gps.final_pos[1], gps.final_pos[2]])  # map to Img
			gps.finishing_time = None

		if gps.U is None:
			print("None in gps.U")
		# else:
			# # save the action for current start_index and timestep
			# local_action_name = "local_action_{0}_{1}".format(idx, timestep)
			# local_action_name = os.path.join(data_files_dir, local_action_name) 
			# print("action to save", gps.U[0])                                              
			# # np.save(local_action_name, np.concatenate(gps.U[0]))
			# np.save(local_action_name, gps.U[0])
		
		
		return gps.finishing_time, gps.U, gps.final_pos, gps.X, data_files_dir
		

class Environment(object):
	def __init__(self):
		# load the map
		self.img_name = "map.jpg"
		self.crop_img = "local_map.jpg"
		self.map = Map(self.img_name, self.crop_img)
		self.route = self.map.route

def test_action(env, start_idx, final_idx, actions, polygons):
	# Create a world for testing
	route = copy.copy(env.route)
	map_size = copy.copy(env.map.map_size)
	pt0 = route[start_idx]
	# x0 = [pt0[0] + LANE_WIDTH/2*np.cos(pt0[2]+np.pi/2) , pt0[1] + LANE_WIDTH/2*np.sin(pt0[2]+np.pi/2) - ymin, pt0[2], INIT_VEL, INIT_VEL, 0.]
	x0 = [pt0[0]  , pt0[1] - env.map.ymin, pt0[2], INIT_VEL, INIT_VEL, 0.]
	pt1 = route[final_idx]
	# target = [ pt1[0] + LANE_WIDTH/2*np.cos(pt1[2]+np.pi/2) , pt1[1] + LANE_WIDTH/2*np.sin(pt1[2]+np.pi/2) - ymin , pt1[2]]
	target = [ pt1[0], pt1[1]-env.map.ymin, pt1[2]]
	display_center = set_display_center(x0, env.map.map_size, cut_map = False)
	x0 = np.hstack((x0[0] - display_center[0], display_center[1] - x0[1], x0[2:]))
	target = [target[0] - display_center[0], display_center[1] - target[1], target[2]]
	# print("target for test_action", target)
	test_world = BusWorld(x0=x0, target=target, render=True, polygons=polygons, map_size=map_size, route=route)
	if final_action is not None:
		test_world.run_test(actions)	


def collect_action(action_file, start_idx, final_idx, env, loop):
	# loop: 
	# 0:save action and img for all timesteps within one idx
	# 1: go through all indexes to test whether the bus can pass the route
	# 2: loop through indexes and save action and img for all timesteps in each index
	# use the uncut map but divide it into tiles
	if loop == 1:
		action_policy = []
		np.save(action_file, action_policy)
		step = 1
		idx = start_idx
		pt0 = env.route[idx]
		pt0 = np.hstack((pt0, [INIT_VEL, INIT_VEL, 0]))
	
		while (idx < final_idx):
			print("idx", idx)
			x0 = [pt0[0], pt0[1] - env.map.ymin, pt0[2], pt0[3], pt0[4], pt0[5]]
			pt1 = env.route[idx+1]
			target = [ pt1[0], pt1[1]-env.map.ymin, pt1[2]]
			print("get policy for ", x0, target) # x0 and target are wrt opencv system
			# dist = euclidDist(final_pos, target) # make use of the euclidDist in map.py
			# print("remaining distance", dist)

			display_center = set_display_center(x0, env.map.map_size, cut_map = True)
			# coords transformation, modify the coords wrt to display center TODO: put together and make functions for it
			x0 = np.hstack((x0[0] - display_center[0], display_center[1] - x0[1], x0[2:]))
			target = [target[0] - display_center[0], display_center[1] - target[1], target[2]]
			local_map_state = env.map.map_state[int(display_center[1]-CUT_MAP_SIZE[1]/2):int(display_center[1]+CUT_MAP_SIZE[1]/2), 
								int(display_center[0]-CUT_MAP_SIZE[0]/2):int(display_center[0]+CUT_MAP_SIZE[0]/2)].copy()
			# run gps and save img, save action
			# try to cut the map and place the bus based on its yaw angle
			finishing_time, U, final_pos, X, data_files_dir = get_policy(idx, CUT_MAP_SIZE, x0, target, map_state=local_map_state, polygons=None, display_center=display_center)
			if final_pos is None or (not final_pos.any()):
				# neither reach the target nor have enough speed to continue
				raise ValueError("cannot reach & speed is too low, please check")
			else:
				# save the action and recall the maps
				recall_map(data_files_dir, idx, CUT_MAP_SIZE, local_map_state, target, U, X, action_file)	
				final_pos = [final_pos[0]+display_center[0], display_center[1]-final_pos[1], final_pos[2], final_pos[3], -final_pos[4], 0]
				pt0 = final_pos

			if finishing_time:
				# find the next nearest appropriate target, i.e. especially for the curves, it may be easier to jump over a few targets
				print("new pt0: ", pt0)
				idx = idx + step
				while euclidDist(final_pos, env.route[idx+step])<5:
					# try to jump over some nearby points, without changing the map
					idx = idx + step
			else:
				# not reached, continue without updating the target
				continue

	elif loop == 0:
		# save the policy for a single step
		pt0 = env.route[start_idx]
		# coords transformation 	TODO: make functions to take care of all coords
		x0 = [pt0[0]  , pt0[1] - ymin, pt0[2], INIT_VEL, INIT_VEL, 0.]
		pt1 = env.route[start_idx+1]
		# coords transformation 	TODO: make functions to take care of all coords
		org_target = [ pt1[0], pt1[1]-ymin, pt1[2]]
		print("get policy for ", x0, org_target) # x0 and target are wrt opencv system
		final_pos = x0
		dist = euclidDist(final_pos, org_target) # make use of the euclidDist in map.py
		print("remaining distance", dist)
		timestep = 0
		
		while(dist>3):
			print("timestep", timestep)
			# TODO: save the corresponding occupancy map, later match with action by index or name		
			# try to cut the map and place the bus at one of the four corners based on its yaw angle
			display_center = set_display_center(x0, env.map.map_size, cut_map = True)
			# coords transformation, modify the coords wrt to display center TODO: put together and make functions for it
			x0[0] = x0[0] - display_center[0]
			x0[1] = display_center[1] - x0[1]
			target = [org_target[0] - display_center[0], display_center[1] - org_target[1], org_target[2]]
			local_map_state = env.map.map_state[int(display_center[1]-CUT_MAP_SIZE[1]/2):int(display_center[1]+CUT_MAP_SIZE[1]/2), 
								int(display_center[0]-CUT_MAP_SIZE[0]/2):int(display_center[0]+CUT_MAP_SIZE[0]/2)].copy()
			# run gps and save img, save action
			finishing_time, U, final_pos, X = get_policy(idx, CUT_MAP_SIZE, x0, target, map_state=local_map_state, polygons=None, display_center=display_center)
			if final_pos is None or (not final_pos.any()):
				# neither reach the target nor have enough speed to continue
				raise ValueError("cannot reach & speed is too low, please check")
			next_pos = X[1]			
			dist = euclidDist(next_pos, target) # make use of the euclidDist in map.py
			print("remaining distance", dist)
			# coords transformation
			# final_pos = [final_pos[0]+display_center[0],display_center[1]-final_pos[1], final_pos[2], final_pos[3], -final_pos[4], 0]	
			next_pos = [next_pos[0]+display_center[0],display_center[1]-next_pos[1], next_pos[2], next_pos[3], -next_pos[4], 0]		
			print("returned from get_policy", next_pos)
			x0 = next_pos
			timestep = timestep + 1
	elif loop == 2:
		# method1: run gps, save the action and map for the current timestep only and repeat the gps
		step = 1
		idx = start_idx
		pt0 = env.route[idx]
		pt0 = np.hstack((pt0, [INIT_VEL, INIT_VEL, 0]))
	
		while (idx < final_idx):
			print("idx", idx)
			# save the policy for a single step
			# coords transformation 	TODO: make functions to take care of all coords
			x0 = [pt0[0], pt0[1] - ymin, pt0[2], pt0[3], pt0[4], pt0[5]]
			pt1 = env.route[idx+1]
			# coords transformation 	TODO: make functions to take care of all coords
			org_target = [ pt1[0], pt1[1]-ymin, pt1[2]]
			print("get policy for ", x0, org_target) # x0 and target are wrt opencv system
			final_pos = x0
			dist = euclidDist(final_pos, org_target) # make use of the euclidDist in map.py
			print("remaining distance", dist)
			timestep = 0
			
			while(dist>3):
				print("timestep", timestep)
				# TODO: save the corresponding occupancy map, later match with action by index or name		
				# try to cut the map and place the bus at one of the four corners based on its yaw angle
				display_center = set_display_center(x0, env.map.map_size, cut_map = True)
				# coords transformation, modify the coords wrt to display center TODO: put together and make functions for it
				x0[0] = x0[0] - display_center[0]
				x0[1] = display_center[1] - x0[1]
				target = [org_target[0] - display_center[0], display_center[1] - org_target[1], org_target[2]]
				local_map_state = env.map.map_state[int(display_center[1]-CUT_MAP_SIZE[1]/2):int(display_center[1]+CUT_MAP_SIZE[1]/2), 
									int(display_center[0]-CUT_MAP_SIZE[0]/2):int(display_center[0]+CUT_MAP_SIZE[0]/2)].copy()
				# run gps and save img, save action
				finishing_time, U, final_pos, X = get_policy(idx, CUT_MAP_SIZE, x0, target, map_state=local_map_state, polygons=None,  display_center=display_center)
				if final_pos is None or (not final_pos.any()):
					# neither reach the target nor have enough speed to continue
					raise ValueError("cannot reach & speed is too low, please check")
				next_pos = X[1]			
				dist = euclidDist(next_pos, target) # make use of the euclidDist in map.py
				print("remaining distance", dist)
				# coords transformation
				# final_pos = [final_pos[0]+display_center[0],display_center[1]-final_pos[1], final_pos[2], final_pos[3], -final_pos[4], 0]	
				next_pos = [next_pos[0]+display_center[0],display_center[1]-next_pos[1], next_pos[2], next_pos[3], -next_pos[4], 0]		
				print("returned from get_policy", next_pos)
				x0 = next_pos
				timestep = timestep + 1

			# find the nearest appropriate target, i.e. especially for the curves, it may be easier to jump over a few targets
			pt0 = next_pos
			idx = idx + step
			while euclidDist(final_pos, env.route[idx+step])<5:
				# try to jump over some nearby points, without changing the map
				idx = idx + step


def recall_map(data_files_dir, idx, map_size, map_state, target, U, X, action_file, save_map=True):
	
	# save all actions within one file, easier for visualization
	action_policy = np.load(action_file)
	# np.reshape(action_policy,(-1, U.shape[1]))
	if len(action_policy) == 0:
		np.save(action_file, U)
	else:
		np.save(action_file, np.concatenate((action_policy, U)))
	
	# save the map for each tiemstep, with setting the interval to save space and to avoid redundancy
	org_map = map_state.copy()
	for timestep in range(len(U)):
		if timestep % 5 == 0:
			# save the action for current idx and timestep
			local_action_name = "local_action_{0}_{1}".format(idx, timestep)
			local_action_name = os.path.join(data_files_dir, local_action_name) 
			# print("action to save", U[timestep])                                              
			np.save(local_action_name, U[timestep])
			
			x0 = X[timestep]
			# print("x0 in recall_map", x0)
			# print("target in recall_map", target)
			# save the map for current idx and timestep
			local_map_name = "local_map_{0}_{1}.jpg".format(idx, timestep)
			img_name = os.path.join(data_files_dir, local_map_name)
			cv2.imwrite(img_name, map_state)		
			# cv2.waitKey(2000) # display image until a key is pressed
			# polygon fitting for map_state
			polygon_map = 'polygon_map.jpg'
			polygons = approx_polygon(img_name,polygon_map)
			# print("polygons")
			# print(polygons)
			img = cv2.imread(polygon_map)
			# cv2.imshow("polygon", img)
			# cv2.waitKey(3000)
			img_x0 = (x0[0]+map_size[1]/2, map_size[1]/2-x0[1])
			img_target = (target[0]+map_size[1]/2, map_size[1]/2-target[1])
			# print("img_x0, img_target", img_x0, img_target)
			img = draw_rot_rectangle(map_state, img_x0, x0[2], BUS_LENGTH, BUS_WIDTH, pixel=BUS)
			img = draw_rot_rectangle(map_state, img_target, target[2], BUS_LENGTH, BUS_WIDTH, pixel=TARGET)
			height, width = map_state.shape
			cv2.imwrite(img_name, img)
			# cv2.imshow("map with bus", img)
			# cv2.waitKey(3000)
			map_state = org_map.copy()

def set_display_center(x0, map_size, cut_map):
	# TODO: look into it to find how to collect the final policy after N iterations specified in `hyperparams.config 'iterations
	# print("x0[2]", x0[2])
	if cut_map:
		# try to cut the map and place the bus at one of four corners based on its yaw angle information
		if x0[2]>0 and x0[2]< 1.57:
			# 1st quadrant, put to lower left corner
			display_center=[x0[0]+CUT_MAP_SIZE[0]/4, x0[1]-CUT_MAP_SIZE[1]/4]
		if x0[2]>=1.57:
			# 2nd quadrant, put to lower right corner
			display_center=[x0[0]-CUT_MAP_SIZE[0]/4, x0[1]-CUT_MAP_SIZE[1]/4]
		if x0[2]<-1.57:
			# 3rd quadrant, put to upper right corner
			display_center=[x0[0]-CUT_MAP_SIZE[0]/4, x0[1]+CUT_MAP_SIZE[1]/4]
		if x0[2]>=-1.57 and x0[2]<=0:
			# 4th quadrant, put to upper left corner
			display_center=[x0[0]+CUT_MAP_SIZE[0]/4, x0[1]+CUT_MAP_SIZE[1]/4]
	else:
		# use the whole map
		display_center = [map_size[1]/2, map_size[0]/2]
	return display_center

if __name__ == "__main__":
	
	policy_list = []
	img_list = []
	# Generte the map of the whole route, env.map
	env = Environment()
	start_idx = 52
	final_idx = 53
	# final_idx = len(env.map.route)-1

	img_name = env.img_name 

	polygons = None
	# if_collect_action = False
	if_collect_action = True
	if_test_action = True
	# if_test_action = False
	action_file = "actions_tmp.npy"
	last_action_file = "actions.npy"

	loop = 1
	if if_collect_action:
		collect_action(action_file, start_idx, final_idx, env, loop)
	if if_test_action:
		final_action = np.load(action_file)
		test_action(env, start_idx, final_idx, final_action, polygons)




       