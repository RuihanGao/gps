""" Hyperparameters for Box2D Vehicle """
from __future__ import division

import os.path
from datetime import datetime
import numpy as np 

from gps import __file__ as gps_filepath
from gps.agent.box2d.agent_box2d import AgentBox2D 
from gps.agent.box2d.vehicle_world import VehicleWorld 
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt 
from gps.algorithm.cost.cost_state import CostState 
from gps.algorithm.cost.cost_action import CostAction 
from gps.algorithm.cost.cost_sum import CostSum 
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior 
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM 
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython 
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.gui.config import generate_experiment_info
from gps.proto.gps_pb2 import END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION 
from gps.agent.box2d.traffic import Traffic 
from gps.agent.box2d.map import *

SENSOR_DIMS = {
	END_EFFECTOR_POINTS: 3,
	END_EFFECTOR_POINT_VELOCITIES: 3,
	ACTION: 2
}  # END_EFFECTOR [x, y, yaw, dx, dy, dyaw]  ACTION [a, delta]

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/box2d_vehicle_example/'

common = {
	'experiment_name': 'box2d_vehicle_example' + '_' + \
			datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
	'experiment_dir': EXP_DIR,
	'data_files_dir': EXP_DIR + 'data_files/',
	'log_filename': EXP_DIR + 'log.txt',
	'conditions': 1,  # number of different world setting
}

if not os.path.exists(common['data_files_dir']):
	os.makedirs(common['data_files_dir'])

agent = {
	'type': AgentBox2D,
	'target_state': np.array([20, 20, 0]),
	'world': VehicleWorld,
	# 'render': False,
	'render': True,   # False is much faster than True
	# 'x0': np.array([0, 0, 0, 0, 0, 0]),
	'x0':[0, 0, 0, 0, 0, 0],
	#TODO: what is 'rk'?
	'rk': 0,
	'dt': 0.05,
	'substeps': 1,
	'conditions': common['conditions'],
	'pos_body_idx': np.array([]),
	'pos_body_offset': np.array([]),
	'T': 100,  # horizon
	'sensor_dims': SENSOR_DIMS,
	'state_include': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
	'obs_include': [],
}

algorithm = {
	'type': AlgorithmTrajOpt,
	'conditions': common['conditions'],
}

algorithm['init_traj_distr'] = {
	'type': init_lqr,
	'init_gains': np.zeros(SENSOR_DIMS[ACTION]),
	'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
	'init_var': 0.1,
	'stiffness': 0.01,
	'dt': agent['dt'],  # pass the param to keep consistency
	'T': agent['T'],
}

action_cost = {
	#TODO: check how the cost is expressed
	'type': CostAction,
	'wu': np.array([1, 1]), # torque penalty, in cost.config
}

state_cost = {
	'type': CostState,
	#TODO: check how to express data_types
	'data_types': {
		END_EFFECTOR_POINTS:{
            # 'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
            'wp': np.array([1, 1, 0.1]),
            'target_state': agent["target_state"],
		},
	},
}

algorithm['cost'] = {
	'type': CostSum,
	'costs': [action_cost, state_cost],
	'weights': [1e-5, 1.0],
}

algorithm['dynamics'] = {
	#TODO: check how the Prior and GMM work
	'type': DynamicsLRPrior,
	'regularization': 1e-6,
	'prior': {
		'type': DynamicsPriorGMM,
		'max_clusters': 20,
		'min_samples_per_cluster': 40,
		'max_samples': 20,
	},
}

algorithm['traj_opt'] = {
	#TODO: check the type, how it works
	'type': TrajOptLQRPython,
}

algorithm['policy_opt'] = {}

config = {
	'iterations': 50,
	'num_samples': 10,
	'verbose_trials': 5,
	'common': common,
	'agent': agent,
	'gui_on': True,
	'algorithm': algorithm,
}


#TODO: check why modify after config{}
common['info'] = generate_experiment_info(config)