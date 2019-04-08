""" Hyperparameters for Box2D Vehicle """
from __future__ import division

import os.path
from datetime import datetime
import numpy as np 

from gps import __file__ as gps_filepath
from gps.agent.box2d.agent_bus import AgentBus
from gps.agent.box2d.bus_world import BusWorld 
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt 
from gps.algorithm.algorithm_badmm import AlgorithmBADMM
from gps.algorithm.cost.cost_state import CostState 
from gps.algorithm.cost.cost_action import CostAction 
from gps.algorithm.cost.cost_collision import CostCollision
from gps.algorithm.cost.cost_sum import CostSum 
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior 
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM 
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython 
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.algorithm.policy.policy_prior_gmm import PolicyPriorGMM
from gps.algorithm.policy_opt.policy_opt_tf import PolicyOptTf
from gps.algorithm.policy_opt.tf_model_example import tf_network
from gps.algorithm.policy_opt.tf_model_example import multi_modal_network
from gps.gui.config import generate_experiment_info
from gps.proto.gps_pb2 import END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, RGB_IMAGE, RGB_IMAGE_SIZE
from gps.agent.box2d.traffic import Traffic 
from gps.agent.box2d.map import *


IMAGE_WIDTH = 48
IMAGE_HEIGHT = 48
IMAGE_CHANNELS = 1
BUS_GAINS = np.array([1, 0.1])

SENSOR_DIMS = {
	END_EFFECTOR_POINTS: 3,
	END_EFFECTOR_POINT_VELOCITIES: 3,
	ACTION: 2
}  # END_EFFECTOR [x, y, yaw, dx, dy, dyaw]  ACTION [a, delta]

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/box2d_bus_example/'

common = {
	'experiment_name': 'box2d_bus_example' + '_' + \
			datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
	'experiment_dir': EXP_DIR,
	'data_files_dir': EXP_DIR + 'map_13_/' + 'data_files/',
	'log_filename': EXP_DIR + 'map_13_/' + 'log.txt',
	'conditions': 1,  # number of different world setting,
	'iterations': 15,
	'train_conditions': [0],
    'test_conditions': [0],

}

if not os.path.exists(common['data_files_dir']):
	os.makedirs(common['data_files_dir'])

agent = {
	'type': AgentBus,
	'target_state': np.array([20., 20., 0.]),
	'world': BusWorld,
	'render': False,
	# 'render': True,    # False is much faster than True
	'x0': np.array([0., 0., 0., 1., 1., 0.]),
	# 'x0': get_x0_from_route()
	#TODO: what is 'rk'?
	'rk': 0,
	'dt': 0.05,
	'substeps': 1,
    'conditions': common['conditions'],
    'iterations': common['iterations'],
	'train_conditions': common['train_conditions'],
    'test_conditions': common['test_conditions'],
	'pos_body_idx': np.array([]),
	'pos_body_offset': np.array([]),
	'T': 200,  # horizon
	'sensor_dims': SENSOR_DIMS,
	'state_include': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
	'obs_include': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES], # equivalent to self.obs_data_types which is used to match data_type in sample.get_obs()
	'polygons': None,
	'map_size': None,
	'map_state': None,
	'display_center': None,
}



algorithm = {
	'type': AlgorithmTrajOpt,  
	'conditions': common['conditions'],
    'iterations': common['iterations'],
    'train_conditions': common['train_conditions'],
    'test_conditions': common['test_conditions'],
}

# algorithm = {
#     'type': AlgorithmBADMM,
#     'conditions': common['conditions'],
#     'iterations': common['iterations'],
#     'train_conditions': common['train_conditions'],
#     'test_conditions': common['test_conditions'],
#     'lg_step_schedule': np.array([1e-4, 1e-3, 1e-2, 1e-2]),
#     'policy_dual_rate': 0.2,
#     'ent_reg_schedule': np.array([1e-3, 1e-3, 1e-2, 1e-1]),
#     'fixed_lg_step': 3,
#     'kl_step': 5.0,
#     'min_step_mult': 0.01,
#     'max_step_mult': 1.0,
#     'sample_decrease_var': 0.05,
#     'sample_increase_var': 0.1,
# }
# algorithm['policy_opt'] = {
# 	'type':PolicyOptTf,
# 	'network_params': {
# 		'num_filters': [64, 32, 32],
# 		# TODO: in proto, only have RGB_IMAGE, DEPTH_IMAGE, CONTEXT_IMAGE, need to make a better choice later
# 		# use the image later, try to run the tf network first
# 		# 'obs_include': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, RGB_IMAGE],
# 		'obs_include': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
# 		'obs_vector_data': [END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
# 		# # 'obs_image_data': [RGB_IMAGE],
#         # 'image_width': IMAGE_WIDTH,
#         # 'image_height': IMAGE_HEIGHT,
#         # 'image_channels': IMAGE_CHANNELS,
# 		'sensor_dims': SENSOR_DIMS,
# 	},
# 	# 'network_model': multi_modal_network,
# 	'network_model': tf_network,
# 	'iterations': 1000,
# 	'weights_file_prefix': EXP_DIR + 'policy',
# }

# algorithm['policy_prior'] = {
#     'type': PolicyPriorGMM,
#     'max_clusters': 20,
#     'min_samples_per_cluster': 40,
#     'max_samples': 20,
# }


algorithm['init_traj_distr'] = {
	'type': init_lqr,
	'init_gains': 1/BUS_GAINS,
	'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
	'init_var': 0.1,
	'stiffness': 0.01,
	'stiffness_vel': 0.05,
	'dt': agent['dt'],  # pass the param to keep consistency
	'T': agent['T'],
}
action_cost = {
	'type': CostAction,
	'wu': np.array([1, 1]), # torque penalty, in cost.config
}

state_cost = {
	'type': CostState,
	'data_types': {
		END_EFFECTOR_POINTS:{
            # 'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
            'wp': np.array([1, 1, 0.1]),
            # 'target_state': algorithm['agent']._hyperparams["target_state"],
			'target_state': None,
		},
	},
}

collision_cost = {
	'type': CostCollision,
	'data_types': {
		END_EFFECTOR_POINTS:{
            # 'wp': np.ones(SENSOR_DIMS[END_EFFECTOR_POINTS]),
            'wp': np.array([1, 1, 0.1]),
			# 'target_state': algorithm['agent']._hyperparams["target_state"],
			'target_state': None,
            # 'map_state': algorithm['agent']._hyperparams["map_state"],
			'map_state': None,
		},
	},
}


algorithm['cost'] = {
	'type': CostSum,
	# 'costs': [action_cost, state_cost, collision_cost],
	# 'weights': [1e-5, 1.0, 1.0],
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
		'max_samples': 50,  # 20
	},
}

algorithm['traj_opt'] = {
	#TODO: check the type, how it works
	'type': TrajOptLQRPython,
	# 'type': TrajOptPILQR, # not working due to data type
}

config = {
    'iterations': algorithm['iterations'],
	'num_samples': 10,
	'verbose_trials': 5,
	# comment the below to use TrajOpt only, uncomment for PolicyOpt
	# 'verbose_policy_trials': 1,
	'common': common,
	'agent': agent,
	'gui_on': False,
	# 'gui_on': True,
	'algorithm': algorithm,	
}

#TODO: check why modify after config{}
common['info'] = generate_experiment_info(config)	