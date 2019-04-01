""" This file defines the collision cost, written by RH """
import copy

import numpy as np

from gps.algorithm.cost.config import COST_COLLISION
from gps.algorithm.cost.cost import Cost
from gps.algorithm.cost.cost_utils import evall1l2term, get_ramp_multiplier

# keep consistent with bus_world.py
BUS_LENGTH = 10 # 5.
BUS_WIDTH = 4 # 1.
# keep consistent with map.py
ROAD = 50 # 150

class CostCollision(Cost):
    """ Computes l1/l2 distance to a fixed target state. """
    def __init__(self, hyperparams):
        config = copy.deepcopy(COST_COLLISION)
        config.update(hyperparams)
        Cost.__init__(self, config)

    def eval(self, sample):
        """
        Evaluate cost function and derivatives on a sample.
        Args:
            sample:  A single sample
        """
        T = sample.T
        Du = sample.dU
        Dx = sample.dX

        final_l = np.zeros(T)
        final_lu = np.zeros((T, Du))
        final_lx = np.zeros((T, Dx))
        final_luu = np.zeros((T, Du, Du))
        final_lxx = np.zeros((T, Dx, Dx))
        final_lux = np.zeros((T, Du, Dx))

        for data_type in self._hyperparams['data_types']:
            # print("data_type", data_type)
            config = self._hyperparams['data_types'][data_type]
            wp = config['wp']
            x = sample.get(data_type)
            _, dim_sensor = x.shape
            # print("x in cost_collision", x.shape)
            # print(x)

            wpm = get_ramp_multiplier(
                self._hyperparams['ramp_option'], T,
                wp_final_multiplier=self._hyperparams['wp_final_multiplier']
            )
            wp = wp * np.expand_dims(wpm, axis=-1)
            # Compute state penalty.

            # TODO
            # create a state map with all polygons represented as 1
            # draw a corresponding box to represent the bus, and then use the dot multiplication of overlapping to indicate collision loss
            map_size = config['map_size']
            map_state = config['map_state']
            # print("map_size for collision", map_size)
            
            target_state = config['target_state']
            # print(target_state)
            # print("is target on road", map_state[ int(target_state[1]), int(target_state[0])])
            target_state = [target_state[0]-map_size[1]/2, map_size[0]/2-target_state[1], target_state[2]]
            # print("target for cost_collision")
            # print(target_state)

            dist = np.zeros(x.shape)

            for i in range(len(x)):
                # find the rectangle bus, given the center position
                x1 = int(BUS_LENGTH/2*np.cos(x[i][2]) + BUS_WIDTH/2*np.sin(x[i][2])+x[i][0])
                x2 = int(BUS_LENGTH/2*np.cos(x[i][2]) - BUS_WIDTH/2*np.sin(x[i][2])+x[i][0])
                x3 = int(-BUS_LENGTH/2*np.cos(x[i][2]) + BUS_WIDTH/2*np.sin(x[i][2])+x[i][0])
                x4 = int(-BUS_LENGTH/2*np.cos(x[i][2]) - BUS_WIDTH/2*np.sin(x[i][2])+x[i][0])
                y1 = int(BUS_LENGTH/2*np.sin(x[i][2]) + BUS_WIDTH/2*np.cos(x[i][2])+x[i][1])
                y2 = int(BUS_LENGTH/2*np.sin(x[i][2]) - BUS_WIDTH/2*np.cos(x[i][2])+x[i][1])
                y3 = int(-BUS_LENGTH/2*np.sin(x[i][2]) + BUS_WIDTH/2*np.cos(x[i][2])+x[i][1])
                y4 = int(-BUS_LENGTH/2*np.sin(x[i][2]) - BUS_WIDTH/2*np.cos(x[i][2])+x[i][1])
                xmin = min(x1, x2, x3, x4)
                xmax = max(x1, x2, x3, x4)
                ymin = min(y1, y2, y3, y4)
                ymax = max(y1, y2, y3, y4)
                # simplify the overlapping as the sum of four endpoints of a bounding box
                # print(xmin, xmax, ymin, ymax) # which are based on box2D coords
                xmin = xmin + map_size[1]/2
                xmax = xmax + map_size[1]/2
                ymin = map_size[0]/2 - ymin
                ymax = map_size[0]/2 - ymax
                # print(xmin, xmax, ymin, ymax)
                # print(map_state[xmin, ymin], map_state[xmin, ymax], map_state[xmax, ymin], map_state[xmax, ymax] )
                dist_temp = map_state[ymin, xmin] + map_state[ymax, xmin]+ map_state[ymin, xmax]+ map_state[ymax, xmax] - 4*ROAD
                # print("dist", dist_temp)
                dist[i] = [dist_temp, dist_temp, 0]

            # Evaluate penalty term.
            l, ls, lss = evall1l2term(
                wp, dist, np.tile(np.eye(dim_sensor), [T, 1, 1]),
                np.zeros((T, dim_sensor, dim_sensor, dim_sensor)),
                self._hyperparams['l1'], self._hyperparams['l2'],
                self._hyperparams['alpha']
            )

            final_l += l

            sample.agent.pack_data_x(final_lx, ls, data_types=[data_type])
            sample.agent.pack_data_x(final_lxx, lss,
                                     data_types=[data_type, data_type])
        # print("return collision_cost")
        # print(final_l, final_lx, final_lu, final_lxx, final_luu, final_lux)
        # print("dsit_temp", dist_temp)
        # print("cost_collisoion", final_l[0])
        return final_l, final_lx, final_lu, final_lxx, final_luu, final_lux