#!/usr/bin/python3
import os
from math import degrees, acos
from log_manager import Log_Manager
from common import *
from config import *

Log = Log_Manager()


class Joint_Degree_Calculator():
    # distance mm from previous link
    #  current settings : x y z = width(x) depth(y) height(z)
    ARM = {
        'BASE_LINK': {'w': 0.0, 'h': 0.0, 'd': 0.0},
        '5': {'w': 0.0, 'h': 48.2, 'd': 0.0},
        '4': {'w': 0.0, 'h': 31.0, 'd': 15.4},
        '3': {'w': 0.0, 'h': 61.5 + 50.0, 'd': 0.0},
        '2': {'w': 0.0, 'h': 61.5 + 50.0, 'd': 0.0},
        '1': {'w': 36.0, 'h': 2.0, 'd': -25.6},
        '0': {'w': 15.0, 'h': 65.8, 'd': 0.0},
        'BASE_LINK_4': {'w': 0.0, 'h': 48.2 + 31.0, 'd': 15.4},  # BASE_LINK to _4
        '1_0': {'w': 36.0 + 15.0, 'h': 15.0 + 65.8, 'd': -25.6 + 0.0},  # _1 to _0
    }
    GRIP_STATES = {'ch00': {'open': 98, 'close': 124},
                   'ch06': {'open': 85, 'close': 56}}
    adjust_deg_for_y = 0.0
    DEBUG_CH = 'jdc'
    IS_DEV = False

    # DEG_ADJUSTMENT = 90.0

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))

    def calc_node_dists(self, xyz: [int, int, int]) -> {float, float, float, float, float, float}:
        Log.intervally(self.DEBUG_CH, 20, '[Joint] data {}'.format(xyz))
        [x, y, z] = [float(val) for val in xyz]
        _2_x = x - self.ARM['1_0']['w']
        _2z_len = sqr_theorem(_2_x, z)
        _2yz_len = sqr_theorem(_2z_len, y)
        node_dists = {'x': x, 'y': y, 'z': z, '_2x': _2_x, '_2z': _2z_len, '_2yz': _2yz_len, }
        Log.intervally(self.DEBUG_CH, 40, '[node_dists] {}'.format(node_dists))
        return node_dists

    def calc_node_degs(self, node_dists: dict, arm_state='TARGETING', is_dev=False) -> {int, int, int, int, int, int,
                                                                                        int}:
        self.IS_DEV = is_dev
        [ch02, ch03, ch04] = self._calc_xy_surface_degs(node_dists)
        [ch01, ch05] = self._calc_XZ_surface_degs(node_dists)
        ch00, ch06 = self._calc_grip_degs(arm_state)
        node_degs = {ch: deg for ch, deg in zip(CHS, [ch00, ch01, ch02, ch03, ch04, ch05, ch06])}
        Log.intervally(self.DEBUG_CH, 40, '[node_degs] {}'.format(node_degs))
        return node_degs

    def _calc_xy_surface_degs(self, node_dists) -> [int, int, int]:
        [cos_theta_2, cos_theta_3, cos_theta_4, cos_theta_y] = self.__calc_thetas(node_dists)

        self.adjust_deg_for_y = round_3(degrees(acos(cos_theta_y)))
        self.adjust_deg_for_y = self.adjust_deg_for_y if node_dists['y'] >= 0.0 else -1.0 * self.adjust_deg_for_y
        ch03 = degrees(acos(cos_theta_3)) + self.adjust_deg_for_y
        ch04 = degrees(acos(cos_theta_4)) + self.adjust_deg_for_y
        ch02 = 540.0 - (ch03 + (ch04 + 90.0) + 180.0 + 90.0)
        return [int(ch) for ch in [ch02, ch03, ch04]]

    def _calc_XZ_surface_degs(self, node_dists) -> [int, int]:
        Log.intervally(self.DEBUG_CH, 20,
                       '[dists] _2z, z, x {}'.format([node_dists['_2z'], node_dists['z'], node_dists['x']]))
        cos_theta_5 = cos_theorem(node_dists['_2z'], node_dists['z'], node_dists['_2x'])
        ch05 = degrees(acos(cos_theta_5))
        ch01 = ch05
        return [int(ch) for ch in [ch01, ch05]]

    def _calc_grip_degs(self, arm_state: str) -> [int, int]:
        if arm_state in ['GRIP-CLOSED', 'LIFTING_UP', 'REACHING']:
            return self.GRIP_STATES['ch00']['close'], self.GRIP_STATES['ch06']['close']
        else:
            return self.GRIP_STATES['ch00']['open'], self.GRIP_STATES['ch06']['open']

    def __calc_thetas(self, node_dists) -> [int, int, int, int]:
        cos_theta_2 = cos_theorem(self.ARM['2']['h'], node_dists['_2yz'], self.ARM['3']['h'])
        cos_theta_3 = cos_theorem(self.ARM['2']['h'], self.ARM['3']['h'], node_dists['_2yz'])
        cos_theta_4 = cos_theorem(node_dists['_2yz'], self.ARM['3']['h'], self.ARM['2']['h'])
        cos_theta_y = cos_theorem(node_dists['_2yz'], node_dists['_2z'], node_dists['y'])
        if self.IS_DEV: print({'2': cos_theta_2, '3': cos_theta_3, '4': cos_theta_4, 'y': cos_theta_y, })
        return [cos_theta_2, cos_theta_3, cos_theta_4, cos_theta_y]


if __name__ == '__main__':
    joint_degree = Joint_Degree_Calculator()

    test_data = [[157.0, 10.0, 0.0], [157.0, 0.0, 0.0], [157.0, -10.0, 0.0]]

    for data in test_data:
        print('\n*****')
        params = joint_degree.calc_node_dists(data)
        print(params)

        joint_degrees = joint_degree.calc_node_degs(params, is_dev=True)

        for p_key, d_key in zip(params, joint_degrees):
            print('{:8}'.format(p_key), '{:5}     '.format(params[p_key]), '{:2}'.format(d_key),
                  '{:5}'.format(joint_degrees[d_key]))
