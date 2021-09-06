#!/usr/bin/python3
import os
from math import degrees, acos
from log_manager import LogManager
from common import Common as cmn
from config import *

Log = LogManager()


class JointDegreeCalculator:
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
    # GRIP_STATES = {'ch00': {'open': 98, 'close': 124},
    #                'ch06': {'open': 85, 'close': 56}}
    adjust_deg_for_y = 0.0
    DEBUG_CH = 'jdc'
    IS_DEV = bool(os.getenv('is_dev'))

    # DEG_ADJUSTMENT = 90.0
    ADJUST_VALUE = 10.0

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))

    def calc_node_mms(self, xyz: [int, int, int]) -> {float, float, float, float, float, float}:
        Log.only(self.DEBUG_CH, 20, '[Joint] data {}'.format(xyz))
        [x, y, z] = [float(val) for val in xyz]
        x += self.ADJUST_VALUE
        _2_x = x - self.ARM['1_0']['w']
        _2z_len = cmn.sqr_theorem(_2_x, z)
        _2yz_len = cmn.sqr_theorem(_2z_len, y)
        node_mms = {'x': x, 'y': y, 'z': z, '_2x': _2_x, '_2z': _2z_len, '_2yz': _2yz_len, }
        Log.only(self.DEBUG_CH, 40, '[node_dists] {}'.format(node_mms))
        return node_mms

    @staticmethod
    def _up(xyz: list):
        return [xyz[0], 00.0, xyz[2]]

    @staticmethod
    def _down(xyz: list):
        return [xyz[0], -20.0, xyz[2]]

    @staticmethod
    def _leave(xyz: list):
        return INIT_POS

    @staticmethod
    def _stay(xyz: list):
        return xyz

    def adjust_xyz_by_state(self, xyz: list, arm_state, label):
        _ARM_POS_LIST = [self._up, self._stay, self._down, self._up, self._stay, self._stay, self._leave]
        ARM_POS_LIST = {key: pos for key, pos in zip(ARM_STATES, _ARM_POS_LIST)}
        return ARM_POS_LIST[arm_state](xyz)

    def calc_node_degs(self, node_dists: dict, arm_state='TARGETING', is_dev=False) \
            -> {int, int, int, int, int, int, int}:
        self.IS_DEV = is_dev
        [ch02, ch03, ch04] = self._calc_xy_surface_degs(node_dists)
        [ch01, ch05] = self._calc_xz_surface_degs(node_dists)
        ch00, ch06 = self._calc_grip_degs(arm_state)
        _degs = [ch00, ch01, ch02, ch03, ch04, ch05, ch06]
        _degs = _degs[:len(CHS)]
        node_degs = {ch: deg for ch, deg in zip(CHS, _degs)}
        Log.only(self.DEBUG_CH, 40, '[node_degs] {}'.format(node_degs))
        return node_degs

    def _calc_xy_surface_degs(self, node_dists) -> [int, int, int]:
        [cos_theta_3, cos_theta_4, cos_theta_y] = self.__calc_thetas(node_dists)
        self.adjust_deg_for_y = cmn.round_3(degrees(acos(cos_theta_y)))
        self.adjust_deg_for_y = self.adjust_deg_for_y if node_dists['y'] >= 0.0 else -1.0 * self.adjust_deg_for_y
        Log.only(self.DEBUG_CH, 20, 'cos_theta_3 adjust_deg_for_y {}'.format([cos_theta_3, self.adjust_deg_for_y]))
        ch03 = degrees(acos(cos_theta_3)) + self.adjust_deg_for_y
        ch04 = degrees(acos(cos_theta_4)) + self.adjust_deg_for_y
        ch02 = 540.0 - (ch03 + (ch04 + 90.0) + 180.0 + 90.0)
        return [int(ch) for ch in [ch02, ch03, ch04]]

    def _calc_xz_surface_degs(self, node_dists) -> [int, int]:
        f = '[dists] _2z, z, x {}'
        Log.only(self.DEBUG_CH, 20, f.format([node_dists['_2z'], node_dists['z'], node_dists['x']]))
        cos_theta_5 = cmn.cos_theorem(node_dists['_2z'], node_dists['z'], node_dists['_2x'])
        ch05 = degrees(acos(cos_theta_5))
        ch01 = ch05
        return [int(ch) for ch in [ch01, ch05]]

    def _calc_grip_degs(self, arm_state: str) -> [int, int]:
        if arm_state in ['GRIP-CLOSED', 'LIFTING_UP', 'REACHING']:
            return GRIP_DEGS['ch00']['close'], GRIP_DEGS['ch06']['close']
        else:
            return GRIP_DEGS['ch00']['open'], GRIP_DEGS['ch06']['open']

    def __calc_thetas(self, node_dists) -> [int, int, int, int]:
        # cos_theta_2 = cos_theorem(self.ARM['2']['h'], node_dists['_2yz'], self.ARM['3']['h'])
        cos_theta_3 = cmn.cos_theorem(self.ARM['2']['h'], self.ARM['3']['h'], node_dists['_2yz'])
        cos_theta_4 = cmn.cos_theorem(node_dists['_2yz'], self.ARM['3']['h'], self.ARM['2']['h'])
        cos_theta_y = cmn.cos_theorem(node_dists['_2yz'], node_dists['_2z'], node_dists['y'])
        Log.only(self.DEBUG_CH, 40, 'cos345 {}'.format({'3': cos_theta_3, '4': cos_theta_4, 'y': cos_theta_y, }))
        return [cos_theta_3, cos_theta_4, cos_theta_y]

    def __del__(self):
        print('***** Finalize {}'.format(__class__.__name__))


if __name__ == '__main__':
    jdc = JointDegreeCalculator()

    _xyz = [10.0, 0.0, 0.0]
    xyz = jdc.adjust_xyz_by_state(_xyz, 'APPROACHING', None)
    print(xyz)
