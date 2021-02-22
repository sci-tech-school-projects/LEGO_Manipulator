#!/usr/bin/env python3
import os
from math import degrees, acos, floor
import logging

logger = logging.getLogger('LoggingTest')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Joint_Degree_Calculator():
    # xyz = depth(Red), width(Green), height(Blue)
    # relative joint position from later position to self position
    # xyz_theta
    # scale : mm
    # pro SG90
    # rel_pos = {
    #     'base_link': [0.0, 0.0, 0.0, 0.0],
    #     '_5': [0.0, 0.0, 24.0, 0.0],
    #     '_4': [0.0, 0.0, 25.0, 0.0],
    #     '_3': [0.0, 0.0, 44.0, 0.0],
    #     '_2': [0.0, 0.0, 44.0, 0.0],
    #     '_1': [0.0, 0.0, 38.0, 0.0],
    #     '_0': [24.0, 0.0, 58.0, 0.0],
    #     '_base_link_4': [0.0, 0.0, 49.0, 0.0],  # base_link to _4
    #     '_2_0': [24.0, 0.0, 96.0, 0.0],  # _2 to _0
    # }

    # 996
    rel_pos = {
        'base_link': [0.0, 0.0, 0.0, 0.0],
        '_5': [0.0, 0.0, 48.2, 0.0],
        '_4': [0.0, 0.0, 31.0, 0.0],
        '_3': [0.0, 0.0, 61.5, 0.0],
        '_2': [0.0, 0.0, 61.5, 0.0],
        '_1': [0.0, 0.0, 50.6, 0.0],
        '_0': [24.0, 0.0, 57.5, 0.0],
        '_base_link_4': [0.0, 0.0, 79.2, 0.0],  # base_link to _4
        '_2_0': [24.0, 0.0, 169.6, 0.0],  # _2 to _0
    }

    coordinate_after_seconds = [0.0, 0.0, 0.0]

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))

    def Get_Lengths(self, xyz=None):
        logger.log(20, '[INFO] data {}'.format(xyz))
        rel_pos, target_xyz = self._generate_data(self.rel_pos, xyz, self.coordinate_after_seconds)
        distance = self._calc_XZ_planar_distance(target_xyz)

        x, y, z = target_xyz
        w = distance
        h = rel_pos['_2_0'][2] - rel_pos['_base_link_4'][2] + z
        l = (w ** 2 + h ** 2) ** (1 / 2)
        lengths = {'x': x, 'y': y, 'z': z, 'w': w, 'h': h, 'l': l,
                   'l2': rel_pos['_2'][2],
                   'l3': rel_pos['_3'][2]}
        logger.log(20, '[INFO] lengths {}'.format(lengths))

        return lengths

    @staticmethod
    def _generate_data(rel_pos, xyz, coordinate_after_seconds):
        target_xyz = [50.0 - rel_pos['_0'][0], 0.0, 0.0]  # if None
        if xyz != None:
            x = float(xyz[0]) - rel_pos['_0'][0] + coordinate_after_seconds[0]
            target_xyz = [x, float(xyz[1]), float(xyz[2])]
        return rel_pos, target_xyz

    def _calc_XZ_planar_distance(self, target_xyz):
        w = target_xyz[0]
        h = target_xyz[1]
        distance = (w ** 2 + h ** 2) ** (1 / 2)
        return distance

    def Get_Thetas(self, lengths, arm_status='targeting'):
        logger.log(20, '[INFO] data {}'.format(lengths))
        x, y, z, w, h, l, l2, l3 = self.__expand_dict(lengths)
        XY_thetas = self._calc_XY_planar_theta(w, l, l2, l3)
        XZ_thetas = self._calc_XZ_planar_theta(w, z, x)

        joint_degrees = {
            'ch00': self._calc_grip_theta(arm_status),
            'ch01': int(floor(XZ_thetas[0])),
            'ch02': int(floor(XY_thetas[0])),
            'ch03': int(floor(XY_thetas[1])),
            'ch04': int(floor(XY_thetas[2])),
            'ch05': int(floor(XZ_thetas[1])),
        }
        logger.log(30, '[INFO] joint_degrees {}'.format(joint_degrees.values()))

        return joint_degrees

    @staticmethod
    def __expand_dict(lengths):
        array = []
        for key in lengths.keys():
            array.append(lengths[key])
        return array

    def _calc_XY_planar_theta(self, w, l, l2, l3):
        cos_theta_1_2 = self.cosine_theorem(l2, l, l3)
        cos_theta_2_3 = self.cosine_theorem(l2, l3, l)
        cos_theta_3_4 = self.cosine_theorem(l, l3, l2)

        Theta = degrees(acos(w / l))
        logger.log(10, '[INFO] cos_theta_1_2, Theta {} {}'.format(cos_theta_1_2, Theta))
        Theta_1_2 = ((degrees(acos(cos_theta_1_2)) + (90.0 - Theta)))
        Theta_2_3 = (degrees(acos(cos_theta_2_3)))
        Theta_3_4 = (degrees(acos(cos_theta_3_4)) + Theta)
        Theta_1_2, Theta_2_3, Theta_3_4 \
            = self.__adjust_thetas(Theta_1_2, Theta_2_3, Theta_3_4)

        return [Theta_1_2, Theta_2_3, Theta_3_4]

    def _calc_XZ_planar_theta(self, w, z, x):
        logger.log(20, '[INFO]  w, z, x {} {} {}'.format(w, z, x))
        cos_theta_0_5 = self.cosine_theorem(w, z, x)
        Theta_0_5 = degrees(acos(cos_theta_0_5))

        Theta_1_2 = 90
        if Theta_0_5 < 90.0:
            Theta_1_2 = 0.0 + Theta_0_5
        else:
            Theta_1_2 = Theta_0_5 - 0.0

        return [Theta_1_2, Theta_0_5]

    def _calc_grip_theta(self, arm_status):
        # arm_states = ['targeting', 'grip-holding', 'lifting', 'reaching', 'grip-releasing']
        if arm_status in ['grip-holding', 'lifting', 'reaching', ]:
            return 180
        else:
            return 135

    @staticmethod
    def cosine_theorem(w, h, l):
        logger.log(20, '[INFO] w, h, l {} {} {}'.format(w, h, l))
        if 0.0 in [w, h]:
            return 0.0
        else:
            return ((w ** 2) + (h ** 2) - (l ** 2)) / (2 * w * h)

    @staticmethod
    def __adjust_thetas(Theta_1_2, Theta_2_3, Theta_3_4):
        Theta_1_2 =Theta_1_2-90
        Theta_2_3 = Theta_2_3- 90
        Theta_3_4 =  Theta_3_4
        return Theta_1_2, Theta_2_3, Theta_3_4


if __name__ == '__main__':
    joint_degree = Joint_Degree_Calculator()
    while True:
        _data = input(' input target coordinate : x y z')
        data = _data.split(' ')
        print(data)
        lengths = joint_degree.Get_Lengths(data)
        joint_degrees = joint_degree.Get_Thetas(lengths)
