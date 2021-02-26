#!/usr/bin/python3
import os, sys
from math import degrees, acos, floor
import logging
from log_manager import Log_Manager

Log = Log_Manager()


class Joint_Degree_Calculator():
    ch = 'jdc'
    # distance mm from previous link
    #  current settings : x y z = width(x) depth(y) height(z)
    rel_pos = {
        'base_link': {'width': 0.0, 'height': 0.0, 'depth': 0.0},
        '_5': {'width': 0.0, 'height': 48.2, 'depth': 0.0},
        '_4': {'width': 0.0, 'height': 31.0, 'depth': 0.0},
        '_3': {'width': 0.0, 'height': 61.5, 'depth': 0.0},
        '_2': {'width': 0.0, 'height': 61.5, 'depth': 0.0},
        '_1': {'width': 0.0, 'height': 10.0, 'depth': 0.0},
        '_0': {'width': 24.0, 'height': 57.5, 'depth': 0.0},
        '_base_link_4': {'width': 0.0, 'height': 79.2, 'depth': 0.0},  # base_link to _4
        '_1_0': {'width': 34.0, 'height': 67.5, 'depth': 0.0},  # _2 to _0
    }  # 169.6 - 79.2 = 90.4

    # _0 = 110
    # _30 = 190
    # _45 = 230
    # _60 = 270
    # _90 = 350
    # _120 = 435
    # _135 = 475
    # _150 = 515
    # _180 = 600
    # _1_deg = (_180 - _0) // 180
    grip_deg = {'open': 90, 'close': 135}

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))

    def Get_Lengths(self, xyz=None):
        def _correct_x_in_xyz(rel_pos, xyz):
            return [float(xyz[0] - rel_pos['_1_0']['width']), xyz[1], xyz[2]]

        def _calc_XZ_surface_distance(self, xyz):  # planar : flat surface
            xz_len = ((xyz[0] ** 2) + (xyz[2] ** 2)) ** (1 / 2)
            return self.Cut_Small_Number(xz_len)

        Log.intervally(self.ch, 20, '[Joint] data {}'.format(xyz))
        rel_pos = self.rel_pos
        next_xyz = _correct_x_in_xyz(rel_pos, xyz)

        # calc_view_from_ciel_distance
        xz_len = _calc_XZ_surface_distance(self, next_xyz)

        x, y, z = next_xyz
        _h = (rel_pos['_1_0']['height'] - rel_pos['_base_link_4']['height']) + y  # target y = Ros Z = height
        h = self.Cut_Small_Number(_h)
        _2_4_len = self.Cut_Small_Number(((xz_len ** 2) + (h ** 2)) ** (1 / 2))
        Pos_Params = {'x': x, 'y': y, 'z': z,
                      'xz_len': xz_len, 'h': h, '_2_4_len': _2_4_len,
                      '_2_len': rel_pos['_2']['height'], '_3_len': rel_pos['_3']['height']}
        Log.intervally(self.ch, 20, '[Joint] lengths {}'.format(Pos_Params))

        return Pos_Params

    def Get_Thetas(self, Pos_Params, arm_status='targeting'):
        def __adjust_thetas(Deg_1_2, Deg_2_3):
            adjusting_deg_value = 90
            Deg_1_2 = Deg_1_2 - adjusting_deg_value
            Deg_2_3 = Deg_2_3 - adjusting_deg_value
            return Deg_1_2, Deg_2_3

        def _calc_XY_surface_theta(self, xz_len, _2_4_len, _2_len, _3_len):
            cos_theta_1_2 = self.Cut_Small_Number(self.Cosine_Theorem(_2_len, _2_4_len, _3_len))
            cos_theta_2_3 = self.Cut_Small_Number(self.Cosine_Theorem(_2_len, _3_len, _2_4_len))
            cos_theta_3_4 = self.Cut_Small_Number(self.Cosine_Theorem(_2_4_len, _3_len, _2_len))
            Log.intervally(self.ch, 20,
                           '[Joint] _1_2, _2_3, _3_4 {} {} {}'.format(cos_theta_1_2, cos_theta_2_3, cos_theta_3_4))
            Deg = degrees(acos(xz_len / _2_4_len))
            Deg_1_2 = ((degrees(acos(cos_theta_1_2)) + (90.0 - Deg)))
            Deg_2_3 = (degrees(acos(cos_theta_2_3)))
            Deg_3_4 = (degrees(acos(cos_theta_3_4)) + Deg)
            Deg_1_2, Deg_2_3 = __adjust_thetas(Deg_1_2, Deg_2_3)
            return self.Floar_Int([Deg_1_2, Deg_2_3, Deg_3_4])

        def _calc_XZ_surface_theta(self, xz_len, z, x):
            Log.intervally(self.ch, 20, '[Joint]  xz_len, z, x {} {} {}'.format(xz_len, z, x))
            cos_theta_0_5 = self.Cosine_Theorem(xz_len, z, x)
            Deg_0_5 = degrees(acos(cos_theta_0_5))
            Deg_1_2 = 0.0 + Deg_0_5 if Deg_0_5 < 90.0 else Deg_0_5 - 0.0
            return self.Floar_Int([Deg_1_2, Deg_0_5])

        def _calc_grip_theta(self, arm_status):
            # arm_states = ['targeting', 'grip-holding', 'lifting', 'reaching', 'grip-releasing']
            if arm_status in ['grip-holding', 'lifting_up', 'reaching',]:
                return self.grip_deg['close']
            else:
                return self.grip_deg['open']

        # x, y, z, xz_len, h, _2_4_len, _2_len, _3_len = self.__expand_dict(Pos_Params)
        x, y, z, xz_len, h, _2_4_len, _2_len, _3_len = [Pos_Params[key] for key in Pos_Params]
        XY_Degs = _calc_XY_surface_theta(self, xz_len, _2_4_len, _2_len, _3_len)
        _XZ_Degs = _calc_XZ_surface_theta(self, xz_len, z, x)
        grip_Deg = _calc_grip_theta(self, arm_status)

        joint_degrees = {'ch00': grip_Deg, 'ch01': _XZ_Degs[0], 'ch02': XY_Degs[0],
                         'ch03': XY_Degs[1], 'ch04': XY_Degs[2], 'ch05': _XZ_Degs[1], }

        Log.intervally(self.ch, 30, '[Joint] joint_degrees {}'.format(joint_degrees.values()))
        return joint_degrees

    @staticmethod
    def Cosine_Theorem(w, h, d):
        if 0.0 in [w, h]:
            return 0.0
        else:
            return ((w ** 2) + (h ** 2) - (d ** 2)) / (2 * w * h)

    @staticmethod
    def Cut_Small_Number(num):
        return int(num * 100) / 100

    @staticmethod
    def Floar_Int(list):
        for i, item in enumerate(list):
            list[i] = int(floor(item))
        return list


if __name__ == '__main__':
    joint_degree = Joint_Degree_Calculator()
    while True:
        _data = input(' input target coordinate : x y z')
        data = _data.split(' ')
        print(data)
        for i, d in enumerate(data):
            data[i] = float(d)
        if len(data) != 3:
            sys.exit()
        Pos_Params = joint_degree.Get_Lengths(data)
        joint_degrees = joint_degree.Get_Thetas(Pos_Params)
