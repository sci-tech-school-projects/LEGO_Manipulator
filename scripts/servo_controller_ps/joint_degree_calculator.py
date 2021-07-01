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
        '5': {'width': 0.0, 'height': 48.2, 'depth': 0.0},
        '4': {'width': 0.0, 'height': 31.0, 'depth': 15.4},
        '3': {'width': 0.0, 'height': 61.5 + 50, 'depth': 0.0},
        '2': {'width': 0.0, 'height': 61.5 + 50, 'depth': 0.0},
        '1': {'width': -36.0, 'height': 2.0, 'depth': -25.6},
        '0': {'width': 15.0, 'height': 65.8, 'depth': 0.0},
        'base_link_4': {'width': 0.0, 'height': 48.2 + 31.0, 'depth': 15.4},  # base_link to _4
        '1_0': {'width': -36.0 + 15.0, 'height': 15.0 + 65.8, 'depth': -25.6 + 0.0},  # _1 to _0
    }

    grip_deg = {'ch00': {'open': 98, 'close': 113},
                'ch06': {'open': 85, 'close': 70}}

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))

    def Get_Lengths(self, xyz=None):
        def _correct_x_in_xyz():
            _x = xyz[0] + self.rel_pos['1_0']['width']
            x = _x if _x >= 0 else xyz[0]
            return [float(x), xyz[1], xyz[2]]

        def _calc_XZ_surface_distance():  # planar : flat surface
            xz_len = (x ** 2 + z ** 2) ** (1 / 2)
            return round(xz_len, 2)

        Log.intervally(self.ch, 20, '[Joint] data {}'.format(xyz))
        [x, y, z] = _correct_x_in_xyz()
        _h = (self.rel_pos['1_0']['height'] - self.rel_pos['base_link_4']['height']) + y  # target y = Ros Z = height
        h = round(_h, 2)

        xz_len = _calc_XZ_surface_distance()  # calc_view_from_ciel_distance
        _2_4_len = x
        # L = round((x ** 2 + z ** 2) ** (1 / 2), 1)
        y_xz_len = round((xz_len ** 2 + y ** 2) ** (1 / 2), 2)

        Pos_Params = {'x': x, 'y': y, 'z': z, 'h': h,
                      'xz_len': xz_len,
                      '2_4_len': _2_4_len,
                      '2_len': self.rel_pos['2']['height'],
                      '3_len': self.rel_pos['3']['height'],
                      'y_xz_len': y_xz_len}
        return Pos_Params

    def Get_Thetas(self, Pos_Params, arm_status='targeting'):
        def _calc_XY_surface_theta():
            cos_theta_2 = round(self.Cosine_Theorem(_2_len, xz_len, _3_len), 2)
            cos_theta_3 = round(self.Cosine_Theorem(_2_len, _3_len, xz_len), 2)
            cos_theta_4 = round(self.Cosine_Theorem(xz_len, _3_len, _2_len), 2)

            adjust_deg_for_y = degrees(acos(floor(self.Cosine_Theorem(y_xz_len, xz_len, y))))
            # adjust_deg_for_y = 0.0
            ch03 = degrees(acos(cos_theta_3)) + 30.0 + adjust_deg_for_y
            ch04 = degrees(acos(cos_theta_4)) + adjust_deg_for_y
            # ch02 = 540.0 - (ch03 + (ch04 + 90.0) + 180.0 + 90.0) + adjust_deg_for_y
            ch02 = degrees(acos(cos_theta_2)) - 30.0 + adjust_deg_for_y
            [ch02, ch03, ch04] = self.Floar_Int([ch02, ch03, ch04])

            return [ch02, ch03, ch04]

        def _calc_XZ_surface_theta():
            Log.intervally(self.ch, 20, '[Joint]  xz_len, z, x {} {} {}'.format(xz_len, z, x))
            cos_theta_5 = self.Cosine_Theorem(xz_len, z, x)
            ch05 = degrees(acos(cos_theta_5))
            ch01 = ch05
            return self.Floar_Int([ch01, ch05])

        def _calc_grip_theta():
            if arm_status in ['grip-holding', 'lifting_up', 'reaching', ]:
                return self.grip_deg['ch00']['close'], self.grip_deg['ch06']['close']
            else:
                return self.grip_deg['ch00']['open'], self.grip_deg['ch06']['open']

        x, y, z, h, xz_len, _2_4_len, _2_len, _3_len, y_xz_len = Pos_Params.values()

        [ch02, ch03, ch04] = _calc_XY_surface_theta()
        [ch01, ch05] = _calc_XZ_surface_theta()
        grip_Deg_00, grip_Deg_06 = _calc_grip_theta()

        joint_degrees = {'ch00': grip_Deg_00, 'ch01': ch01, 'ch02': ch02,
                         'ch03': ch03, 'ch04': ch04, 'ch05': ch05, 'ch06': grip_Deg_06}

        Log.intervally(self.ch, 40, '[Joint] joint_degrees {}'.format(joint_degrees))
        return joint_degrees

    def _Adjust_Deg_For_y(self, x, y, L):
        _adjust_deg_for_y = self.Cosine_Theorem(x, L, y)
        return degrees(acos(_adjust_deg_for_y))

    @staticmethod
    def Cosine_Theorem(w, h, d):
        if 0.0 in [w, h]:
            return 0.0
        else:
            return ((w ** 2) + (h ** 2) - (d ** 2)) / (2 * w * h)

    @staticmethod
    def Floar_Int(list):
        for i, item in enumerate(list):
            list[i] = int(floor(item))
        return list


if __name__ == '__main__':
    joint_degree = Joint_Degree_Calculator()
    while True:
        _data = input('input target coordinate : x y z')
        data = _data.split(' ')
        print(data)
        for i, d in enumerate(data):
            data[i] = float(d)
        if len(data) != 3:
            sys.exit()
        Pos_Params = joint_degree.Get_Lengths(data)
        joint_degrees = joint_degree.Get_Thetas(Pos_Params)
