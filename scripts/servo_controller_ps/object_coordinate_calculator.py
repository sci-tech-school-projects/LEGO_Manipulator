#!/usr/bin/python3
import time
from math import degrees, acos
import cv2
import numpy as np
import logging
from argparse import ArgumentParser
from random import randrange
from common import *
import warnings

# from typing import TypedDict

logger = logging.getLogger('LoggingTest')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)

warnings.resetwarnings()
warnings.simplefilter('error')


# class Type_Spec_In_Dict(TypedDict):
#     x: int
#     y: int
#     w: int
#     h: int
#     start: dict
#     end: dict
#     center: dict
#     A: dict
#     B: dict
#     C: dict
#     D: dict
#     E: dict
#     len: int
#     deg: int
#     x_f: float
#     y_f: float


class Object_Coordinate_Calculator():
    got_stage_px = False
    IS_DEV = False
    is_called = False
    _tunes = {'th_btm': 100, 'th_top': 170, 'min_area': 1500}
    _contours = []

    CAP = None
    ZOOM_RATE = '1/2'
    ARM = {'TO_STAGE_MM': 110, 'TO_FRAME_MM': 0, 'BTM_TO_BTM_MM': 0}
    FRAME_PX = {'1/1': {'w': 0, 'h': 0}, '1/2': {'w': 0, 'h': 0}, '1/4': {'w': 0, 'h': 0}, }
    STAGE_MM = {'w': 150, 'h': 110}
    _px_per_mm = {'x': 0.0, 'y': 0.0}
    obj_px = {'start': {'x': 0, 'y': 0}, 'end': {'x': 0, 'y': 0}, 'center': {'x': 0, 'y': 0}, }
    stage_px = {key: {'x': 0, 'y': 0} for key in ['A', 'B', 'C', 'D', 'E']}
    _arm_obj_mm = {'x_f': 0.0, 'y_f': 0.0}
    arm_obj_len_deg = {'len': 0, 'deg': 0}
    _got_darknet_frame = False
    _detections = []

    @property
    def frame_px_1_1(self):
        return self.FRAME_PX

    @frame_px_1_1.setter
    def frame_px_1_1(self, FRAME_PX_1_1):
        self.FRAME_PX['1/1'] = FRAME_PX_1_1
        self._got_darknet_frame = True

    @property
    def got_darknet_frame(self):
        return self._got_darknet_frame

    @property
    def cap(self):
        return self.CAP

    @cap.setter
    def cap(self, cap):
        self.CAP = cap

    def __init__(self):
        ap = ArgumentParser()
        ap.add_argument('-d', '--dev', default=False, help='if is_dev True, Developer mode.')
        ap.add_argument('-z', '--zoom_rate', default='1/2', help='image will be resized by this value.')
        args = ap.parse_args()
        self.IS_DEV = args.dev
        self.ZOOM_RATE = args.zoom_rate
        if self.IS_DEV:
            print('***** Dev mode')
            self.CAP = cv2.VideoCapture(0)
        else:
            pass

    def get_stage_px(self):
        """:return: {key: {'x': int, 'y': int}, ...}"""
        print('***** get_stage_px started')
        self.got_stage_px = False
        while not self.got_stage_px:
            ret, img = self.CAP.read()
            self._init_frame_param(img)
            img = self._resize_image(img)
            if ret:
                self._contours = self._get_contours(img)
                if self._contours != []:
                    self.stage_px = self._calc_stage_px()

                    img = self._draw_contours(img)
                    self.got_stage_px = True
                else:
                    logger.log(20, 'contours not found')
                self._key_control(img)
        # self._destroy_cap_not_dev()
        return self.stage_px

    def _init_frame_param(self, img):
        if not self.is_called:
            h, w, _ = (*self.FRAME_PX['1/1'].values(), None) if self._got_darknet_frame else np.shape(img)
            self.FRAME_PX = {'1/1': {'w': w, 'h': h},
                             '1/2': {'w': zoomed_50(w), 'h': zoomed_50(h)},
                             '1/4': {'w': zoomed_25(w), 'h': zoomed_25(h)}}
            self.is_called = True
            logger.log(20, '***** ARM {}'.format([self.ARM]))
            logger.log(20, 'h w FRAME_PX {}'.format([h, w, self.FRAME_PX]))
        else:
            pass

    def _resize_image(self, img):
        return cv2.resize(img, (self.FRAME_PX[self.ZOOM_RATE]['w'], self.FRAME_PX[self.ZOOM_RATE]['h']))

    def _get_contours(self, img):
        """:return: contours[[],[],...]"""
        h, w, c = np.shape(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, self._tunes['th_btm'], self._tunes['th_top'], cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = list(filter(lambda x: int(h * w * 0.75) > cv2.contourArea(x) > self._tunes['min_area'], contours))
        return contours

    def _calc_stage_px(self):
        """:return: {key: {'x': int, 'y': int},...}"""
        max_len_contour = self.__get_max_len_contour()
        cx, cy = self.__get_center_of_contour(max_len_contour)
        rect_prams = self.__get_rect_prams(max_len_contour, cx, cy)
        _stage_list_px = self.__calc_stage_px(cx, cy, rect_prams)
        stage_px = {key: {'x': x, 'y': y} for key, [x, y] in zip(['A', 'B', 'C', 'D', 'E'], _stage_list_px)}
        logger.log(20, '***** stage_px : {}'.format(stage_px))
        return stage_px

    def _draw_contours(self, img):
        img = self.__draw_stage(img)
        dict = {'Bot : {}': self._tunes['th_btm'], 'Top : {}': self._tunes['th_top'], }
        img = self.__put_text_by_dict(img, dict, 0)
        return img

    def _key_control(self, img):
        if self.IS_DEV:
            cv2.imshow("img", img)
            if cv2.waitKey(0) % 0xff == ord('q'):
                self.got_stage_px = True
            elif cv2.waitKey(0) % 0xff == ord('a'):
                logger.log(20, 'pass')
                pass
            elif cv2.waitKey(0) % 0xff == ord('w'):
                self._tunes['th_btm'] += 10
            elif cv2.waitKey(0) % 0xff == ord('s'):
                self._tunes['th_btm'] -= 10
            elif cv2.waitKey(0) % 0xff == ord('e'):
                self._tunes['th_top'] += 10
            elif cv2.waitKey(0) % 0xff == ord('d'):
                self._tunes['th_top'] -= 10
            elif cv2.waitKey(0) % 0xff == ord('r'):
                self._tunes['min_area'] += 100
            elif cv2.waitKey(0) % 0xff == ord('f'):
                self._tunes['min_area'] -= 100
            time.sleep(0.2)

    def __get_max_len_contour(self):
        logger.log(10, '***** self.contours : {}'.format(self._contours))
        lens = [len(contour) for contour in self._contours]
        logger.log(10, '***** lens : {}'.format(lens))

        max_len = max(lens)
        max_len_idx = None
        for idx, contour in enumerate(self._contours):
            if max_len == len(contour): max_len_idx = idx
        logger.log(10, '***** max_idx : {}'.format(max_len_idx))
        max_len_contour = self._contours[max_len_idx]
        return max_len_contour

    def __get_center_of_contour(self, contour):
        M = cv2.moments(contour)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        logger.log(10, '***** M : {}'.format(M))
        return cx, cy

    def __get_rect_prams(self, contour, cx: int, cy: int):
        """:return: {key: {'square': [float, ], 'x': [int, ], 'y': [int, ], } }"""
        rect_prams = {key: {'square': [], 'x': [], 'y': [], } for key in ['A', 'B', 'C', 'D']}
        keys = ['square', 'x', 'y']

        for x, y in zip([cont[0][0] for cont in contour], [cont[0][1] for cont in contour]):
            square = sqr_theorem(x - cx, y - cy)
            bool_corner_sets = {'A': x < cx and y < cy, 'B': x < cx and y > cy,
                                'C': x > cx and y > cy, 'D': x > cx and y < cy}
            for corner, bool in bool_corner_sets.items():
                if bool:
                    for _key, _val in {'square': square, 'x': x, 'y': y}.items(): rect_prams[corner][_key].append(_val)

        logger.log(10, '***** lens A square x y : {}'.format([len(rect_prams['A'][key]) for key in keys]))
        return rect_prams

    def __calc_stage_px(self, cx: int, cy: int, rect_prams):
        _stage_list_px = []
        for corner in ['A', 'B', 'C', 'D']:
            max_val = max(rect_prams[corner]['square'])
            max_index = rect_prams[corner]['square'].index(max_val)
            _stage_list_px.append([rect_prams[corner]['x'][max_index], rect_prams[corner]['y'][max_index]])
            logger.log(10, '***** max_index : {}  '.format(max_index))
        _stage_list_px.append([cx, cy])

        logger.log(10, '***** stage_px : {}'.format(_stage_list_px))
        return _stage_list_px

    def __draw_stage(self, img):
        for val in self.stage_px.values():
            x, y = val.values()
            cv2.circle(img, (int(x), int(y)), 3, (0, 0, 255))
        return img

    def __put_text_by_dict(self, img, dict, start_number):
        for idx, key in enumerate(dict):
            text = key.format(dict[key])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, text, (20, 25 * (idx + 1 + start_number)), font, 0.5, (0, 0, 255), 2)
        return img

    def get_obj_arm_len_deg(self):
        """:return: {'len': int, 'deg': int}"""
        # self._is_set_obj_px()
        self.got_stage_px = True
        if 0.0 in [val for val in self._px_per_mm.values()]: self._px_per_mm = self._get_px_per_mm()
        self.ARM['TO_FRAME_MM'], self.ARM['BTM_TO_BTM_MM'] = self._get_arm_offset_from_frame_mm()

        while self.got_stage_px:
            self.obj_px = self._update_obj_px()
            self._arm_obj_mm = self._get_obj_arm_xy()
            self.arm_obj_len_deg = self._xy_into_len_deg()

            if self.IS_DEV:
                self._development_function()
            else:
                self.got_stage_px = False
        return self.arm_obj_len_deg

    def get_xyz_mm(self, detections):
        """:return: [float, 0.0, float] """
        self._detections = detections
        self.arm_obj_len_deg = self.get_obj_arm_len_deg()
        # Be aware the difference of meaning x & y between publisher file & this file.
        xyz_mm = [self._arm_obj_mm['y_f'],
                  0.0,
                  self._arm_obj_mm['x_f']]
        return xyz_mm

    # def _is_set_obj_px(self):
    #     if not self.IS_DEV :
    #         raise Exception('obj_px is not set. run obj_px setter before this function.')
    #     else:
    #         print('***** run get_obj_arm_len_deg')

    def _obj_px_values(self):
        """:return: [int,int,int,int,int,int]"""
        return np.array([list(val.values()) for val in self.obj_px.values()]).flatten()

    def _get_px_per_mm(self, ) -> {float, float}:
        stage_w_in_frame = self.stage_px['D']['x'] - self.stage_px['A']['x']
        stage_h_in_frame = self.stage_px['C']['y'] - self.stage_px['A']['y']
        _x = round_3(stage_w_in_frame / self.STAGE_MM['w'])
        _y = round_3(stage_h_in_frame / self.STAGE_MM['h'])
        px_per_mm = {'x': _x, 'y': _y}
        logger.log(20, '***** px_per_mm {}'.format(px_per_mm))
        return px_per_mm

    def _get_arm_offset_from_frame_mm(self) -> [int, int]:
        _BTM_TO_BTM_PX_1 = self.FRAME_PX[self.ZOOM_RATE]['h'] - self.stage_px['B']['y']
        _BTM_TO_BTM_PX_2 = self.FRAME_PX[self.ZOOM_RATE]['h'] - self.stage_px['C']['y']
        BTM_TO_BTM_PX = int((_BTM_TO_BTM_PX_1 + _BTM_TO_BTM_PX_2) / 2)
        BTM_TO_BTM_MM = self.__calc_mm_from_px(BTM_TO_BTM_PX, 'y', 'int')
        TO_FRAME_MM = self.ARM['TO_STAGE_MM'] - BTM_TO_BTM_MM

        # 57 = 110 - 53
        logger.log(20, '***** TO_STAGE_MM  TO_FRAME_MM  BTM_TO_BTM_MM\n {}'.format(
            [self.ARM['TO_STAGE_MM'], TO_FRAME_MM, BTM_TO_BTM_MM]))
        logger.log(20, ' FRAME_PX {}'.format(self.FRAME_PX))
        logger.log(20, ' stage_px {}'.format(self.stage_px))

        return [TO_FRAME_MM, BTM_TO_BTM_MM]

    def _get_obj_arm_xy(self, ) -> {float, float}:
        """:return: {'x_f': float, 'y_f': float}"""
        _frame_right_2_obj_x_mm = self.__calc_mm_from_px(self.obj_px['center']['x'], 'x', 'float')
        _arm_x_pos_mm = self.__calc_mm_from_px(self.stage_px['E']['x'], 'x', 'float')

        _stage_btm_2_obj_y_px = abs(self.stage_px['B']['y'] - self.obj_px['center']['y'])
        _arm_2_obj_y_mm = self.ARM['TO_STAGE_MM'] + self.__calc_mm_from_px(_stage_btm_2_obj_y_px, 'y', 'float')

        _arm_obj_mm = {'x_f': round_3(_frame_right_2_obj_x_mm - _arm_x_pos_mm),
                       'y_f': round_3(_arm_2_obj_y_mm)}
        logger.log(20, '***** _obj_px {}'.format(self.obj_px))
        logger.log(20, ' _arm_obj_mm {}'.format(_arm_obj_mm))
        return _arm_obj_mm

    def _xy_into_len_deg(self, ) -> {int, int}:
        """:return: {'len': int, 'deg': int}"""
        _l: float = sqr_theorem(self._arm_obj_mm['x_f'], self._arm_obj_mm['y_f'])
        _cos: float = cos_theorem(self._arm_obj_mm['y_f'], _l, self._arm_obj_mm['x_f'])
        arm_obj_deg = self.__calc_arm_obj_deg(_cos)
        arm_obj_len_deg = {'len': int(_l),
                           'deg': arm_obj_deg}
        logger.log(20, 'x y len cos deg {}'.format([self._arm_obj_mm.values(), _l, _cos, arm_obj_deg]))
        return arm_obj_len_deg

    def _development_function(self, ) -> None:
        ret, img = self.CAP.read()
        if ret:
            img = self._resize_image(img)
            img = self.__draw_mark(img)
            txts = {'l : {}': self.arm_obj_len_deg['len'], 'deg : {}': self.arm_obj_len_deg['deg'], }
            img = self.__put_text_by_dict(img, txts, 0)
            # self.__draw_lines(img)
            self._key_control(img)

    def _update_obj_px(self) -> dict:
        """:return: {'start': {'x': int, 'y': int}, 'end': {'x': int, 'y': int}, 'center': {'x': int, 'y': int}}"""
        if self.IS_DEV:
            return self.__dammy_params()
        else:
            self.obj_px = self.__parse_detections()

    def __calc_mm_from_px(self, px, xy: str, int_float: str):
        return eval(int_float)(float(px) / self._px_per_mm[xy])

    def __calc_px_from_mm(self, mm, xy: str, int_float: str):
        return eval(int_float)(float(mm) * self._px_per_mm[xy])

    def __calc_arm_obj_deg(self, _cos):
        is_plus = self._arm_obj_mm['x_f'] >= 0.0
        _deg = int(degrees(acos(_cos)))
        _arm_obj_deg = 90 - _deg if is_plus else 90 + _deg
        return _arm_obj_deg

    def __draw_mark(self, img):
        stage_center = (self.obj_px['center']['x'], self.obj_px['center']['y'])
        center_edge_x = (self.stage_px['E']['x'], self.FRAME_PX[self.ZOOM_RATE]['h'] - 5)
        centers = [stage_center, center_edge_x]
        for center in centers:
            cv2.circle(img, center, 3, (255, 0, 0))
        return img

    def __draw_lines(self, img):
        """
        :param img:
        :return:
        """
        btm = self.__calc_px_from_mm(self.ARM['TO_FRAME_MM'], 'y', 'int')
        top = self.FRAME_PX[self.ZOOM_RATE]['h']
        cv2.line(img, (50, btm), (50, top), (0, 0, 255), 2)

    def __dammy_params(self):
        s_x, s_y = randrange(60, 240, 10), randrange(130, 170, 10)
        e_x, e_y = s_x + randrange(10, 20, 1), s_y + randrange(10, 20, 1)
        c_x, c_y = int((s_x + e_x) / 2), int((s_y + e_y) / 2)
        return {'start': {'x': s_x, 'y': s_y}, 'end': {'x': e_x, 'y': e_y}, 'center': {'x': c_x, 'y': c_y}}

    def __parse_detections(self):
        """
        :param :
        :return: obj_px = {'start': {'x': int, 'y': int}, 'end': {'x': int, 'y': int}, 'center': {'x': int, 'y': int}, }
        """
        for label, confidence, bbox in self._detections:
            x, y, w, h = bbox
            end_x, end_y = x + w, h + h
            return {'start': {'x': x, 'y': y}, 'end': {'x': end_x, 'y': end_y},
                    'center': {'x': calc_mid_px(x, end_x), 'y': calc_mid_px(y, end_y)}, }


if __name__ == '__main__':
    occ = Object_Coordinate_Calculator()
    occ.get_stage_px()
    obj_arm_len_deg = occ.get_obj_arm_len_deg()
