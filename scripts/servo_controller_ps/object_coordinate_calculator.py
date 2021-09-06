#!/usr/bin/python3
import time
import os
from math import degrees, acos
import cv2
import numpy as np
# import logging
from argparse import ArgumentParser
from random import randrange
from common import Common as cmn
from config import *
from log_manager import LogManager

Log = LogManager()


# from typing import TypedDict


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


class ObjectCoordinateCalculator:
    got_stage_px = False
    IS_DEV = False if os.getenv('is_dev') == 'False' else True
    is_frame_init_done = False
    _params = {'th_btm': 100, 'th_top': 170, 'min_area': 1500}
    _contours = []
    DEBUG_CH = 'occ'

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
    counter = 0
    label = None

    @property
    def frame_px_1_1(self):
        return self.FRAME_PX

    @frame_px_1_1.setter
    def frame_px_1_1(self, frame_px_1_1):
        self.FRAME_PX['1/1'] = frame_px_1_1
        self._got_darknet_frame = True

    def __init__(self):
        self._get_args()

    def _get_args(self):
        if self.IS_DEV:
            print('***** Dev mode')
            self.CAP = cv2.VideoCapture(0)
            ap = ArgumentParser()
            ap.add_argument('-d', '--dev', default=False, help='if is_dev True, Developer mode.')
            ap.add_argument('-z', '--zoom_rate', default='1/2', help='image will be resized by this value.')
            args = ap.parse_args()
            self.IS_DEV = args.dev
            self.ZOOM_RATE = args.zoom_rate
        else:
            self.ZOOM_RATE = '1/1'
            # self.CAP will be set by 'occ.cap = img_dtct.cap' at publisher file

    def can_get_stage_px(self, img=None):
        print('***** get_stage_px started')
        self.got_stage_px = False
        self.counter = 0
        while not self.got_stage_px:
            ret, img = self._get_img(img)
            if ret:
                self._init_frame_param()
                img = self._resize_image(img)
                self._contours, img = self._get_contours(img)
                self._key_control(img)
                if bool(self._contours):
                    self.stage_px = self._calc_stage_px()
                    if self.stage_px['A'] != {'x': 0, 'y': 0} and self._is_area_match(img, self.stage_px):
                        img = self._draw_contours(img)
                        self.got_stage_px = True
                    else:
                        return False
                else:
                    return False
                self._key_control(img)
        Log.only(self.DEBUG_CH, 30, '***** Got stage_px')
        return True

    def _get_img(self, img):
        if img is None:
            ret, img = self.CAP.read()
        else:
            ret, img = True, img
        return ret, img

    def _init_frame_param(self):
        if not self.is_frame_init_done:
            w, h = self.FRAME_PX['1/1'].values()
            self.FRAME_PX = {'1/1': {'w': w, 'h': h},
                             '1/2': {'w': cmn.zoomed_50(w), 'h': cmn.zoomed_50(h)},
                             '1/4': {'w': cmn.zoomed_25(w), 'h': cmn.zoomed_25(h)}}
            self.is_frame_init_done = True
            Log.only(self.DEBUG_CH, 20, 'h w FRAME_PX {}'.format([h, w, self.FRAME_PX]))
        else:
            if self._got_darknet_frame:
                pass
            else:
                Log.only(self.DEBUG_CH, 30, '***** Raise Exception')
                raise Exception('set FRAME_PX by setter frame_px_1_1 before run _init_frame_param().')

    def _resize_image(self, img):
        img = cmn.get_trimmed_img(img)
        return cv2.resize(img, (NETWORK['W'], NETWORK['H']))

    def _get_contours(self, img):
        """
        :return: contours[[],[],]
        """
        h, w, c = np.shape(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, self._params['th_btm'], self._params['th_top'], cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.__print_areas(contours)
        img = cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
        upper, lower = int(h * w * 0.8), int(h * w * 0.40)
        contours = list(filter(lambda x: upper > cv2.contourArea(x) > lower, contours))
        Log.only(self.DEBUG_CH, 30, 'contours len {}'.format(len(contours)))
        return contours, img

    @staticmethod
    def _is_area_match(img, _stage_px):
        h, w, _ = np.shape(img)
        frame_area = h * w
        areas = {'min': int(frame_area * 0.4), 'max': int(frame_area * 0.7)}
        w = _stage_px['C']['x'] - _stage_px['A']['x']
        h = _stage_px['C']['y'] - _stage_px['A']['y']
        area = int(w * h)
        return areas['min'] <= area <= areas['max']

    def _calc_stage_px(self):
        """:return: {key: {'x': int, 'y': int},...}"""
        max_len_contour = self.__get_max_len_contour()
        cx, cy = self.__get_center_of_contour(max_len_contour)
        rect_prams = self.__get_rect_prams(max_len_contour, cx, cy)
        _stage_list_px = self.__calc_stage_px(cx, cy, rect_prams)
        _stage_px = {key: {'x': x, 'y': y} for key, [x, y] in zip(['A', 'B', 'C', 'D', 'E'], _stage_list_px)}
        if self.counter % 5 == 0:
            Log.only(self.DEBUG_CH, 20, '***** stage_px : {}'.format(_stage_px))
        self.counter += 1
        return _stage_px

    def _draw_contours(self, img):
        img = self.__draw_stage(img)
        txts = {'Bot : {}': self._params['th_btm'], 'Top : {}': self._params['th_top'], }
        img = self.__put_text_by_dict(img, txts, 0)
        return img

    def _key_control(self, img):
        cv2.imshow("occ", img)
        if self.IS_DEV:
            if cv2.waitKey(0) % 0xff == ord('q'):
                self.got_stage_px = True
            elif cv2.waitKey(0) % 0xff == ord('a'):
                pass
            elif cv2.waitKey(0) % 0xff == ord('w'):
                self._params['th_btm'] += 10
            elif cv2.waitKey(0) % 0xff == ord('s'):
                self._params['th_btm'] -= 10
            elif cv2.waitKey(0) % 0xff == ord('e'):
                self._params['th_top'] += 10
            elif cv2.waitKey(0) % 0xff == ord('d'):
                self._params['th_top'] -= 10
            elif cv2.waitKey(0) % 0xff == ord('r'):
                self._params['min_area'] += 100
            elif cv2.waitKey(0) % 0xff == ord('f'):
                self._params['min_area'] -= 100
        else:
            pass
            if cv2.waitKey(1) % 0xff == ord('a'):
                pass
            else:
                pass
        time.sleep(0.2)

    @staticmethod
    def __print_areas(contours):
        for cnt in contours:
            print(cv2.contourArea(cnt))

    def __get_max_len_contour(self):
        Log.only(self.DEBUG_CH, 10, '***** self.contours : {}'.format(self._contours))
        lens = [len(contour) for contour in self._contours]
        Log.only(self.DEBUG_CH, 10, '***** lens : {}'.format(lens))

        max_len = max(lens)
        max_len_idx = None
        for idx, contour in enumerate(self._contours):
            if max_len == len(contour):
                max_len_idx = idx
        Log.only(self.DEBUG_CH, 10, '***** max_idx : {}'.format(max_len_idx))
        max_len_contour = self._contours[max_len_idx]
        return max_len_contour

    def __get_center_of_contour(self, contour):
        m = cv2.moments(contour)
        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])
        Log.only(self.DEBUG_CH, 10, '***** m : {}'.format(m))
        return cx, cy

    def __get_rect_prams(self, contour, cx: int, cy: int):
        """:return: {key: {'square': [float, ], 'x': [int, ], 'y': [int, ], } }"""
        rect_prams = {key: {'square': [], 'x': [], 'y': [], } for key in ['A', 'B', 'C', 'D']}
        keys = ['square', 'x', 'y']

        for x, y in zip([cont[0][0] for cont in contour], [cont[0][1] for cont in contour]):
            square = cmn.sqr_theorem(x - cx, y - cy)
            bool_corner_sets = {'A': x < cx and y < cy, 'B': x < cx and y > cy,
                                'C': x > cx and y > cy, 'D': x > cx and y < cy}
            for corner, boolean in bool_corner_sets.items():
                if boolean:
                    for _key, _val in {'square': square, 'x': x, 'y': y}.items():
                        rect_prams[corner][_key].append(_val)

        Log.only(self.DEBUG_CH, 10, '***** lens A square x y : {}'.format([len(rect_prams['A'][key]) for key in keys]))
        return rect_prams

    def __calc_stage_px(self, cx: int, cy: int, rect_prams):
        _stage_list_px = []
        for corner in ['A', 'B', 'C', 'D']:
            max_val = max(rect_prams[corner]['square'])
            max_index = rect_prams[corner]['square'].index(max_val)
            _stage_list_px.append([rect_prams[corner]['x'][max_index], rect_prams[corner]['y'][max_index]])
            Log.only(self.DEBUG_CH, 10, '***** max_index : {}  '.format(max_index))
        _stage_list_px.append([cx, cy])

        Log.only(self.DEBUG_CH, 10, '***** stage_px : {}'.format(_stage_list_px))
        return _stage_list_px

    def __draw_stage(self, img):
        for val in self.stage_px.values():
            x, y = val.values()
            cv2.circle(img, (int(x), int(y)), 3, (0, 0, 255))
        return img

    @staticmethod
    def __put_text_by_dict(img, txts, start_number):
        for idx, key in enumerate(txts):
            text = key.format(txts[key])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, text, (20, 25 * (idx + 1 + start_number)), font, 0.5, (0, 0, 255), 2)
        return img

    def get_obj_arm_len_deg(self):
        """:return: {'len': int, 'deg': int}"""
        # self._is_set_obj_px()
        run = True
        if 0.0 in self._px_per_mm.values():
            self._px_per_mm = self._get_px_per_mm()
        if 0 in self.ARM.values():
            self.ARM['TO_FRAME_MM'], self.ARM['BTM_TO_BTM_MM'] = self._get_arm_offset_from_frame_mm()
            Log.only(self.DEBUG_CH, 20, '***** ARM {}'.format([self.ARM]))

        while run:
            self.obj_px, self.label = self._update_obj_px()
            self._arm_obj_mm = self._get_obj_arm_xy()
            self.arm_obj_len_deg = self._xy_into_len_deg()

            if self.IS_DEV:
                self._development_function()
            else:
                run = False
        return self.arm_obj_len_deg, self.obj_px, self.label

    def get_xyz_mm(self, detections):
        """
        detections -> self.__parse_detections()
        :return: [float, 0.0, float]
        """
        if len(detections) >= 1 and len(detections[0]) == 3:
            self._detections = detections
            self.arm_obj_len_deg, self.obj_px, self.label = self.get_obj_arm_len_deg()
            # Be aware the difference of meaning x & y between publisher file & this file.
            xyz_mm = [self._arm_obj_mm['y_f'], 0.0, self._arm_obj_mm['x_f']]
            return xyz_mm, self.label
        else:
            text = [len(detections), len(detections[0])]
            Log.only(self.DEBUG_CH, 20, '***** len(detections) len(detections[0]) {}'.format(text))
            return [None, None, None], None

    def _get_px_per_mm(self, ) -> {float, float}:
        stage_w_in_frame = self.stage_px['D']['x'] - self.stage_px['A']['x']
        stage_h_in_frame = self.stage_px['C']['y'] - self.stage_px['A']['y']
        _x = cmn.round_3(stage_w_in_frame / self.STAGE_MM['w'])
        _y = cmn.round_3(stage_h_in_frame / self.STAGE_MM['h'])
        px_per_mm = {'x': _x, 'y': _y}
        Log.only(self.DEBUG_CH, 20, '***** px_per_mm {}'.format(px_per_mm))
        return px_per_mm

    def _get_arm_offset_from_frame_mm(self) -> [int, int]:
        _BTM_TO_BTM_PX_1 = self.FRAME_PX[self.ZOOM_RATE]['h'] - self.stage_px['B']['y']
        _BTM_TO_BTM_PX_2 = self.FRAME_PX[self.ZOOM_RATE]['h'] - self.stage_px['C']['y']
        BTM_TO_BTM_PX = int((_BTM_TO_BTM_PX_1 + _BTM_TO_BTM_PX_2) / 2)
        BTM_TO_BTM_MM = self.__calc_mm_from_px(BTM_TO_BTM_PX, 'y', int)
        TO_FRAME_MM = self.ARM['TO_STAGE_MM'] - BTM_TO_BTM_MM
        return [TO_FRAME_MM, BTM_TO_BTM_MM]

    def _update_obj_px(self):
        """:return: {'start': {'x': int, 'y': int}, 'end': {'x': int, 'y': int}, 'center': {'x': int, 'y': int}}"""
        if self.IS_DEV:
            return self.__dummy_params()
        else:
            return self.__parse_detections()

    def __parse_detections(self):
        """
        :return: obj_px = {'start': {'x': int, 'y': int}, 'end': {'x': int, 'y': int}, 'center': {'x': int, 'y': int}}
        """
        for _label, confidence, bbox in self._detections:
            x, y, w, h = list(map(int, bbox))
            end_x, end_y = x + w, y + h
            _obj_px = {'start': {'x': x, 'y': y}, 'end': {'x': end_x, 'y': end_y},
                       'center': {'x': cmn.calc_mid_px(x, end_x), 'y': cmn.calc_mid_px(y, end_y)}, }
            Log.only(self.DEBUG_CH, 30, '[occ] _obj_px {}'.format([_label, confidence, _obj_px]))
            return _obj_px, _label

    def _get_obj_arm_xy(self, ) -> {float, float}:
        """:return: {'x_f': float, 'y_f': float}"""
        _frame_right_2_obj_x_mm = self.__calc_mm_from_px(self.obj_px['center']['x'], 'x', float)
        _arm_x_pos_mm = self.__calc_mm_from_px(self.stage_px['E']['x'], 'x', float)

        _stage_btm_2_obj_y_px = abs(self.stage_px['B']['y'] - self.obj_px['center']['y'])
        _arm_2_obj_y_mm = self.ARM['TO_STAGE_MM'] + self.__calc_mm_from_px(_stage_btm_2_obj_y_px, 'y', float)

        _arm_obj_mm = {'x_f': cmn.round_3(_frame_right_2_obj_x_mm - _arm_x_pos_mm),
                       'y_f': cmn.round_3(_arm_2_obj_y_mm)}
        Log.only(self.DEBUG_CH, 20, '***** _arm_obj_mm {}'.format(_arm_obj_mm))
        return _arm_obj_mm

    def _xy_into_len_deg(self, ) -> {int, int}:
        """:return: {'len': int, 'deg': int}"""
        _l: float = cmn.sqr_theorem(self._arm_obj_mm['x_f'], self._arm_obj_mm['y_f'])
        _cos: float = cmn.cos_theorem(self._arm_obj_mm['y_f'], _l, self._arm_obj_mm['x_f'])
        arm_obj_deg = self.__calc_arm_obj_deg(_cos)
        arm_obj_len_deg = {'len': int(_l),
                           'deg': arm_obj_deg}
        Log.only(self.DEBUG_CH, 30, 'x y len cos deg {}'.format([self._arm_obj_mm.values(), _l, _cos, arm_obj_deg]))
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

    def __calc_mm_from_px(self, px, xy: str, int_float):
        return int_float(float(px) / self._px_per_mm[xy])

    def __calc_px_from_mm(self, mm, xy: str, int_float):
        return int_float(float(mm) * self._px_per_mm[xy])

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

    @staticmethod
    def __dummy_params():
        s_x, s_y = randrange(60, 240, 10), randrange(130, 170, 10)
        e_x, e_y = s_x + randrange(10, 20, 1), s_y + randrange(10, 20, 1)
        c_x, c_y = int((s_x + e_x) / 2), int((s_y + e_y) / 2)
        _obj_px = {'start': {'x': s_x, 'y': s_y}, 'end': {'x': e_x, 'y': e_y}, 'center': {'x': c_x, 'y': c_y}}
        _label = 'connector'
        return _obj_px, _label

    def __del__(self):
        print('***** Finalize {}'.format(__class__.__name__))
        cv2.destroyAllWindows()


if __name__ == '__main__':
    occ = ObjectCoordinateCalculator()
    stage_px = occ.can_get_stage_px()
    obj_arm_len_deg, obj_px, label = occ.get_obj_arm_len_deg()
