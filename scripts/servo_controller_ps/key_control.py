#!/usr/bin/python3
import os
import time
import numpy as np
import cv2
import rospy
from std_msgs.msg import String

from config import *
from common import Common as cmn
from log_manager import LogManager
from image_detector import ImageDetection
from joint_degree_calculator import JointDegreeCalculator
from object_coordinate_calculator import ObjectCoordinateCalculator
from sub import Node

if os.getenv('is_dev') == 'False':
    print('***** Load Image_Detection')
    img_dtct = ImageDetection()
    print('***** Load Object_Coordinate_Calculator')
    occ = ObjectCoordinateCalculator()

print('***** Load Joint_Degree_Calculator')
jdc = JointDegreeCalculator()
node = Node()
Log = LogManager()


class ServoController:
    IS_DEV = False if os.getenv('is_dev') == 'False' else True
    is_run_cv2 = True
    is_init_packs = False
    is_init_arm = False
    is_init_stage_px = False

    xyz_mm = [65.0, 10.0, -50.0]
    ex_xyz_mm = [205.0, 10.0, 10.0]
    INIT_POS = INIT_POS
    NODES = {}
    INITIAL_DEGS = [100, 90, 90, 165, 85, 90, 80]
    is_nodes_published = {ch: False for ch in CHS}
    current_degs = {ch: _deg for ch, _deg in zip(CHS, INITIAL_DEGS)}
    next_degs = {ch: _deg for ch, _deg in zip(CHS, INITIAL_DEGS)}
    BOWLS = {0: {'x': 105, 'y': 30, 'z': 0}}
    is_down = False

    # SERVO = {'MOVE_PITCH': str(4), 'STEP_SEC': str(0.01)}
    GRIP_STATES = ['GRIP-CLOSED', 'GRIP-OPENED']
    arm_state = ARM_STATES[-1]
    DEBUG_CH = 'pub'
    Hz = 1526

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))
        self.cap = self._init_cap()
        rospy.init_node('publisher', anonymous=True)
        self.NODES = {ch: rospy.Publisher(ch, String, queue_size=10) for ch in CHS}

    def _init_cap(self):
        Log.only(self.DEBUG_CH, 40, '***** Debug mode {}'.format(self.IS_DEV))
        if self.IS_DEV:
            return cv2.VideoCapture(-1)
        else:
            return img_dtct.cap

    def main(self):
        rate = rospy.Rate(self.Hz)
        while not rospy.is_shutdown() and self.is_run_cv2:
            self._handle()
            rate.sleep()
        self.__del__()

    def _handle(self):
        for ch in CHS:
            message = self._compile_message(ch)
            self.NODES[ch].publish(message)
            self.is_nodes_published[ch] = True

            is_false_not_in_publish = False not in self.is_nodes_published.values()
            # is_grip_done = self.is_nodes_published['ch00'] and self.arm_state in self.GRIP_STATES
            if is_false_not_in_publish:
                self._init_packages()
                self._go_to_next_state()
                break
            else:
                pass
        # rospy.sleep(node.calc_sleep_time(self.current_degs, self.next_degs, self.is_down))
        rospy.sleep(1.0)

    def _compile_message(self, ch: str) -> str:
        _from = str(self.current_degs[ch])
        _to = str(self.next_degs[ch])
        _step_time = self.__calc_extra_step_time(ch)
        message = ','.join([ch, _from, _to, SERVO['STR']['MOVE_PITCH'], _step_time])
        if ch == 'ch02':
            Log.only(self.DEBUG_CH, 10, '[ch02 message] {}'.format(message))
        return message

    def __calc_extra_step_time(self, ch):
        if ch in ['ch03', 'ch04']:
            self.is_down = self.xyz_mm[1] > self.ex_xyz_mm[1]
            if self.is_down:
                return SERVO['STR']['STEP_SEC_SLOW']
            else:
                return SERVO['STR']['STEP_SEC']
        else:
            return SERVO['STR']['STEP_SEC']

    def _init_packages(self):
        if not self.IS_DEV:
            if not self.is_init_packs and self.is_init_arm:
                Log.only(self.DEBUG_CH, 30, '***** init packages')
                img_dtct.set_up_darknet()
                occ.frame_px_1_1 = img_dtct.frame_wh
                self.is_init_packs = True
        else:
            self.is_init_packs = True

    def _go_to_next_state(self):
        self.arm_state = self.__get_next_arm_state()
        self.xyz_mm, self.arm_state = self.__get_xyz()
        self.current_degs = self.next_degs
        self.ex_xyz_mm = self.xyz_mm
        node_mms = jdc.calc_node_mms(self.xyz_mm)
        self.next_degs = jdc.calc_node_degs(node_mms, self.arm_state)
        self.is_nodes_published = {_ch: False for _ch in CHS}

    def __get_next_arm_state(self) -> str:
        _next_idx = ARM_STATES.index(self.arm_state) + 1
        next_idx = 0 if _next_idx == len(ARM_STATES) else _next_idx
        arm_state = ARM_STATES[next_idx]
        return arm_state

    def __get_xyz(self) -> [list, str]:
        if not self.is_init_packs:
            img, self.xyz_mm = self.___init_xyz()
        else:
            if self.IS_DEV:
                img = self.___update_xyz_by_catch_key()
            else:
                img, self.xyz_mm, label = self.___update_xyz_by_darknet()
                self.xyz_mm = jdc.adjust_xyz_by_state(self.xyz_mm, self.arm_state, label)
        self.___show_img(img)
        self.xyz_mm, arm_state = self.___key_operations(self.xyz_mm, self.arm_state)
        Log.only(self.DEBUG_CH, 20, 'xyz_mm arm_state {}'.format([self.xyz_mm, arm_state]))
        return self.xyz_mm, arm_state

    def ___init_xyz(self):
        self.is_init_arm = True
        _img = None
        return _img, self.INIT_POS

    def ___update_xyz_by_darknet(self):
        img, xyz_mm, label = None, self.xyz_mm, None
        if self.arm_state in ['LEAVING']:
            img = self.____get_img()
            detections = self.____get_detections(img)
            xyz_mm, label = occ.get_xyz_mm(detections)
            if label is None:
                img, xyz_mm, label = self.___update_xyz_by_darknet()
        return img, xyz_mm, label

    def ____get_img(self):
        img = img_dtct.read_img()
        if not self.is_init_stage_px:
            did_got_stage_px = occ.can_get_stage_px(img)
            if did_got_stage_px:
                self.is_init_stage_px = True
            else:
                img = self.____get_img()
        return img

    def ____get_detections(self, img):
        detections = img_dtct.get_detections(img)
        if len(detections) == 0:
            detections = self.____get_detections(img)
        return detections

    def ___update_xyz_by_catch_key(self):
        # self.xyz = xyz if xyz != None else self.xyz
        Log.only(self.DEBUG_CH, 30, '*** Input any key in w s d a r f')
        ret, img = self.cap.read()
        img = cmn.get_trimmed_img(img)
        if ret:
            img = cv2.resize(img, (NETWORK['W'], NETWORK['H']))
        else:
            img = None
        return img

    @staticmethod
    def ___show_img(img):
        if img is not None:
            cv2.imshow("key_control", img)

    def ___key_operations(self, xyz_mm, arm_state):
        if self.IS_DEV:
            pushed_key = cv2.waitKey(0) % 0xFF
            arm_state = self.____get_arm_state(arm_state)
            if pushed_key == ord('w'):
                print('w')
                xyz_mm[0] += 10.0
            elif pushed_key == ord('s'):
                xyz_mm[0] -= 10.0
            elif pushed_key == ord('d'):
                xyz_mm[2] += 10.0
            elif pushed_key == ord('a'):
                xyz_mm[2] -= 10.0
            elif pushed_key == ord('r'):
                xyz_mm[1] += 5.0
            elif pushed_key == ord('f'):
                xyz_mm[1] -= 5.0
            elif pushed_key == ord('g'):  # 'GRIP-CLOSED', 'GRIP-OPENED'
                arm_state = 'GRIP-CLOSED'
            elif pushed_key == ord('t'):
                arm_state = 'GRIP-OPENED'
            elif pushed_key == ord('q'):
                print('q')
                self.is_run_cv2 = False
                xyz_mm[0] += 1.0
        else:
            pushed_key = cv2.waitKey(1) % 0xFF
            if pushed_key == ord('z'):
                pass
            else:
                pass
        return xyz_mm, arm_state

    @staticmethod
    def ____get_arm_state(arm_state):
        GRIP_STATES = ['GRIP-CLOSED', 'GRIP-OPENED']
        OPENED_STATES = ['APPROACHING', 'TARGETING', 'LEAVING']
        CLOSED_STATES = ['LIFTING_UP', 'REACHING']
        if arm_state not in GRIP_STATES:
            if arm_state in OPENED_STATES:
                arm_state = 'APPROACHING'
            elif arm_state in CLOSED_STATES:
                arm_state = 'LIFTING_UP'
        return arm_state

    def __del__(self):
        if self.IS_DEV:
            cv2.destroyAllWindows()
        else:
            print('***** Finalize {}'.format(__class__.__name__))
            self.is_run_cv2 = False
            # self.cap.release()
            cv2.destroyAllWindows()
            # del img_dtct
            # del occ
            # del jdc


if __name__ == '__main__':
    servo_controller = ServoController()
    servo_controller.main()
