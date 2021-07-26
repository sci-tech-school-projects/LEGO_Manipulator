#!/usr/bin/python3
import os, time, re, sys, random

import numpy as np
import rospy
from std_msgs.msg import String
from joint_degree_calculator import Joint_Degree_Calculator
from object_coordinate_calculator import Object_Coordinate_Calculator
import logging
from sub import Node
import cv2
from log_manager import Log_Manager
from config import *
from common import *
from image_detection import Image_Detection

jdc = Joint_Degree_Calculator()
occ = Object_Coordinate_Calculator()
occ.get_stage_px()
img_dtct = Image_Detection()
occ.cap = img_dtct.cap

node = Node()
Log = Log_Manager()

logger = logging.getLogger('LoggingTest')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Servo_Controller():
    IS_DEV = True
    is_run_cv2 = True
    xyz_mm = [205.0, 0.0, 0.0]
    NODES = []
    INITIAL_DEGS = [90, 90, 45, 45, 45, 45, 90]
    is_nodes_published = {ch: False for ch in CHS}
    current_degs = {ch: _deg for ch, _deg in zip(CHS, INITIAL_DEGS)}
    next_degs = {ch: _deg for ch, _deg in zip(CHS, INITIAL_DEGS)}

    SERVO = {'MOVE_PITCH': str(4), 'SLEEP_SEC': str(0.01)}
    ARM_STATES = ['APPROACHING', 'TARGETING', 'GRIP-CLOSED', 'LIFTING_UP', 'REACHING', 'GRIP-OPENED']
    GRIP_STATES = ['GRIP-CLOSED', 'GRIP-OPENED']
    arm_state = ARM_STATES[-1]
    DEBUG_CH = 'pub'

    def __init__(self):
        if not img_dtct.is_darknet_run:
            self.cap = cv2.VideoCapture(-1)
        else:
            self.cap = img_dtct.cap
        print('***** Init {}'.format(os.path.basename(__file__)))
        rospy.init_node('publisher', anonymous=True)
        for i in range(len(CHS)):
            ch = '{:0=2}'.format(i)
            pub = rospy.Publisher('ch' + ch, String, queue_size=10)
            self.NODES.append(pub)

    def main(self, ):
        hz = 1526
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown() and self.is_run_cv2:
            self._handle()
            rate.sleep()
        self.cap.release()
        cv2.destroyAllWindows()

    def _handle(self):
        for i in range(len(CHS)):
            ch = 'ch' + '{:0=2}'.format(i)
            if self.current_degs[ch] != self.next_degs[ch]:
                message = self._compile_message(ch)
                self.NODES[i].publish(message)

            self.is_nodes_published[ch] = True

            if False not in self.is_nodes_published.values():
                self._go_to_next_state()
            elif self.is_nodes_published['ch00'] and self.arm_state in self.GRIP_STATES:
                self._go_to_next_state()
            else:
                pass

    def _compile_message(self, ch: str) -> str:
        _from = str(self.current_degs[ch])
        _to = str(self.next_degs[ch])
        message = ','.join([ch, _from, _to, self.SERVO['MOVE_PITCH'], self.SERVO['SLEEP_SEC']])
        Log.intervally(self.DEBUG_CH, 10, '[message] {}'.format(message))
        return message

    def _go_to_next_state(self):
        self.arm_state = self.__get_next_arm_state()
        if self.IS_DEV:
            self.__update_xyz_by_catch_key()
        else:
            img, detections, self.xyz_mm = self.__update_xyz_by_darknet()

        self.current_degs = self.next_degs

        node_dists = jdc.calc_node_dists(self.xyz_mm)
        self.next_degs = jdc.calc_node_degs(node_dists, self.arm_state)

        self.__init_published_flag()
        rospy.sleep(self.__calc_waiting_time_to_next_publish())

    def __init_published_flag(self):
        for node in self.is_nodes_published.keys():
            self.is_nodes_published[node] = False

    def __calc_waiting_time_to_next_publish(self):
        deg_old, deg_new = self.___get_max_pulse_diff()
        pulses, pitches = node.deg_to_pulse([deg_old, deg_new, int(self.SERVO['MOVE_PITCH'])])
        Log.intervally(self.DEBUG_CH, 40, 'pulses {}\n pitches {}'.format(pulses, pitches))
        times = 0
        if pulses[0] - pulses[1] != 0:
            for i, pulse in enumerate(pulses):
                if i == len(pulses) - 1:
                    break
                for j in range(pulses[i], pulses[i + 1], pitches[i]):
                    times += 1
        waiting_time = float(self.SERVO['SLEEP_SEC']) * times
        return waiting_time

    def __get_next_arm_state(self, arm_state=None) -> str:
        arm_state = self.arm_state if arm_state == None else arm_state
        idx = self.ARM_STATES.index(arm_state)
        next_index = 0
        if idx != len(self.ARM_STATES) - 1:
            next_index = idx + 1
        arm_state = self.ARM_STATES[next_index]
        return arm_state

    def __update_xyz_by_darknet(self):
        """
        {'start': {'x': 0, 'y': 0}, 'end': {'x': 0, 'y': 0}, 'center': {'x': 0, 'y': 0}, }
        :return:
        """
        img, detections, FRAME_PX = img_dtct.main()
        if not occ.got_darknet_frame: occ.frame_px_1_1 = FRAME_PX
        xyz_mm = occ.get_xyz_mm(detections)
        xyz_mm[1] = -4.0 if self.arm_state in self.ARM_STATES[1:3] else 0.0
        return img, detections, xyz_mm

    def __update_xyz_by_catch_key(self, xyz=None):
        # self.xyz = xyz if xyz != None else self.xyz
        logger.log(20, '*** Input any key in w s d a r f'.format(self.xyz_mm[0], self.xyz_mm[1], self.xyz_mm[2], ))
        ret, img = self.cap.read()
        if ret:
            h, w, c = np.shape(img)
            img = cv2.resize(img, (zoomed_25(w), zoomed_25(h)))
            cv2.imshow("image", img)
            pushed_key = cv2.waitKey(0) % 0xFF
            if pushed_key == ord('w'):
                print('w')
                self.xyz_mm[0] += 10.0
            elif pushed_key == ord('s'):
                self.xyz_mm[0] -= 10.0
            elif pushed_key == ord('d'):
                self.xyz_mm[2] += 10.0
            elif pushed_key == ord('a'):
                self.xyz_mm[2] -= 10.0
            elif pushed_key == ord('r'):
                self.xyz_mm[1] += 5.0
            elif pushed_key == ord('f'):
                self.xyz_mm[1] -= 5.0
            elif pushed_key == ord('g'):  # 'GRIP-CLOSED', 'GRIP-OPENED'
                self.arm_state = 'GRIP-CLOSED'
            elif pushed_key == ord('t'):
                self.arm_state = 'GRIP-OPENED'
            elif pushed_key == ord('q'):
                print('q')
                self.is_run_cv2 = False
                self.xyz_mm[0] += 1.0
            else:
                pass
        logger.log(20, 'x y z {}'.format(self.xyz_mm))
        # if xyz != None: return xyz

    def ___get_max_pulse_diff(self, degs=None) -> [int, int]:
        current_degs, next_degs = [self.current_degs, self.next_degs] if None == degs else degs
        diffs = []
        for ch in CHS:
            diff = abs(current_degs[ch] - next_degs[ch])
            diffs.append(diff)
        max_diff_key = 'ch0' + str(diffs.index(max(diffs)))
        deg_old, deg_new = current_degs[max_diff_key], next_degs[max_diff_key]
        Log.intervally(self.DEBUG_CH, 40, 'old {} new {}'.format(deg_old, deg_new))
        return [deg_old, deg_new]


if __name__ == '__main__':
    servo_controller = Servo_Controller()
    servo_controller.main()
