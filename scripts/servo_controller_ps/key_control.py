#!/usr/bin/python3
import os, time, re, sys, random

import numpy as np
import rospy
from std_msgs.msg import String
from joint_degree_calculator import Joint_Degree_Calculator
import logging
from sub import Node
import cv2
from log_manager import Log_Manager

# from global_settings import *

joint_degree = Joint_Degree_Calculator()
node = Node()
Log = Log_Manager()

logger = logging.getLogger('LoggingTest')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Servo_Controller():
    # x y x = height
    xyz = [100.0, 0.0, 0.0]
    nodes = []
    published_nodes = {'ch00': False, 'ch01': False, 'ch02': False, 'ch03': False, 'ch04': False, 'ch05': False,
                       'ch06': False, }
    current_pos = {'ch00': 90, 'ch01': 90, 'ch02': 45, 'ch03': 45, 'ch04': 45, 'ch05': 45, 'ch06': 90, }
    next_pos = {'ch00': 90, 'ch01': 90, 'ch02': 45, 'ch03': 45, 'ch04': 45, 'ch05': 45, 'ch06': 90, }
    nodes_len = len(published_nodes.values())

    arm_states = ['approaching', 'targeting', 'grip-holding', 'lifting_up', 'reaching', 'grip-releasing']
    arm_states_only_grip = ['grip-holding', 'grip-releasing']
    arm_state = arm_states[-1]

    pitch = str(4)
    sleep_sec = str(0.01)
    ch = 'pub'
    waiting_time = 0

    run_cv2 = True

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        print('***** Init {}'.format(os.path.basename(__file__)))
        rospy.init_node('publisher', anonymous=True)
        for i in range(self.nodes_len):
            ch = '{:0=2}'.format(i)
            pub = rospy.Publisher('ch' + ch, String, queue_size=10)
            self.nodes.append(pub)

    def Main(self, ):
        hz = 1526
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown() and self.run_cv2:
            self.Handle()
            rate.sleep()
        self.cap.release()
        cv2.destroyAllWindows()

    def Handle(self):
        def compile_message(self, node_name):
            _from = str(self.current_pos[node_name])
            _to = str(self.next_pos[node_name])
            message = ','.join([node_name, _from, _to, self.pitch, self.sleep_sec])
            return message

        for i in range(self.nodes_len):
            ch = 'ch' + '{:0=2}'.format(i)
            message = compile_message(self, ch)
            if self.current_pos[ch] != self.next_pos[ch]:
                self.nodes[i].publish(message)
            self.published_nodes[ch] = True

            if False not in self.published_nodes.values():
                self.go_to_next_state()
                rospy.sleep(self.calc_waiting_time_to_next_publish())
            else:
                if self.published_nodes['ch00'] and (self.arm_state in self.arm_states_only_grip):
                    self.go_to_next_state()
                    rospy.sleep(self.calc_waiting_time_to_next_publish())
                else:
                    pass

    def go_to_next_state(self):
        def __get_next_arm_state(self):
            idx = self.arm_states.index(self.arm_state)
            next_index = 0
            if idx != len(self.arm_states) - 1:
                next_index = idx + 1
            self.arm_state = self.arm_states[next_index]

        def __catch_key(self):
            ex_xyz = self.xyz
            logger.log(20, '*** Input any key in w s d a r f'.format(self.xyz[0], self.xyz[1], self.xyz[2], ))

            ret, image = self.cap.read()
            h, w, c = np.shape(image)
            if ret:
                image = cv2.resize(image, (w // 10, h // 10))
                # while ex_xyz == self.xyz:
                cv2.imshow("image", image)
                ret, image = self.cap.read()
                c = cv2.waitKey(0) % 0xFF
                if c == ord('w'):
                    print('w')
                    self.xyz[0] += 10.0
                elif c == ord('s'):
                    self.xyz[0] -= 10.0
                elif c == ord('d'):
                    self.xyz[2] += 10.0
                elif c == ord('a'):
                    self.xyz[2] -= 10.0
                elif c == ord('r'):
                    self.xyz[1] += 10.0
                elif c == ord('f'):
                    self.xyz[1] -= 10.0
                elif c == ord('g'):  # 'grip-holding', 'grip-releasing'
                    self.arm_state = 'grip-holding'
                elif c == ord('t'):
                    self.arm_state = 'grip-releasing'
                elif c == ord('q'):
                    print('q')
                    self.run_cv2 = False
                    self.xyz[0] += 1.0
                else:
                    pass
            logger.log(20, 'x:{} y:{} z:{}'.format(self.xyz[0], self.xyz[1], self.xyz[2], ))

        def __init_published_flag(self):
            for node in self.published_nodes.keys():
                self.published_nodes[node] = False

        __get_next_arm_state(self)
        __catch_key(self)
        self.current_pos = self.next_pos

        Pos_Params = joint_degree.Get_Lengths(self.xyz)
        self.next_pos = joint_degree.Get_Thetas(Pos_Params, self.arm_state)
        logger.log(40, '[Pos_Params] {}'.format(Pos_Params))
        logger.log(40, '[Joint] {}'.format(self.next_pos))
        logger.log(40,
                   '[02, 03 ,04 Sum] {}'.format(self.next_pos['ch02'] + self.next_pos['ch03'] + self.next_pos['ch04']))
        __init_published_flag(self)

    def calc_waiting_time_to_next_publish(self):
        def get_max_pulse_diff():
            diffs = []
            for key in self.current_pos:
                diff = abs(self.current_pos[key] - self.next_pos[key])
                diffs.append(diff)
            max_diff_key = 'ch0' + str(diffs.index(max(diffs)))
            return self.current_pos[max_diff_key], self.next_pos[max_diff_key]

        times = 0
        max_deg_from, max_deg_to = get_max_pulse_diff()
        Log.intervally(self.ch, 40, 'from {} to {}'.format(max_deg_from, max_deg_to))
        pulses, pitches = node.Deg_To_Pulse(max_deg_from, max_deg_to, int(self.pitch))
        Log.intervally(self.ch, 40, 'pulses {}\n pitches {}'.format(pulses, pitches))
        if pulses[0] - pulses[1] != 0:
            for i, pulse in enumerate(pulses):
                if i == len(pulses) - 1:
                    break
                for j in range(pulses[i], pulses[i + 1], pitches[i]):
                    times += 1

        waiting_time = float(self.sleep_sec) * times
        Log.intervally(self.ch, 40,
                       'waiting_time {}\n     froms {}\n     tos {}'.format(waiting_time, self.current_pos,
                                                                            self.next_pos))
        # Log.interval = waiting_time
        return waiting_time


if __name__ == '__main__':
    servo_controller = Servo_Controller()
    servo_controller.Main()
