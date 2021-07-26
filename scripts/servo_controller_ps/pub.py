#!/usr/bin/python3
import os, time, re, sys, random
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
    xyz = [157.0, 0.0, 0.0]
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
            _from = str(self.current_degs[node_name])
            _to = str(self.next_degs[node_name])
            message = ','.join([node_name, _from, _to, self.MOVE_PITCH, self.SLEEP_SEC])
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
            idx = self.ARM_STATES.index(self.arm_state)
            next_index = 0
            if idx != len(self.ARM_STATES) - 1:
                next_index = idx + 1
            self.arm_state = self.ARM_STATES[next_index]

        def __generate_xyz(self):
            x = 120.0
            if self.arm_state == 'approaching':
                next_xyz = [x, 50.0, 50.0]
                self.xyz_mm = [int(next_xyz[0]), int(next_xyz[1]), int(next_xyz[2])]
            elif self.arm_state == 'targeting':
                next_xyz = [x, 0.0, 50.0]
                self.xyz_mm = [int(next_xyz[0]), int(next_xyz[1]), int(next_xyz[2])]
            elif self.arm_state == 'grip-holding':
                rospy.sleep(0)
                pass
            elif self.arm_state == 'lifting_up':
                next_xyz = [x, 50.0, 20.0]
                self.xyz_mm = [int(next_xyz[0]), int(next_xyz[1]),
                               int(next_xyz[2]) + random.randrange(-30.0, 30.0, 10.0)]
            elif self.arm_state == 'reaching':
                self.xyz_mm = [self.xyz_mm[0], self.xyz_mm[1], self.xyz_mm[2] - 50.0]
            elif self.arm_state == 'grip-releasing':
                pass

        def __init_published_flag(self):
            for node in self.is_nodes_published.keys():
                self.is_nodes_published[node] = False

        __get_next_arm_state(self)
        __generate_xyz(self)
        self.current_pos = self.next_pos

        Pos_Params = joint_degree.calc_node_dists(self.xyz)
        self.next_pos = joint_degree.calc_node_degs(Pos_Params, self.arm_state)
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
        pulses, pitches = node.deg_to_pulse(max_deg_from, max_deg_to, int(self.pitch))
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
