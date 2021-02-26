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
logger.setLevel(40)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Servo_Controller():
    # x y x = height
    xyz = [0.0, 0.0, 0.0]
    nodes = []
    published_nodes = {'ch00': False, 'ch01': False, 'ch02': False, 'ch03': False, 'ch04': False, 'ch05': False, }
    current_pos = {'ch00': 135, 'ch01': 90, 'ch02': 90, 'ch03': 90, 'ch04': 90, 'ch05': 90}
    next_pos = {'ch00': 90, 'ch01': 90, 'ch02': 90, 'ch03': 90, 'ch04': 90, 'ch05': 90}

    arm_states = ['approaching', 'targeting', 'grip-holding', 'lifting_up', 'reaching', 'grip-releasing']
    arm_states_only_grip = ['grip-holding', 'grip-releasing']
    arm_state = arm_states[-1]

    pitch = str(4)
    sleep_sec = str(0.01)
    ch = 'pub'
    waiting_time = 0

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))
        rospy.init_node('publisher', anonymous=True)
        for i in range(6):
            ch = '{:0=2}'.format(i)
            pub = rospy.Publisher('ch' + ch, String, queue_size=10)
            self.nodes.append(pub)

    def Main(self, ):
        hz = 1526
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            self.Handle()
            rate.sleep()

    def Handle(self):
        def compile_message(self, node_name):
            _from = str(self.current_pos[node_name])
            _to = str(self.next_pos[node_name])
            message = ','.join([node_name, _from, _to, self.pitch, self.sleep_sec])
            return message

        for i in range(6):
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
        # sys.exit()

    def go_to_next_state(self):
        def __get_next_arm_state(self):
            idx = self.arm_states.index(self.arm_state)
            next_index = 0
            if idx != len(self.arm_states) - 1:
                next_index = idx + 1
            self.arm_state = self.arm_states[next_index]

        def __generate_xyz(self):
            x = 120.0
            if self.arm_state == 'approaching':
                next_xyz = [x, 50.0, 50.0]
                self.xyz = [int(next_xyz[0]), int(next_xyz[1]), int(next_xyz[2])]
            elif self.arm_state == 'targeting':
                next_xyz = [x, 0.0, 50.0]
                self.xyz = [int(next_xyz[0]), int(next_xyz[1]), int(next_xyz[2])]
            elif self.arm_state == 'grip-holding':
                rospy.sleep(0)
                pass
            elif self.arm_state == 'lifting_up':
                next_xyz = [x, 50.0, 20.0]
                self.xyz = [int(next_xyz[0]), int(next_xyz[1]),
                            int(next_xyz[2]) + random.randrange(-30.0, 30.0, 10.0)]
            elif self.arm_state == 'reaching':
                self.xyz = [self.xyz[0], self.xyz[1], self.xyz[2] - 50.0]
            elif self.arm_state == 'grip-releasing':
                pass

        def __init_published_flag(self):
            for node in self.published_nodes.keys():
                self.published_nodes[node] = False

        __get_next_arm_state(self)
        __generate_xyz(self)
        self.current_pos = self.next_pos

        Pos_Params = joint_degree.Get_Lengths(self.xyz)
        self.next_pos = joint_degree.Get_Thetas(Pos_Params, self.arm_state)
        __init_published_flag(self)

    def calc_waiting_time_to_next_publish(self):
        def get_max_pulse_diff():
            diffs = []
            for key in self.current_pos:
                diff = abs(self.current_pos[key] - self.next_pos[key])
                diffs.append(diff)
            max_deff_key = 'ch0' + str(diffs.index(max(diffs)))
            return self.current_pos[max_deff_key], self.next_pos[max_deff_key]

        times = 0
        max_deg_from, max_deg_to = get_max_pulse_diff()
        Log.intervally(self.ch, 40, 'from {} to {}'.format(max_deg_from, max_deg_to))
        pulses, pitchs = node.Deg_To_Pulse(max_deg_from, max_deg_to, int(self.pitch))
        Log.intervally(self.ch, 40, 'pulses {}\n pitchs {}'.format(pulses, pitchs))
        if pulses[0] - pulses[1] != 0:
            for i, pulse in enumerate(pulses):
                if i == len(pulses) - 1:
                    break
                for j in range(pulses[i], pulses[i + 1], pitchs[i]):
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
