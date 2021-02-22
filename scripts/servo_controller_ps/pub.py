#!/usr/bin/python3
import os, time, re, sys, random
import rospy
from std_msgs.msg import String
from joint_degree_calculator import Joint_Degree_Calculator
import logging
import cv2

print(sys.version)

logger = logging.getLogger('LoggingTest')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Servo_Controller():
    xyz = [0.0, 0.0, 0.0]
    nodes = []
    hz = 0
    published_nodes = {'ch00': False, 'ch01': False, 'ch02': False, 'ch03': False, 'ch04': False, 'ch05': False, }
    _froms = {'ch00': 180, 'ch01': 90, 'ch02': 90, 'ch03': 90, 'ch04': 90, 'ch05': 90}
    _tos = {'ch00': 180, 'ch01': 90, 'ch02': 90, 'ch03': 90, 'ch04': 90, 'ch05': 90}

    arm_states = ['targeting', 'grip-holding', 'reaching', 'grip-releasing']
    arm_states_only_grip = [arm_states[1], arm_states[3]]
    arm_state = arm_states[-1]

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))
        self.joint_degree = Joint_Degree_Calculator()

        rospy.init_node('publisher', anonymous=True)
        for i in range(6):
            ch = '{:0=2}'.format(i)
            pub = rospy.Publisher('ch' + ch, String, queue_size=10)
            self.nodes.append(pub)

    def Main(self, ):
        self.hz = 1526
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.Handle()
            rate.sleep()

    def Handle(self):
        for i in range(6):
            ch = 'ch' + '{:0=2}'.format(i)
            message = self.compile_message(ch)
            if self._froms[ch] != self._tos[ch]:
                self.nodes[i].publish(message)
            self.published_nodes[ch] = True

            if False not in self.published_nodes.values():
                self.go_to_next_state()
            else:
                if self.published_nodes['ch00'] and (self.arm_state in self.arm_states_only_grip):
                    self.go_to_next_state()
                else:
                    pass
        self.__logger_1(20, 'Sub')

    def compile_message(self, node_name):
        _from = str(self._froms[node_name])
        _to = str(self._tos[node_name])
        pitch = str(1)
        # sleep_sec = str(0.02)
        sleep_sec = str(0.1)
        message = ','.join([node_name, _from, _to, pitch, sleep_sec])
        return message

    def go_to_next_state(self):
        def __what_is_next_arm_state(self):
            idx = self.arm_states.index(self.arm_state)
            if idx == len(self.arm_states) - 1:
                idx = 0
            else:
                idx += 1
            self.arm_state = self.arm_states[idx]

        def __swap_from_to(self):
            self._froms = self._tos

        def __go_to_goal_box(self):
            _xyz = self.xyz
            self.xyz = [10.0, _xyz[1] + 50.0, _xyz[2] - random.randrange(90.0, 110.0)]

        def __generate_xyz(self):
            rospy.sleep(1)
            if self.arm_state == self.arm_states[0]:
                rospy.sleep(1)
                xyz = [120.0, 5.0, 0.0]
                self.xyz = [int(xyz[0]), int(xyz[1]), int(xyz[2])]
            elif self.arm_state == self.arm_states[1]:
                rospy.sleep(0.5)
                pass
            elif self.arm_state == self.arm_states[2]:
                rospy.sleep(1)
                __go_to_goal_box(self)
            elif self.arm_state == self.arm_states[3]:
                pass

        def __init_published_flag(self):
            for node in self.published_nodes.keys():
                self.published_nodes[node] = False

        __what_is_next_arm_state(self)
        __swap_from_to(self)
        __generate_xyz(self)

        lengths = self.joint_degree.Get_Lengths(self.xyz)
        self._tos = self.joint_degree.Get_Thetas(lengths, self.arm_state)

        # self.__adjust_degree()
        __init_published_flag(self)

    def __logger_1(self, level, comment):
        logger.log(50, '[{}] message {}'.format(comment, ''))
        logger.log(level, '[{}] self._tos {}'.format(comment, self._tos))
        logger.log(level, '[{}] self.arm_state {}'.format(comment, self.arm_state))


if __name__ == '__main__':
    servo_controller = Servo_Controller()
    servo_controller.Main()
