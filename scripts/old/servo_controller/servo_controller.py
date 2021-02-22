#!/usr/bin/env python3
"""
usage of TriggerResponse is in below
https://github.com/leggedrobotics/free_gait/blob/master/free_gait_action_loader/bin/free_gait_action_loader/action_loader.py
"""
import os, time, re, sys, random
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from joint_degree_calculator import Joint_Degree_Calculator
import logging

# import cv2

logger = logging.getLogger('LoggingTest')
logger.setLevel(30)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Servo_Controller():
    published_nodes = {'ch00': False, 'ch01': False, 'ch02': False, 'ch03': False, 'ch04': False, 'ch05': False, }
    _from = 75
    _to = 90
    _froms = {'ch00': 90, 'ch01': 90, 'ch02': _from, 'ch03': _from, 'ch04': _from, 'ch05': 90}
    _tos = {'ch00': 90, 'ch01': 90, 'ch02': _to, 'ch03': _to, 'ch04': _to, 'ch05': 90}
    adjusting_degrees = {'ch00': 0, 'ch01': 0, 'ch02': 45, 'ch03': -45, 'ch04': 0, 'ch05': 0}
    arm_states = ['targeting', 'grip-holding', 'lifting', 'reaching', 'grip-releasing']
    arm_states_only_grip = [arm_states[1], arm_states[4]]
    arm_state = arm_states[-1]
    xyz = [100.0, 5.0, 20.0]

    def __init__(self):
        print('***** Init {}'.format(os.path.basename(__file__)))
        self.joint_degree = Joint_Degree_Calculator()

    def Main(self, ):
        rospy.init_node('servo_server')
        for i in range(6):
            ch = '{:0=2}'.format(i)
            s = rospy.Service('ch' + ch, Trigger, self.Handle_service)
        rospy.spin()

    def Handle_service(self, req):
        node_name = req._connection_header['node_name']

        response = self._compile_response(node_name)
        self.published_nodes[node_name] = True
        self.__logger_1(20, 'Handle', response, )

        if False not in self.published_nodes.values():
            self.__what_is_next_arm_state()
            self.__go_to_next_state()
        else:
            if self.published_nodes['ch00'] and (self.arm_state in self.arm_states_only_grip):
                self.__what_is_next_arm_state()
                self.__go_to_next_state()
            else:
                pass
        return response

    def __logger_1(self, level, comment, response, ):
        logger.log(50, '[{}] message {}'.format(comment, response.message))
        logger.log(level, '[{}] self._tos {}'.format(comment, self._tos))
        logger.log(level, '[{}] self.arm_state {}'.format(comment, self.arm_state))

    def _compile_response(self, node_name):
        _from = str(self._froms[node_name])
        _to = str(self._tos[node_name])
        pitch = str(5)
        sleep_sec = str(0.01)
        message = ','.join([node_name, _from, _to, pitch, sleep_sec])

        response = TriggerResponse()
        response.success = True
        response.message = message
        return response

    def __what_is_next_arm_state(self):
        idx = self.arm_states.index(self.arm_state)
        if idx == len(self.arm_states) - 1:
            idx = 0
        else:
            idx += 1
        self.arm_state = self.arm_states[idx]

    def __go_to_next_state(self):
        self.__logger_2(20, 'Before')
        self.__swap_from_to()
        self.__generate_xyz(self.arm_state)

        lengths = self.joint_degree.Get_Lengths(self.xyz)
        joint_degrees = self.joint_degree.Get_Thetas(lengths, self.arm_state)
        self._tos = joint_degrees

        self.__adjust_degree()
        self.__init_published_flag()
        self.__logger_2(20, 'After')

    def __logger_2(self, level, comment):
        logger.log(level, '[{}] self.xyz {}'.format(comment, str(self.xyz)))
        logger.log(level, '[{}] self._tos {}'.format(comment, self._tos))
        logger.log(level, '[{}] self._froms {}'.format(comment, self._froms))

    def __swap_from_to(self):
        self._froms = self._tos

    def __generate_xyz(self, arm_status):
        #    arm_states = ['targeting', 'grip-holding', 'lifting', 'reaching', 'grip-releasing']
        if arm_status == self.arm_states[0]:
            rospy.sleep(1)
            xyz = [random.uniform(50, 80), random.uniform(0, 10), 0.0]
            self.xyz = [int(xyz[0]), int(xyz[1]), int(xyz[2])]
        elif arm_status == self.arm_states[1]:
            pass
        elif arm_status == self.arm_states[2]:
            self.xyz[1] = self.xyz[1] + 30.0
        elif arm_status == self.arm_states[3]:
            self.__go_to_goal_box()
        elif arm_status == self.arm_states[4]:
            pass

    def __adjust_degree(self):
        for node in self.published_nodes.keys():
            self._tos[node] = self._tos[node] + self.adjusting_degrees[node]

    def __init_published_flag(self):
        for node in self.published_nodes.keys():
            self.published_nodes[node] = False

    def __go_to_goal_box(self):
        self.xyz[0] = self.xyz[0] + 20.0


if __name__ == '__main__':
    servo_controller = Servo_Controller()
    servo_controller.Main()
