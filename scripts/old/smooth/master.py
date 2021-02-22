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
import cv2

logger = logging.getLogger('LoggingTest')
logger.setLevel(30)
sh = logging.StreamHandler()
logger.addHandler(sh)


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
import cv2

logger = logging.getLogger('LoggingTest')
logger.setLevel(30)
sh = logging.StreamHandler()
logger.addHandler(sh)

class Smooth():
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
        logger.log(30, '[INFO] self.published_nodes {}'.format(self.published_nodes))

        # logger.log(20, '[INFO] self.published_nodes.values() {}'.format(self.published_nodes.values()))
        #
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


if __name__ == '__main__':
    smooth = Smooth()
    smooth.move()
