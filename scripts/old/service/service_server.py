#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Trigger, TriggerResponse

global s, s2


def handle_service(req):
    # req returns ""
    response = TriggerResponse()
    response.success = True
    response.message = '180'
    return response


def service_server():
    rospy.init_node('service_server')
    s = rospy.Service('ch0', Trigger, handle_service)
    # s2 = rospy.Service('ch2', Empty, handle_service)
    # print('s: {}'.format(s))
    # print('s2: {}'.format(s2))
    rospy.spin()


if __name__ == '__main__':
    print('run server.py')
    service_server()
