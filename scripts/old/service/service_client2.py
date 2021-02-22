#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty


def call_service():
    rospy.loginfo('waiting service')
    # call_meという名前のServerが立ち上がるのを待つ
    rospy.wait_for_service('call_me2')
    try:
        # call_meという名前でEmpty型のClientを作成
        service = rospy.ServiceProxy('call_me2', Empty)
        # Clientを呼び出す
        response = service()
    except rospy.ServiceException as  e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    call_service()
