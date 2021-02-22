#! /usr/bin/env python3
import time
import rospy
from std_srvs.srv import Empty, SetBool, Trigger


def call_service():
    rospy.loginfo('waiting service')
    rospy.wait_for_service('ch0')
    try:
        service = rospy.ServiceProxy('ch0', Trigger)
        response = service()
        print('[INFO] {}\n\n'.format(response))
        time.sleep(1)
        if response.success:
            call_service()
    except rospy.ServiceException as  e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    print('run client.py')
    call_service()
