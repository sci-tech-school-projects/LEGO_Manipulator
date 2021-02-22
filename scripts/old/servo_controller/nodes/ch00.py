#!/usr/bin/env python3
import os, time
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import Adafruit_PCA9685
import logging

logger = logging.getLogger('LoggingTest')
logger.setLevel(40)
sh = logging.StreamHandler()
logger.addHandler(sh)

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

pulse = {
    '0_deg': 120.0,
    '180_deg': 610.0,
}

# 2.722...
_1_deg = (pulse['180_deg'] - pulse['0_deg']) / 180.0


def _get_channel_number():
    file_name = os.path.basename(__file__)
    ch = file_name[2:4]
    node_name = 'ch' + ch
    return ch, node_name


def __parse_message(message):
    [node_name, def_from, deg_to, pitch, sleep_sec] = message.split(',')
    return node_name, int(def_from), int(deg_to), int(pitch), float(sleep_sec)


def ___calc_remainder(_pul_to, _pul_from, divide_num):
    remainder = 0
    pitch = 0
    if divide_num == 0:
        remainder = 0
        pitch = 0
    else:
        remainder = int((_pul_to - _pul_from) % divide_num)
        # pitch = int((_pul_to - _pul_from) / divide_num)
        if (_pul_to - _pul_from) >= 0:
            pitch = 5
        else:
            pitch = -5
    return remainder, pitch


def __deg_to_pulse(deg_from, deg_to, pitch):
    _pul_from = pulse['0_deg'] + (float(deg_from) * _1_deg)
    _pul_to = pulse['0_deg'] + (float(deg_to) * _1_deg)
    diff = _pul_to - _pul_from
    divide_num = diff / _1_deg
    remainder, pitch = ___calc_remainder(_pul_to, _pul_from, divide_num)
    pul_from = int(_pul_from) + remainder + pitch
    pul_to = int(_pul_to) + pitch

    logger.log(30, 'diff, divide_num, remainder : {} {} {}'.format(diff, divide_num, remainder))
    logger.log(30, '_pul_from, _pul_to, pitch : {} {} {}'.format(_pul_from, _pul_to, pitch))
    logger.log(30, 'pul_from, pul_to, pitch : {} {} {}'.format(pul_from, pul_to, pitch))

    return pul_from, pul_to, pitch


def _smooth_move(ch, deg_from, deg_to, pitch, sleep_sec):
    # sleep_sec = 0.05
    ch_num = int(ch)
    logger.log(30, '[INFO] ch, ch_num {} {}'.format(ch, ch_num))
    pul_from, pul_to, pitch = __deg_to_pulse(deg_from, deg_to, pitch)
    if pitch != 0:
        for i in range(pul_from, pul_to, pitch):
            logger.log(30, 'type {} {}'.format(type(i), i))
            pwm.set_pwm(ch_num, 0, i)
            rospy.sleep(sleep_sec)
    else:
        rospy.sleep(sleep_sec * 5)


def call_service(ch, node_name):
    rospy.wait_for_service(node_name)
    try:
        header = {'node_name': node_name}
        service = rospy.ServiceProxy(node_name, Trigger, headers=header)
        response = service()
        if response.success:
            logger.log(20, '[INFO] {}'.format(response.message))
            node_name_in_msg, def_from, deg_to, pitch, sleep_sec = __parse_message(response.message)
            if node_name == node_name_in_msg:
                _smooth_move(ch, def_from, deg_to, pitch, sleep_sec)
                # rospy.sleep(1)
            call_service(ch, node_name)
    except rospy.ServiceException as  e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    ch, node_name = _get_channel_number()
    call_service(ch, node_name)
