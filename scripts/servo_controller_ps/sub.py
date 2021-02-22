#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import os, time, sys
import Adafruit_PCA9685
import logging
import numpy as np
from scipy.stats import norm

logger = logging.getLogger('LoggingTest')
logger.setLevel(40)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Node():
    pulse = {
        '0_deg': 120.0,
        '180_deg': 610.0,
    }

    # 2.722...
    _1_deg = (pulse['180_deg'] - pulse['0_deg']) / 180.0

    ch = 0
    node_name = ''

    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self._get_channel_number()

    def _get_channel_number(self):
        _ch = sys.argv[1]
        self.ch = (_ch[-2:len(_ch)])
        self.node_name = 'ch' + self.ch
        print('[{}] Init ch {}'.format(self.node_name, self.ch))

    def Loop(self):
        rospy.init_node('listener' + self.ch, anonymous=True)
        rospy.Subscriber('ch' + self.ch, String, self.Call_service)
        rospy.spin()

    def Call_service(self, data):
        try:
            node_name_in_msg, def_from, deg_to, pitch, sleep_sec = self.__parse_message(data.data)
            logger.log(50, '[{}] parse_message message {}'.format(self.node_name, data.data))
            logger.log(50, '[{}] {} {} {} {} '.format(self.node_name, def_from, deg_to, pitch, sleep_sec))
            if self.node_name == node_name_in_msg:
                # self._smooth_move(self.ch, def_from, deg_to, pitch, sleep_sec)
                if self.node_name != 'ch00':
                    self._smooth_move(int(self.ch), def_from, deg_to, pitch, sleep_sec)
                else:
                    self.__quick_move(int(self.ch), def_from, deg_to, pitch)

        except rospy.ServiceException as  e:
            print("Service call failed: %s" % e)

    def __parse_message(self, message):
        [node_name, def_from, deg_to, pitch, sleep_sec] = message.split(',')
        return node_name, int(def_from), int(deg_to), int(pitch), float(sleep_sec)

    def _smooth_move(self, ch, deg_from, deg_to, pitch, sleep_sec):
        ch_num = int(ch)
        pul_from, pul_to, pitch = self.__deg_to_pulse(deg_from, deg_to, pitch)
        if pitch != 0:
            for i in range(pul_from, pul_to, pitch):
                self.pwm.set_pwm(ch_num, 0, i)
                rospy.sleep(sleep_sec)

    def __deg_to_pulse(self, deg_from, deg_to, pitch):
        _pul_from = self.pulse['0_deg'] + (float(deg_from) * self._1_deg)
        _pul_to = self.pulse['0_deg'] + (float(deg_to) * self._1_deg)
        diff = _pul_to - _pul_from
        divide_num = diff / self._1_deg
        remainder, pitch = self.___calc_remainder(_pul_to, _pul_from, divide_num)
        pul_from = int(_pul_from) + remainder + pitch
        pul_to = int(_pul_to) + pitch

        if self.node_name == 'ch02':
            logger.log(50,
                       '[{}] _pul_from, _pul_to, pitch : {} {} {}'.format(self.node_name, _pul_from, _pul_to, pitch))
            logger.log(50, '[{}] diff, divide_num, remainder : {} {} {}'.format(self.node_name, diff, divide_num,
                                                                                remainder))
            logger.log(50, '[{}] deg_from, deg_to :  {} {}'.format(self.node_name, deg_from, deg_to))
            logger.log(50, '[{}] pul_from, pul_to, pitch : {} {} {}'.format(self.node_name, pul_from, pul_to, pitch))

        return pul_from, pul_to, pitch

    def ___calc_remainder(self, _pul_to, _pul_from, divide_num):
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

    def __quick_move(self, ch_int, deg_from, deg_to, pitch):
        pul_from, pul_to, pitch = self.__deg_to_pulse(deg_from, deg_to, pitch)
        pulse = pul_to - pitch
        logger.log(20, '[{}] node_name pulse {} {}'.format(self.node_name, ch_int, pulse, ))
        self.pwm.set_pwm(ch_int, 0, pulse)



if __name__ == "__main__":
    node = Node()
    node.Loop()
