#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import os, time, sys
import Adafruit_PCA9685
import logging
import numpy as np
from common import *
from config import *
from log_manager import Log_Manager, logger

Log = Log_Manager()


class Node():
    PULSES = {'0_DEG': 121.0, '90_DEG': 355.0, '180_DEG': 621.0, }
    # 2.722...
    PULSE_PER_DEG: float = round_3((PULSES['180_DEG'] - PULSES['90_DEG']) / 90.0)
    CH_NUM: int = None
    CH_STR: str = None
    CH: str = None
    PWM = None
    msg = {'ch': None, 'deg_old': None, 'deg_new': None, 'pitch': None, 'sleep_sec': None}

    def __init__(self):
        self._get_channel_number()
        if type(self.CH_NUM) == int:
            self.PWM = Adafruit_PCA9685.PCA9685()
            self.PWM.set_pwm_freq(60)
        else:
            pass

    def _get_channel_number(self):
        try:
            _ch = sys.argv[1]
            _ch = _ch.split('__name:=')
            ch = _ch[1]
            print(ch)

            if ch == 'publisher':
                self.CH = 'pub'
            else:
                self.CH_STR = ch[-2::]
                self.CH = 'ch' + self.CH_STR
                self.CH_NUM = int(self.CH_STR)
            print('[{}] Init ch {}'.format(self.CH, self.CH_NUM))
        except IndexError:
            raise Exception('Needed "name" in launch file for argument')

    def loop(self):
        rospy.init_node('listener' + self.CH_STR, anonymous=True)
        rospy.Subscriber('ch' + self.CH_STR, String, self._call_service)
        rospy.spin()

    def _call_service(self, data):
        try:
            self.msg = self.__parse_message(data.data)
            if self.CH == self.msg['ch']: self.__smooth_move()
        except rospy.ServiceException as  e:
            print("Service call failed: %s" % e)

    def __parse_message(self, message: str) -> {str, int, int, int, float}:
        ch, deg_old, deg_new, pitch, slp_sec = message.split(',')
        Log.intervally(self.CH, 20, '[{}] parse_message {}'.format(ch, message))
        Log.intervally(self.CH, 20, '[{}] {} '.format(ch, [deg_old, deg_new, pitch, slp_sec]))
        values = [str(ch), int(deg_old), int(deg_new), int(pitch), float(slp_sec)]
        return {key: val for key, val in zip(self.msg, values)}

    def __smooth_move(self):
        pulses, pitches = self.deg_to_pulse()
        if pulses[0] - pulses[1] != 0 or len(pulses) != 0:
            for idx, pulse in enumerate(pulses):
                if idx == len(pulses) - 1:
                    break
                for step in range(pulses[idx], pulses[idx + 1], pitches[idx]):
                    self.PWM.set_pwm(self.CH_NUM, 0, step)
                    rospy.sleep(self.msg['sleep_sec'])

    def __quick_move(self):
        pulses, pitches = self.deg_to_pulse()
        pulse = pulses[-1] - self.msg['pitch']
        Log.intervally(self.CH, 20, '[{}] node_name pulse {}'.format(self.CH, pulse, ))
        self.PWM.set_pwm(self.CH, 0, pulse)

    def deg_to_pulse(self, params=None):
        self.msg = self._update_msg(params)
        pul_old, pul_new, remainder = self._calc_divide_num()
        pulses, pitches = self._calc_pulse_pith_set(pul_old, pul_new, remainder)
        Log.intervally(self.CH, 30, '{} pulses {}'.format(self.CH_NUM, pulses))
        Log.intervally(self.CH, 30, '{} pitches {}'.format(self.CH_NUM, pitches))
        return pulses, pitches

    def _update_msg(self, params):
        if params != None:
            for key, param in zip(['deg_old', 'deg_new', 'pitch'], params):
                self.msg[key] = param
        return self.msg

    def _calc_divide_num(self, ):
        _pul_old = int(self.PULSES['0_DEG'] + (self.msg['deg_old'] * self.PULSE_PER_DEG))
        _pul_new = int(self.PULSES['0_DEG'] + (self.msg['deg_new'] * self.PULSE_PER_DEG))
        div_num = int((_pul_new - _pul_old) / self.PULSE_PER_DEG)
        diff = int((_pul_new - _pul_old))
        remainder = 0 if div_num == 0 else diff % div_num

        pul_old = _pul_old + remainder + self.msg['pitch']
        pul_new = _pul_new + self.msg['pitch']
        return pul_old, pul_new, remainder

    def _calc_pulse_pith_set(self, pul_old, pul_new, remainder):
        equally_div_num = (pul_new - pul_old - remainder) // 6
        pul_3 = pul_new - equally_div_num  # 275 _ 250
        pul_2 = pul_3 - equally_div_num  # 250 _ 200
        pul_1 = pul_2 - equally_div_num * 2  # 225 _ 275
        pulses = [pul_old, pul_1, pul_2, pul_3, pul_new]
        pitch = self.msg['pitch'] if equally_div_num >= 0 else -self.msg['pitch']
        pitches = [pitch // 4, pitch // 4, pitch // 2, pitch // 2]
        return pulses, pitches


if __name__ == "__main__":
    node = Node()
    node.loop()
