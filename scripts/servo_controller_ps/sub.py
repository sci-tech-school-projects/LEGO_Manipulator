#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import os, time, sys
import Adafruit_PCA9685
import logging
import numpy as np
from scipy.stats import norm
from log_manager import Log_Manager

Log = Log_Manager()


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
        self._get_channel_number()
        if self.ch != None or self.ch != 'cher':
            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(60)

    def _get_channel_number(self):
        try:
            _ch = sys.argv[1]
            self.ch = (_ch[-2:len(_ch)])
            self.node_name = 'ch' + self.ch
            print('[{}] Init ch {}'.format(self.node_name, self.ch))
        except IndexError:
            self.ch = None
            self.node_name = None

    def Loop(self):
        rospy.init_node('listener' + self.ch, anonymous=True)
        rospy.Subscriber('ch' + self.ch, String, self.Call_service)
        rospy.spin()

    def Call_service(self, data):
        def __parse_message(self, message):
            [node_name, def_from, deg_to, pitch, sleep_sec] = message.split(',')
            return node_name, int(def_from), int(deg_to), int(pitch), float(sleep_sec)

        def __smooth_move(self, ch, deg_from, deg_to, pitch, sleep_sec):
            ch_num = int(ch)
            pulses, pitchs = self.Deg_To_Pulse(deg_from, deg_to, pitch)
            if pulses[0] - pulses[1] != 0 or len(pulses) != 0:
                for i, pulse in enumerate(pulses):
                    if i == len(pulses) - 1:
                        break
                    for j in range(pulses[i], pulses[i + 1], pitchs[i]):
                        self.pwm.set_pwm(ch_num, 0, j)
                        rospy.sleep(sleep_sec)

        def __quick_move(self, ch_int, deg_from, deg_to, pitch):
            pulses, pitchs = self.Deg_To_Pulse(deg_from, deg_to, pitch)
            pulse = pulses[-1] - pitch
            Log.intervally(self.node_name, 20,
                           '[{}] node_name pulse {} {}'.format(self.node_name, ch_int, pulse, ))
            self.pwm.set_pwm(ch_int, 0, pulse)

        try:
            node_name_in_msg, def_from, deg_to, pitch, sleep_sec = __parse_message(self, data.data)
            Log.intervally(self.node_name, 50, '[{}] parse_message message {}'.format(self.node_name, data.data))
            Log.intervally(self.node_name, 50,
                           '[{}] {} {} {} {} '.format(self.node_name, def_from, deg_to, pitch, sleep_sec))
            if self.node_name == node_name_in_msg:
                # __smooth_move(self, int(self.ch), def_from, deg_to, pitch, sleep_sec)
                if self.node_name != 'ch00':
                    __smooth_move(self, int(self.ch), def_from, deg_to, pitch, sleep_sec)
                else:
                    __quick_move(self, int(self.ch), def_from, deg_to, pitch)
                    # __smooth_move(self, int(self.ch), def_from, deg_to, pitch, sleep_sec)
        except rospy.ServiceException as  e:
            print("Service call failed: %s" % e)

    def Deg_To_Pulse(self, deg_from, deg_to, pitch):
        def calc_remainder(self, _pul_to, _pul_from, divide_num):
            remainder = 0 if divide_num == 0 else int((_pul_to - _pul_from) % divide_num)
            return remainder

        def culc_pulse_pith_set(pul_to, pul_from, remainder, pitch):
            if (pul_to - pul_from) >= 0:
                froms, pitchs = __calc_smooth_pulse(pul_to, pul_from, remainder, pitch)
            else:
                pitch = -pitch
                froms, pitchs = __calc_smooth_pulse(pul_from, pul_to, remainder, pitch)
            return froms, pitchs

        def __calc_smooth_pulse(smaller, larger, remainder, pitch):
            quarter = (larger - smaller) // 4
            current_pulse = smaller
            _1st_quarter_pulse = current_pulse + quarter
            _2nd_quarter_pulse = _1st_quarter_pulse + quarter
            _3rd_quarter_pulse = _2nd_quarter_pulse + quarter
            _4th_quarter_pulse = _3rd_quarter_pulse + quarter + remainder
            froms = [current_pulse, _1st_quarter_pulse, _2nd_quarter_pulse, _3rd_quarter_pulse, _4th_quarter_pulse]
            pitchs = [pitch // 4, pitch // 2, pitch // 2, pitch // 4]
            return froms, pitchs

        _pul_from = self.pulse['0_deg'] + (float(deg_from) * self._1_deg)
        _pul_to = self.pulse['0_deg'] + (float(deg_to) * self._1_deg)
        diff = _pul_to - _pul_from
        divide_num = diff / self._1_deg
        remainder = calc_remainder(self, _pul_to, _pul_from, divide_num)
        Log.intervally(self.node_name, 40, "{} {} {}".format(type(_pul_from), type(remainder), type(pitch)))
        pul_from = int(_pul_from) + remainder + pitch
        pul_to = int(_pul_to) + pitch
        pulses, pitchs = culc_pulse_pith_set(pul_to, pul_from, remainder, pitch)
        return pulses, pitchs


if __name__ == "__main__":
    node = Node()
    node.Loop()
