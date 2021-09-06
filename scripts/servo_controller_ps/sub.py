#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import sys
import Adafruit_PCA9685
from common import Common as cmn
from log_manager import LogManager
from config import *

Log = LogManager()


class Node:
    PULSES = {'0_DEG': 121.0, '90_DEG': 355.0, '180_DEG': 621.0, }
    # 2.722...
    PULSE_PER_DEG: float = cmn.round_3((PULSES['180_DEG'] - PULSES['90_DEG']) / 90.0)
    CH_NUM: int = None
    CH_STR: str = None
    CH: str = None
    PWM = None
    PWM_SUB = None
    msg = {'ch': None, 'deg_old': None, 'deg_new': None, 'pitch': None, 'sleep_sec': None}

    def __init__(self, ch=None):
        if ch is not None:
            self._get_channel_number(ch)
            if type(self.CH_NUM) == int:
                self.PWM = Adafruit_PCA9685.PCA9685()
                self.PWM.set_pwm_freq(60)
                # if self.CH == 'ch00':
                #     self.PWM_SUB = Adafruit_PCA9685.PCA9685()
                #     self.PWM_SUB.set_pwm_freq(60)
        else:
            pass

    def _get_channel_number(self, ch):
        self.CH_STR = ch[-2::]
        self.CH = 'ch' + self.CH_STR
        self.CH_NUM = int(self.CH_STR)
        print('[{}] Init ch {}'.format(self.CH, self.CH_NUM))

    def loop(self):
        node_name = 'listener' + self.CH_STR
        print('***** init {}'.format(node_name))
        rospy.init_node(node_name, anonymous=True)
        rospy.Subscriber(self.CH, String, self._call_service)
        rospy.spin()

    def _call_service(self, data):
        try:
            self.msg = self.__parse_message(data.data)
            if self.CH == self.msg['ch']:
                self.__smooth_move()
                # if self.CH in ['ch00']:
                #     self.__grip_move()
                # else:
                #     self.__smooth_move()
            else:
                raise Exception("self.CH != self.msg['ch'] aborted.")
        except rospy.ServiceException as e:
            raise Exception("Service call failed: %s" % e)

    def __parse_message(self, message: str) -> {str, int, int, int, float}:
        ch, deg_old, deg_new, pitch, slp_sec = message.split(',')
        Log.only(self.CH, 30, '[{}] parse_message {}'.format(ch, message))
        Log.only(self.CH, 30, '[{}] {} '.format(ch, [deg_old, deg_new, pitch, slp_sec]))
        values = [str(ch), int(deg_old), int(deg_new), int(pitch), float(slp_sec)]
        return {key: val for key, val in zip(self.msg.keys(), values)}

    def __smooth_move(self):
        pulses, pitches = self.___deg_to_pulse()
        if pulses[0] - pulses[1] != 0 or len(pulses) != 0:
            for idx, pulse in enumerate(pulses):
                if idx == len(pulses) - 1:
                    break
                for step in range(pulses[idx], pulses[idx + 1], pitches[idx]):
                    self.PWM.set_pwm(self.CH_NUM, 0, step)
                    rospy.sleep(self.msg['sleep_sec'])

    def __grip_move(self):
        if self.msg['deg_new'] == GRIP_DEGS['ch00']['open']:
            self.PWM.set_pwm(self.CH_NUM, 0, GRIP_DEGS['ch00']['open'])
            self.PWM_SUB.set_pwm(6, 0, GRIP_DEGS['ch06']['open'])
        elif self.msg['deg_new'] == GRIP_DEGS['ch00']['close']:
            self.PWM.set_pwm(self.CH_NUM, 0, GRIP_DEGS['ch00']['close'])
            self.PWM_SUB.set_pwm(6, 0, GRIP_DEGS['ch06']['close'])
        else:
            raise Exception("Did not match self.msg['deg_new'] == GRIP_STATES['ch00']['open']")

    def ___deg_to_pulse(self, params=None):
        self.msg = self._update_msg(params)
        pul_old, pul_new, remainder = self.____calc_divide_num()
        pulses, pitches = self.____calc_pulse_pith_set(pul_old, pul_new, remainder)
        Log.only(self.CH, 30, '{} pulses {}'.format(self.CH_NUM, pulses))
        Log.only(self.CH, 30, '{} pitches {}'.format(self.CH_NUM, pitches))
        return pulses, pitches

    def _update_msg(self, params):
        if params is not None:
            for key, param in zip(['deg_old', 'deg_new', 'pitch'], params):
                self.msg[key] = param
        return self.msg

    def ____calc_divide_num(self, ):
        _pul_old = int(self.PULSES['0_DEG'] + (self.msg['deg_old'] * self.PULSE_PER_DEG))
        _pul_new = int(self.PULSES['0_DEG'] + (self.msg['deg_new'] * self.PULSE_PER_DEG))
        div_num = int((_pul_new - _pul_old) / self.PULSE_PER_DEG)
        diff = int((_pul_new - _pul_old))
        remainder = 0 if div_num == 0 else diff % div_num

        pul_old = _pul_old + remainder + self.msg['pitch']
        pul_new = _pul_new + self.msg['pitch']
        return pul_old, pul_new, remainder

    def ____calc_pulse_pith_set(self, pul_old, pul_new, remainder):
        equally_div_num = (pul_new - pul_old - remainder) // 6
        pul_3 = pul_new - equally_div_num  # 275 _ 250
        pul_2 = pul_3 - equally_div_num  # 250 _ 200
        pul_1 = pul_2 - equally_div_num * 2  # 225 _ 275
        pulses = [pul_old, pul_1, pul_2, pul_3, pul_new]
        pitch = self.msg['pitch'] if equally_div_num >= 0 else -self.msg['pitch']
        pitches = [pitch // 4, pitch // 4, pitch // 2, pitch // 2]
        return pulses, pitches

    def calc_sleep_time(self, current_degs, next_degs, is_down):
        deg_old, deg_new = self._get_max_pulse_diff(current_degs, next_degs)
        pulses, pitches = self.___deg_to_pulse([deg_old, deg_new, SERVO['I_F']['MOVE_PITCH']])
        times = 0.0
        if pulses[0] != pulses[1]:
            for i, pulse in enumerate(pulses):
                if i == len(pulses) - 1:
                    break
                for j in range(pulses[i], pulses[i + 1], pitches[i]):
                    times += 1.0
        sleep_time = SERVO['I_F']['STEP_SEC'] * times if not is_down else SERVO['I_F']['STEP_SEC_SLOW'] * times
        return sleep_time

    def _get_max_pulse_diff(self, current_degs, next_degs) -> [int, int]:
        diffs = []
        for ch in CHS:
            diff = abs(current_degs[ch] - next_degs[ch])
            diffs.append(diff)
        max_diff_key = 'ch0' + str(diffs.index(max(diffs)))
        deg_old, deg_new = current_degs[max_diff_key], next_degs[max_diff_key]
        Log.only(self.CH, 10, 'deg old {} new {}'.format(deg_old, deg_new))
        return [deg_old, deg_new]


if __name__ == "__main__":
    _ch = sys.argv[1]
    _, _ch = _ch.split('__name:=')
    if int(_ch[-2::]) in list(range(7)):
        ch = 'ch' + _ch[-2::]
        if ch in CHS:
            node = Node(ch)
            node.loop()
    else:
        Log.any(_ch, 20, 'Not init node for {}'.format(_ch))
