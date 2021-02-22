#!/usr/bin/env python3
import os, time
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import Adafruit_PCA9685
import logging

logger = logging.getLogger('LoggingTest')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

pulse = {
    '0_deg': 120,
    '180_deg': 610,
}

# 2.722...
_1_deg = (pulse['180_deg'] - pulse['0_deg']) / 180.0
_90 = int(pulse['0_deg'] + (_1_deg * 90))
published_nodes = {'ch00': False, 'ch01': False, 'ch02': False, 'ch03': False, 'ch04': False, 'ch05': False, }


def main():
    for node in [0, 1, 2, 3, 4, 5]:
        # pwm.set_pwm(node, 0, _90)
        pwm.set_pwm(node, 0, _90)


if __name__ == "__main__":
    main()
