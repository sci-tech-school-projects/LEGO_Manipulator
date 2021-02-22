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
# import cv2
import Adafruit_PCA9685

logger = logging.getLogger('LoggingTest')
logger.setLevel(30)
sh = logging.StreamHandler()
logger.addHandler(sh)


