#!/usr/bin/python3
import logging
import time
from datetime import datetime

logger = logging.getLogger('Log_Manager')
logger.setLevel(40)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Log_Manager():
    # ch = 'ch00'
    chs_to_log = ['pub']
    interval = 0
    buffer_time = 100

    def __init__(self):
        self.current_time = datetime.now()
        self.last_time = self.current_time

    def intervally(self, ch, level, content):
        self.current_time = datetime.now()
        diff = self.current_time - self.last_time
        if ch in self.chs_to_log:
            if diff.microseconds >= self.interval:
                logger.log(level, content)
                # if diff.microseconds >= self.interval + self.buffer_time:
                self.last_time = self.current_time

    # @property
    # def interval(self):
    #     return self.interval
    #
    # @interval.setter
    # def interval(self, interval):
    #     self.interval = interval
    #
    # @property
    # def ch(self):
    #     return self.ch
    #
    # @ch.setter
    # def ch(self, ch):
    #     self.ch = ch


if __name__ == '__main__':
    lm = Log_Manager()
