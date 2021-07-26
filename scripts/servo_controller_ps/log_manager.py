#!/usr/bin/python3
import logging
import time
from datetime import datetime

logger = logging.getLogger('Log_Manager')
logger.setLevel(10)
sh = logging.StreamHandler()
logger.addHandler(sh)


class Log_Manager():
    # ch = 'ch00'
    TARGET_CHS = ['jdc', 'pub']
    INTERVAL = 0
    BUFFER_TIME = 100

    def __init__(self):
        self.current_time = datetime.now()
        self.last_time = self.current_time

    def intervally(self, ch, level, content):
        self.current_time = datetime.now()
        if ch in self.TARGET_CHS:
            logger.log(level, ch + ' : ' + content)
            self.last_time = self.current_time


if __name__ == '__main__':
    lm = Log_Manager()
