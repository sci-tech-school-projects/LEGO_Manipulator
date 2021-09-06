#!/usr/bin/python3
import logging
from datetime import datetime

logger = logging.getLogger('Log_Manager')
logger.setLevel(20)
sh = logging.StreamHandler()
logger.addHandler(sh)


class LogManager:
    TARGET_CHS = ['pub', 'occ', 'imd']
    INTERVAL = 0
    BUFFER_TIME = 100

    def __init__(self):
        self.current_time = datetime.now()
        self.last_time = self.current_time

    def only(self, ch, level, content):
        self.current_time = datetime.now()
        if ch in self.TARGET_CHS:
            logger.log(level, ch + ' : ' + content)
            self.last_time = self.current_time

    def any(self, ch, level, content):
        logger.log(level, ch + ' : ' + content)


if __name__ == '__main__':
    lm = LogManager()
