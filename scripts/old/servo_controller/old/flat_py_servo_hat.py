"""
https://projects-raspberry.com/pi-servo-hat-hookup-guide/
Channel #	Start Address	Stop Address
Ch 0	0x06	0x08
Ch 1	0x0A	0x0C
Ch 2	0x0E	0x10
Ch 3	0x12	0x14
Ch 4	0x16	0x18
Ch 5	0x1A	0x1C
Ch 6	0x1E	0x20
Ch 7	0x22	0x24
Ch 8	0x26	0x28
Ch 9	0x2A	0x2C
Ch 10	0x2E	0x30
Ch 11	0x32	0x34
Ch 12	0x36	0x38
Ch 13	0x3A	0x3C
Ch 14	0x3E	0x40
Ch 15	0x42	0x44
"""

import smbus, time

bus = smbus.SMBus(1)
addr = 0x40

from argparse import ArgumentParser

## Running this program will move the servo to 0, 45, and 90 degrees with 5 second pauses in between with a 50 Hz PWM signal.

bus.write_byte_data(addr, 0, 0x20)  # enable word writes
time.sleep(.25)
bus.write_byte_data(addr, 0, 0x10)  # enable Prescale change as noted in the datasheet
time.sleep(.25)  # delay for reset
bus.write_byte_data(addr, 0xfe,
                    0x88)  # changes the Prescale register value for 50 Hz, using the equation in the datasheet (I later adjusted the value to fine tune the signal with an oscilloscope. The original value was 0x79.)
time.sleep(.25)
bus.write_byte_data(addr, 0, 0x20)  # enables word writes

channels = {
    0: {'start': 0x06, 'end': 0x08},
    1: {'start': 0x0A, 'end': 0x0C},
    2: {'start': 0x0E, 'end': 0x10},
    3: {'start': 0x12, 'end': 0x14},
    4: {'start': 0x16, 'end': 0x18},
    5: {'start': 0x1A, 'end': 0x1C},
    6: {'start': 0x1E, 'end': 0x20},
    7: {'start': 0x22, 'end': 0x24},
}

_0 = 80
_180 = 440
_90 = 270
_45 = _0 + int((_90 - _0) / 2)
_135 = _90 + int((_180 - _90) / 2)

# ピンを有効にする
bus.write_word_data(addr, channels[0]['start'], 0)
bus.write_word_data(addr, channels[1]['start'], 0)
bus.write_word_data(addr, channels[2]['start'], 0)
bus.write_word_data(addr, channels[3]['start'], 0)
bus.write_word_data(addr, channels[4]['start'], 0)
bus.write_word_data(addr, channels[5]['start'], 0)
bus.write_word_data(addr, channels[6]['start'], 0)
bus.write_word_data(addr, channels[7]['start'], 0)


bus.write_word_data(addr, channels[1]['end'], _90)
bus.write_word_data(addr, channels[3]['end'], _90)
bus.write_word_data(addr, channels[5]['end'], _90)
bus.write_word_data(addr, channels[7]['end'], _90)
bus.write_word_data(addr, channels[0]['end'], _90)
bus.write_word_data(addr, channels[2]['end'], _90)
bus.write_word_data(addr, channels[4]['end'], _90)
bus.write_word_data(addr, channels[6]['end'], _90)
