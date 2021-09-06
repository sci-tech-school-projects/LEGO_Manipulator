import unittest
from key_control import Servo_Controller

sc = Servo_Controller()


class Test_Servo_Controller(unittest.TestCase):
    test_ch = 'ch04'
    arm_state = 'LIFTING_UP'
    current_degs = {'ch00': 98, 'ch01': 90, 'ch02': 41, 'ch03': 89, 'ch04': 48, 'ch05': 90, 'ch06': 85}
    next_degs = {'ch00': 98, 'ch01': 90, 'ch02': 46, 'ch03': 87, 'ch04': 46, 'ch05': 90, 'ch06': 85}
    xyz = [205.0, 0.0, -10.0]

    # def setUp(self) -> None:

    def test_compile_message(self):
        self.assertEqual(sc._compile_message(self.test_ch), 'ch04,48,46,4,0.01')

    def test__get_next_arm_state(self):
        self.assertEqual(sc.__get_next_arm_state(self.arm_state), 'REACHING')

    def test___get_max_pulse_diff(self):
        degs = [self.current_degs, self.next_degs]
        self.assertEqual(sc._get_max_pulse_diff(degs), [41, 46])

    # def test__catch_key(self):
    #     self.assertEqual(sc.__catch_key(self.xyz),)


if __name__ == '__main__':
    unittest.main()
