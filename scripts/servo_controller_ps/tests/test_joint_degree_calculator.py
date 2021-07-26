import unittest
from joint_degree_calculator import Joint_Degree_Calculator


class Test_Joint_Degree_Calculator(unittest.TestCase):
    xyz = [205.0, 0.0, -10.0]
    node_dists = {'x': 205.0, 'y': 0.0, 'z': -10.0, '_2x': 154.0, '_2z': 154.324, '_2yz': 154.324}
    node_degs = {'ch00': 98, 'ch01': 93, 'ch02': 46, 'ch03': 87, 'ch04': 46, 'ch05': 93, 'ch06': 85}

    def setUp(self) -> None:
        self.jdc = Joint_Degree_Calculator()

    def test_calc_node_dists(self):
        self.assertEqual(self.jdc.calc_node_dists(self.xyz), self.node_dists)

    def test_calc_node_degs(self):
        self.assertEqual(self.jdc.calc_node_degs(self.node_dists), self.node_degs)


if __name__ == '__main__':
    unittest.main()
