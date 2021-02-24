#!/usr/bin/python3
from math import degrees, acos, floor


class Object_Coordinate_Calculator():

    def __init__(self):
        """"""
        # get_convert_scale(img)

    def main(self, img_coordinate, belt_scale_mm):
        belt_coordinate_pixel = self.get_pixeled_belt_coordinate(img_coordinate)
        pixel_per_mm = self.get_convert_scale(belt_coordinate_pixel, belt_scale_mm)
        target_abs_mm = self.pixel_to_mm(belt_coordinate_pixel, pixel_per_mm)
        target_rel_mm = self.from_arm_to_obj_xy_mm(target_abs_mm)
        target_rel_vector = self.from_arm_to_obj_vector(target_rel_mm)
        return target_rel_vector

    def get_pixeled_belt_coordinate(self, img_coordinate=None):
        # belt size in img from camera(number below is assumed). each coordinate means base position of belt.
        belt_coordinate_pixel = {'start': {'x': 30, 'y': 20},
                                 'end': {'x': 450, 'y': 340}}
        if img_coordinate != None:
            x, y, w, h = img_coordinate
            belt_coordinate_pixel = {'start': {'x': x, 'y': y},
                                     'end': {'x': x + w, 'y': y + h}}
        return belt_coordinate_pixel

    def get_convert_scale(self, belt_coordinate_pixel, belt_scale_mm=None):
        # mm scaled belt size

        if belt_scale_mm == None:
            belt_scale_mm = {'w': 400, 'h': 200}
        new_x = (belt_coordinate_pixel['end']['x'] - belt_coordinate_pixel['start']['x']) / belt_scale_mm['w']
        new_y = (belt_coordinate_pixel['end']['y'] - belt_coordinate_pixel['start']['y']) / belt_scale_mm['h']
        pixel_per_mm = {'x': new_x, 'y': new_y}
        return pixel_per_mm

    def pixel_to_mm(self, belt_coordinate_pixel, pixel_per_mm):
        ppm = pixel_per_mm
        target_abs_x_mm = (belt_coordinate_pixel['end']['x'] - belt_coordinate_pixel['start']['x']) * ppm['x']
        target_abs_y_mm = (belt_coordinate_pixel['end']['y'] - belt_coordinate_pixel['start']['y']) * ppm['y']
        target_abs_mm = {'x': target_abs_x_mm, 'y': target_abs_y_mm}
        return target_abs_mm

    def from_arm_to_obj_xy_mm(self, target_abs_mm, center_of_arm_axis_mm=None):
        """

        """
        if center_of_arm_axis_mm == None:
            center_of_arm_axis_mm = {'x': 300, 'y': -50}
        target_rel_x = target_abs_mm['x'] - center_of_arm_axis_mm['x']
        target_rel_y = target_abs_mm['y'] - center_of_arm_axis_mm['y']
        target_rel_mm = {'x': target_rel_x, 'y': target_rel_y}
        return target_rel_mm

    def from_arm_to_obj_vector(self, target_rel_mm):
        x = target_rel_mm['x']
        y = target_rel_mm['y']
        l = (x ** 2 + y ** 2) ** (1 / 2)
        cos_theta = (x ** 2 + l ** 2 - y ** 2) / (2 * x * l)
        theta_arm_to_obj = floor(degrees(acos(cos_theta)))
        target_rel_vector = {'l': l, 'theta': theta_arm_to_obj}
        return target_rel_vector


if __name__ == '__main__':
    img_coordinate = None
    belt_scale_mm = None
    object_coordinate_calculator = Object_Coordinate_Calculator()
    target_rel_vector = object_coordinate_calculator.main(img_coordinate, belt_scale_mm)
