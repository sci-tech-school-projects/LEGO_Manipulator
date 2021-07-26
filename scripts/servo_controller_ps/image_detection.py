import darknet
import cv2
import argparse
import os, sys, math, glob, re, shutil
import numpy as np
from datetime import datetime


class Image_Detection():
    args = None
    _cap = None
    FRAME_PX = {'1/1': {'w': 0, 'h': 0}, '1/2': {'w': 0, 'h': 0}, '1/4': {'w': 0, 'h': 0}, }
    prev_img_path = None

    @property
    def cap(self):
        return self._cap

    @property
    def is_darknet_run(self):
        return True

    def __init__(self):
        self.args = self._parser()
        self.check_arguments_errors(self.args)
        self._cap = cv2.VideoCapture(0)

    def _parser(self):
        parser = argparse.ArgumentParser(description="YOLO Object Detection")
        parser.add_argument("--batch_size", default=1, type=int,
                            help="number of images to be processed at the same time")
        parser.add_argument("--weights", default="yolov4.weights",
                            help="yolo weights path")
        parser.add_argument("--dont_show", action='store_true',
                            help="windown inference display. For headless systems")
        parser.add_argument("--ext_output", action='store_true',
                            help="display bbox coordinates of detected objects")
        parser.add_argument("--save_labels", action='store_true',
                            help="save detections bbox for each image in yolo format")
        parser.add_argument("--config_file", default="./cfg/yolov4.cfg",
                            help="path to config file")
        parser.add_argument("--data_file", default="./cfg/coco.data",
                            help="path to data file")
        parser.add_argument("--thresh", type=float, default=.25,
                            help="remove detections with lower confidence")
        return parser.parse_args()

    def load(self, args):
        network, class_names, class_colors = darknet.load_network(
            args.config_file,
            args.data_file,
            args.weights,
            batch_size=args.batch_size
        )
        return network, class_names, class_colors

    def main(self):
        """
        :return: img, detections, FRAME_PX['1/1']
        """
        self._remove_img()
        network, class_names, class_colors = self.load(self.args)
        img, img_path = self._write_img(self._cap)
        img, detections, self.FRAME_PX['1/1'] = self._image_detection(
            img_path, network, class_names, class_colors, self.args.thresh
        )
        return img, detections, self.FRAME_PX['1/1']

    def _remove_img(self):
        if os.path.exists(self.prev_img_path):
            os.remove(self.prev_img_path)
            self.prev_img_path = None

    def _write_img(self, cap):
        ret, img = cap.read()
        h, w, c = np.shape(img)
        crop_size = int((w - h) // 2)
        img = img[:, crop_size:(w - crop_size), :]
        dir = os.path.join(os.environ['HOME'], 'darknet/temp')
        img_name = datetime.now().strftime('%Y%m%d_%H%M%H') + '.jpg'
        img_path = os.path.join(dir, img_name)
        cv2.imwrite(img_path, img)
        self.prev_img_path = img_path
        return img, img_path

    def _image_detection(self, image_path, network, class_names, class_colors, thresh):
        # Darknet doesn't accept numpy images.
        # Create one with image we reuse for each detect
        w = darknet.network_width(network)
        h = darknet.network_height(network)
        darknet_image = darknet.make_image(w, h, 3)

        image = cv2.imread(image_path)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (w, h), interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
        darknet.free_image(darknet_image)
        image = darknet.draw_boxes(detections, image_resized, class_colors)
        frame_wh = {'1/1': {'w': w, 'h': h}}
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB), detections, frame_wh

    def check_arguments_errors(self, args):
        assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
        if not os.path.exists(args.config_file):
            raise (ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
        if not os.path.exists(args.weights):
            raise (ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
        if not os.path.exists(args.data_file):
            raise (ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
