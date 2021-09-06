import numpy as np
from config import *


class Common:
    @classmethod
    def round_3(cls, val: float) -> float:
        return round(val, 3)

    @classmethod
    def cos_theorem(cls, w: float, h: float, d: float) -> float:
        if 0.0 in [w, h]:
            return 0.0
        else:
            return cls.round_3(((w ** 2.0) + (h ** 2.0) - (d ** 2.0)) / (2.0 * w * h))

    @classmethod
    def sqr_theorem(cls, x: float, y: float) -> float:
        return cls.round_3(((x ** 2.0) + (y ** 2.0)) ** (1 / 2))

    @classmethod
    def zoomed_25(cls, length: int) -> int:
        return int(length * ZOOM_25)

    @classmethod
    def zoomed_50(cls, length: int) -> int:
        return int(length * ZOOM_50)

    @classmethod
    def calc_mid_px(cls, start, end):
        return int((end - start) / 2) + start

    @classmethod
    def get_trimmed_img(cls, img):
        h, w, c = np.shape(img)
        _quarter = int((w - h) // 4)
        _start = _quarter * 2
        _end = w - (_quarter * 2)
        return img[:, _start:_end, :]

