# import numpy as np
from config import *


def round_3(val: float) -> float:
    return round(val, 3)


def cos_theorem(w: float, h: float, d: float) -> float:
    if 0.0 in [w, h]:
        return 0.0
    else:
        return round_3(((w ** 2.0) + (h ** 2.0) - (d ** 2.0)) / (2.0 * w * h))


def sqr_theorem(x: float, y: float) -> float:
    return round_3(((x ** 2.0) + (y ** 2.0)) ** (1 / 2))


def zoomed_25(length: int) -> int:
    return int(length * ZOOM_25)


def zoomed_50(length: int) -> int:
    return int(length * ZOOM_50)


def calc_mid_px(start, end):
    return int((end - start) / 2) + start

# def get_frame_whs(img):
#     h, w, c = np.shape(img)
#     frame_px = {'ORIGINAL': {'w': w, 'h': h},
#                 'RESIZED': {'w': int(w // ZOOM_RATE), 'h': int(h / ZOOM_RATE)}, }
#     return frame_px
