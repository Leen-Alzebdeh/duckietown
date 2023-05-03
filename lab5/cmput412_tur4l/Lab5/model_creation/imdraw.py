import numpy as np
import cv2


def hom2d_to_int_imcoord(hom_pt):
    w = hom_pt[2]
    x = (hom_pt[0] / w).item()
    y = -(hom_pt[1] / w).item()
    return int(x), int(y)


def draw_dot(im, imx, imy, color):
    color = np.array(color, dtype=np.uint8)
    imshape = np.shape(im)
    width = imshape[1]
    height = imshape[0]
    imx, imy = int(imx), int(imy)
    xmin = min(width, max(0, imx - 2))
    xmax = min(width, max(0, imx + 2))
    ymin = min(height, max(0, imy - 2))
    ymax = min(height, max(0, imy + 2))
    im[ymin:ymax, xmin:xmax, :] = color[None, None, :]


def draw_dot_ihom(im, ihom_pt, color):
    imx, imy = tuple(ihom_pt)
    draw_dot(im, imx, -imy, color)


def draw_dot_hom(im, hom_pt, color):
    imx, imy, w = tuple(hom_pt)
    if w == 0:
        # ideal point
        return
    else:
        draw_dot(im, imx / w, -imy / w, color)


def draw_line(im, hom_pt1, hom_pt2, color):
    cv2.line(im, hom2d_to_int_imcoord(hom_pt1), hom2d_to_int_imcoord(hom_pt2), color, 1)