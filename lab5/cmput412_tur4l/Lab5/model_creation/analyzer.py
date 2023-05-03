import rosbag
import cv2
import numpy as np
import math
import matplotlib
from matplotlib import pyplot as plt
import util
import digit_recognition
from digit_recognition import plt_showim
import tensorflow as tf
import tag_contour


# matplotlib.use('Agg')


HOST_NAME = 'csc22913'


TIME_CUTOFF_MIN = 1675038119 #526362452
TIME_CUTOFF_MAX = math.inf


def mask_to_grayscale(im, min_value, max_value):
    im = (max_value - np.minimum(max_value, np.maximum(min_value, im))) * \
           (255.9 / (max_value - min_value))
    im = np.array(im, dtype=np.uint8)
    return im


def mask_by_hsv_bounds(im_hsv):
    # TAG_MASK = ((90, 0, 30), (130, 100, 60))
    TAG_MASK = ((100, 130, 80), (107, 220, 140))
    im = cv2.inRange(im_hsv, TAG_MASK[0], TAG_MASK[1])
    return im


def compute_dim(im_hsv, reference_color, reference_std):
    diff_im = np.abs((im_hsv - reference_color[np.newaxis, np.newaxis, :]) / reference_std[np.newaxis, np.newaxis, :])
    diff_im = np.sum(diff_im, axis=2)
    return diff_im


def main():
    WIDTH = 96

    recognizer = digit_recognition.Recognizer()

    LOAD_MODEL = True
    if LOAD_MODEL:
        recognizer.load_model()
    else:
        recognizer.train_data()
        recognizer.save_model(None)

    recognizer.test_data()
    recognizer.test_png()
    image_count = 0

    for topic, msg, t in bag.read_messages(topics=[f'/{HOST_NAME}/camera_node/image/compressed']):
        if t.secs < TIME_CUTOFF_MIN:
            continue
        elif t.secs > TIME_CUTOFF_MAX:
            break

        compressed_image = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)
        # im = cv2.pyrDown(im)
        # im = cv2.flip(cv2.cvtColor(im, cv2.COLOR_BGR2RGB), 1)
        image_count += 1

        if image_count % 2 == 0:
            selectim = np.copy(im)
            pltim = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

            ihom2d_pts = util.selectManyPoints(selectim)
            npts = ihom2d_pts.shape[0]
            for i in range(npts):
                ihom_pt = ihom2d_pts[i, :]
                cx, cy = ihom_pt[0].item(), -ihom_pt[1].item()
                xmin, ymin = int(cx - WIDTH * .5), int(cy - WIDTH * .5)
                xmax, ymax = xmin + WIDTH, ymin + WIDTH

                crop = im[ymin:ymax, xmin:xmax]
                if crop.size == 0:
                    continue
                hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
                REFERENCE_COLOR_BLUE = np.array((103, 175, 135), dtype=np.float32)
                BLUE_STD = np.array((5, 40, 60), dtype=np.float32)
                dim = compute_dim(hsv, REFERENCE_COLOR_BLUE, BLUE_STD)
                dim = np.array(dim < 3, dtype=np.uint8)  # crop = mask_to_grayscale(dim, 3., 3.8)

                # plt_showim(dim)
                # plt_showim(hsv)

                # first findContours call is only used to determine the area of interest
                contours, hierarchy = cv2.findContours(dim, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                square_idx = tag_contour.find_largest_contour_idx(contours, hierarchy, 0)
                if square_idx == -1:
                    print(f'no contour detected in the input!')
                    continue
                bound_x, bound_y, bound_w, bound_h = cv2.boundingRect(contours[square_idx])
                crop_hsv = hsv[bound_y:bound_y + bound_h, bound_x:bound_x + bound_w]

                # second findContours call finds the digit inside the given cropped image
                # note this time we add a penalty to the sides of the images,
                # as the color on sides of the images are unreliable
                REFERENCE_COLOR_BLACK = np.array((120, 40, 50), dtype=np.float32)
                BLACK_STD = np.array((30, 60, 40), dtype=np.float32)
                dim = compute_dim(crop_hsv, REFERENCE_COLOR_BLACK, BLACK_STD)

                dim_w, dim_h = dim.shape[1], dim.shape[0]
                idx_y, idx_x = np.indices(dim.shape)
                distance_sqr = (idx_x / dim_w - .5) ** 2 * 2 + (idx_y / dim_h - .5) ** 2  # penalize x direction more
                digit_im = np.array(dim + (distance_sqr > .21) * 3. < 3, dtype=np.uint8)
                digit_im = cv2.resize(digit_im, (28, 28), cv2.INTER_AREA) * 255
                # plt_showim(digit_im)

                digit = int(recognizer.detect_digit(digit_im))
                # impath = f'../plots/im_{image_count}_{i}_{digit}.png'
                # cv2.imwrite(impath, digit_im)
                print(f'recognized digit:{digit}')
                pltim = cv2.putText(pltim, str(digit), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
                pltim = cv2.polylines(pltim, np.array(
                    (((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)), ), dtype=np.int32), True, (255, 0, 0))
            # if npts > 0:
            #     plt_showim(pltim)
            #     # plt.savefig(f'./images/test{image_count}.png')

    bag.close()


if __name__ == '__main__':
    main()