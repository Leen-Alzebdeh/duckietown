import numpy as np
import cv2


def compute_dim(im_hsv, reference_color, reference_std):
    """
    compute the difference image between the given hsv and the reference color
    """
    diff_im = np.abs((im_hsv - reference_color[np.newaxis, np.newaxis, :]) / reference_std[np.newaxis, np.newaxis, :])
    diff_im = np.sum(diff_im, axis=2)
    return diff_im


def find_largest_contour_idx(contours, hierarchy, first_child_idx):
    """
    first_child_idx specifies the first contour in the level we want to find the largest contour of
    return -1 if no contour is in the list or the given first child is not in the list
    """
    if hierarchy is None or first_child_idx >= hierarchy.shape[1]:
        return -1

    i = first_child_idx
    largest_idx = -1
    largest_area = -1
    while i != -1:
        area = cv2.contourArea(contours[i])
        if area > largest_area:
            largest_idx = i
            largest_area = area

        # update i
        cntinfo = hierarchy[0][i]  # (next, prev, first_child, parent)
        i = cntinfo[0]

    return largest_idx


def get_cropped_digit(im, center):
    """
    center is the estimated center of the digit (doesn't have to be accurate)
    """
    WIDTH = 96
    cx, cy = center[0], center[1]
    xmin, ymin = int(cx - WIDTH * .5), int(cy - WIDTH * .5)
    xmax, ymax = xmin + WIDTH, ymin + WIDTH

    crop = im[ymin:ymax, xmin:xmax]
    if crop.size == 0:
        return None, None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    REFERENCE_COLOR_BLUE = np.array((103, 175, 135), dtype=np.float32)
    BLUE_STD = np.array((5, 40, 60), dtype=np.float32)
    dim = compute_dim(hsv, REFERENCE_COLOR_BLUE, BLUE_STD)
    dim = np.array(dim < 3, dtype=np.uint8)  # crop = mask_to_grayscale(dim, 3., 3.8)

    # first findContours call is only used to determine the area of interest
    contours, hierarchy = cv2.findContours(dim, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    square_idx = find_largest_contour_idx(contours, hierarchy, 0)
    if square_idx == -1:
        print(f'no contour detected in the input!')
        return None, None
    rect = cv2.boundingRect(contours[square_idx])
    bound_x, bound_y, bound_w, bound_h = rect
    crop_hsv = hsv[bound_y:bound_y + bound_h, bound_x:bound_x + bound_w]

    # second findContours call finds the digit inside the given cropped image
    # note this time we add a penalty to the sides of the images,
    # as the color on sides of the images are unreliable
    REFERENCE_COLOR_BLACK = np.array((120, 40, 50), dtype=np.float32)
    BLACK_STD = np.array((30, 60, 40), dtype=np.float32)
    dim = compute_dim(crop_hsv, REFERENCE_COLOR_BLACK, BLACK_STD)

    dim_w, dim_h = dim.shape[1], dim.shape[0]
    idx_y, idx_x = np.indices(dim.shape)
    distance_sqr = (idx_x / dim_w - .5) ** 2 + (idx_y / dim_h - .5) ** 2  # penalize x direction more
    digit_im = np.array(dim + (distance_sqr > .21) * 3. < 3, dtype=np.uint8)
    digit_im = cv2.resize(digit_im, (28, 28), cv2.INTER_AREA) * 255
    return (bound_x + xmin, bound_y + ymin, bound_w, bound_h), digit_im
