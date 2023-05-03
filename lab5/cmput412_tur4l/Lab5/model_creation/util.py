
import numpy as np
import cv2
import imdraw


def centerImagePts(ihom2d_pts, im):
    results = np.zeros(ihom2d_pts.shape)
    results[:, 0] = ihom2d_pts[:, 0] - im.shape[1] * 0.5  # center the image coordinates in the x direction
    results[:, 1] = ihom2d_pts[:, 1] + im.shape[0] * 0.5  # y direction
    return results


def selectPoints(im, npt, pt_color=(255, 255, 255)):
    """
    return npt inhomogeneous coordinates of npt 2d points
    the points will be in planar coordinates where positive y is up
    """
    ihom_pts = np.zeros((npt, 2))
    nselect = 0

    def onClick(event, x, y, flags, param):
        nonlocal nselect
        if event == cv2.EVENT_LBUTTONDOWN and nselect < npt:
            ihom_pts[nselect, :] = np.array((x, -y))
            imdraw.draw_dot(im, x, y, pt_color)
            nselect += 1
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', onClick)

    while True:
        key = cv2.waitKey(16)
        if key % 256 == 27 or nselect == npt:
            break
        cv2.imshow('image', im)
    cv2.destroyWindow('image')
    
    return ihom_pts


def selectManyPoints(im, pt_color = (255, 255, 255)):
    pts = []

    def onClick(event, x, y, flags, param):
        nonlocal pts
        if event == cv2.EVENT_LBUTTONDOWN:
            pts.append([x, -y])
            imdraw.draw_dot(im, x, y, pt_color)
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', onClick)

    while True:
        key = cv2.waitKey(16)
        if key % 256 == 27:
            break
        cv2.imshow('image', im)
    cv2.destroyWindow('image')

    ihom_pts = np.array(pts)
    
    return ihom_pts


def waitForClick(cam):
    clicked = False
    def onClick(event, x, y, flags, param):
        nonlocal clicked
        clicked = True
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', onClick)
    
    while True:
        ret, im = cam.read()
        key = cv2.waitKey(16)
        if key % 256 == 27 or clicked:
            break
        cv2.imshow('image', im)
    cv2.destroyWindow('image')