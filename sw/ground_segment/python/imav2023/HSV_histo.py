#!/usr/bin/python
#This program is to read HSV of any point in an image
#You click on any point, the [H, S, V] will be printed out
#2 windows will pop out:
#1st one is initial image, for you to find the mailbox
#2nd one is the "Hue" image, for you to find the value may differ
import cv2
import cv2 as cv
import numpy as np
#from matplotlib import pyplot as plt
import sys

if len(sys.argv) != 2:
    print("Missing file input")
    sys.exit(1)

file_name = sys.argv[1]

hsv_map = np.zeros((180, 256, 3), np.uint8)
h, s = np.indices(hsv_map.shape[:2])
hsv_map[:,:,0] = h
hsv_map[:,:,1] = s
hsv_map[:,:,2] = 255
hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)
#cv2.imshow('hsv_map', hsv_map)

cv2.namedWindow('hist', cv2.WINDOW_NORMAL)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
hist_scale = 10
h_min = 0
h_max = 179
s_min = 0
s_max = 255
updated = True

def set_scale(val):
    global hist_scale
    global updated
    hist_scale = val
    updated = True

def set_h_min(val):
    global h_min
    global updated
    h_min = val
    updated = True

def set_h_max(val):
    global h_max
    global updated
    h_max = val
    updated = True

def set_s_min(val):
    global s_min
    global updated
    s_min = val
    updated = True

def set_s_max(val):
    global s_max
    global updated
    s_max = val
    updated = True

mouse_hsv = None
def getpos(event,x,y,flags,param):
    global mouse_hsv
    global h_min, h_max, s_min, s_max, updated
    if event == cv2.EVENT_LBUTTONDOWN:
        if mouse_hsv is None:
            mouse_hsv = (y, x)
        else:
            h_min = mouse_hsv[0]
            s_min = mouse_hsv[1]
            h_max = y
            s_max = x
            cv2.setTrackbarPos('h_min', 'hist', h_min)
            cv2.setTrackbarPos('h_max', 'hist', h_max)
            cv2.setTrackbarPos('s_min', 'hist', s_min)
            cv2.setTrackbarPos('s_max', 'hist', s_max)
            mouse_hsv = None
            updated = True

cv2.setMouseCallback("hist",getpos)

cv2.createTrackbar('scale', 'hist', hist_scale, 32, set_scale)
cv2.createTrackbar('h_min', 'hist', h_min, 179, set_h_min)
cv2.createTrackbar('h_max', 'hist', h_max, 179, set_h_max)
cv2.createTrackbar('s_min', 'hist', s_min, 255, set_s_min)
cv2.createTrackbar('s_max', 'hist', s_max, 255, set_s_max)

def plot_hist():
    img = cv.imread(file_name)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # can't swap s param
    _s_min = min(s_min, s_max)
    _s_max = max(s_min, s_max)

    # discard dark colors
    #dark = hsv[...,2] < 32
    #hsv[dark] = 0
    mask = None
    if h_min <= h_max:
        lower = np.array([h_min, _s_min, 0])
        upper = np.array([h_max, _s_max, 255])
        mask = cv2.inRange(hsv, lower, upper)
    else: # splitted
        lower1 = np.array([0, _s_min, 0])
        upper1 = np.array([h_max, _s_max, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        lower2 = np.array([h_min, _s_min, 0])
        upper2 = np.array([179, _s_max, 255])
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = mask1 + mask2

    #hsv_masked = cv2.bitwise_and(hsv, hsv, mask=mask)
    img_masked = cv2.bitwise_and(img, img, mask=mask)

    h = cv2.calcHist( [hsv], [0, 1], None, [180, 256], [0, 180, 0, 256] )

    h = np.clip(h*0.005*hist_scale, 0, 1)
    vis = hsv_map*h[:,:,np.newaxis] / 255.0
    if h_min <= h_max:
        cv2.rectangle(vis,(_s_min,h_min),(_s_max,h_max),(0,255,0),1)
    else: # splitted
        cv2.rectangle(vis,(_s_min,0),(_s_max,h_max),(0,255,0),1)
        cv2.rectangle(vis,(_s_min,h_min),(_s_max,179),(0,255,0),1)
    cv2.imshow('hist', vis)
    cv2.imshow('image', img_masked)

    # HSV output
    print("[[{}, {}, 0],[{}, {}, 255]] | {} {} 0 {} {} 255".format(h_min, min(s_min,s_max), h_max, max(s_min,s_max), h_min, min(s_min,s_max), h_max, max(s_min,s_max)))

while True:
    if updated:
        plot_hist()
        updated = False
    ch = 0xFF & cv2.waitKey(1)
    if ch == 27 or ch == ord('q'):
        break

cv2.destroyAllWindows()


#hist = cv2.calcHist([HSV], [0, 1], None, [180, 256], [0, 180, 0, 256])
#def getpos(event,x,y,flags,param):
#    if event==cv2.EVENT_LBUTTONDOWN:
#        print(HSV[y,x])
#cv.namedWindow("hist", cv.WINDOW_NORMAL)
#cv2.resizeWindow("hist", 1200, 900)
#cv2.imshow('hist',hist)
#cv.namedWindow("image", cv.WINDOW_NORMAL)
#cv2.resizeWindow("image", 1200, 900)
#cv2.imshow('image',image)
#cv.namedWindow("imageHSV", cv.WINDOW_NORMAL)
#cv2.resizeWindow("imageHSV", 1200, 900)
#cv2.imshow("imageHSV",HSV)
#cv2.setMouseCallback("image",getpos)
#cv2.waitKey(0)#keep showing
#
#plt.imshow(hist,interpolation = 'nearest')
#plt.show()
