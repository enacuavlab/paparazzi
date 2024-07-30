import libjevois as jevois
import numpy as np
import cv2
import cv2 as cv
import cv2.aruco as aruco
from DetectRectangle import RectangleDetector
import math

# define JeVois module, process function

class LongRangeArUco:

    def __init__(self):
        self.alt = 1000. # in mm from AP, default in image plane

        # initial color thresholds
        self.rect_aruco = RectangleDetector([[0, 0, 230],[179, 15, 255]],
                (0.3, 0.3),
                aspect_ratio_th=0.2,
                area_th=0.8,
                size_th=1.1, # disable size check for now
                color_name="")

        # aruco params
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.parameters =  aruco.DetectorParameters_create()
        self.marker_len = 150

        # cam params
        self.cam_matrix = np.array([[770., 0., 320.],[0., 770., 240.],[0., 0., 1.]])
        self.cam_dist = np.array([[0.], [0.], [0.], [0.]])
        self.use_fisheye = False

        self.save = None # save current image

    def processNoUSB(self, inframe):
        img = inframe.getCvBGR()
        self.processImage(img) # no need to process returned data

    def process(self, inframe, outframe):
        img = inframe.getCvBGR()
        detect = self.processImage(img)
        if detect is not None:
            if detect[0] == "rect":
                box = cv2.boxPoints(detect[0])
                ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
                cv2.drawContours(img, [ctr], -1, (0, 255, 0), 4)
            elif detect[0] == "tag":
                frame_markers = aruco.drawDetectedMarkers(img, [detect[1]], [detect[2]])
        outframe.sendCv(img)
        #outframe.sendCv(self.mailbox_yellow.mask)

    def processImage(self, img):
        '''
        process a single image
        return a dict with detected featured
        '''
        if self.save is not None:
            cv2.imwrite(self.save, img)
            jevois.LINFO(self.save)
            self.save = None

        # ArUco detector
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        tvec = None
        if ids is not None and len(ids) > 0:
            # only first match ?
            _, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0],
                    self.marker_len, self.cam_matrix, self.cam_dist)

        if tvec is not None:
            self.send_message_pose(ids[0], tvec[0])
            return ("tag", corners[0], ids[0])

        # FIXME compute resolution correctly
        resolution = math.sqrt(self.cam_matrix[0][0] * self.cam_matrix[1][1] / (self.alt * self.alt))

        # white square detection
        rect, _ = self.rect_aruco.detect(img, resolution)

        if len(rect) > 0:
            # send best detection
            self.send_message_rect('0', rect[0])
            return ("rect", rect[0])

        return None

    def send_message_pose(self, mark, pos):
        '''
        send estimated pose
        '''
        jevois.sendSerial("D3 U{} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} 1.0 1.0 0.0 0.0 0.0".format(mark,
            pos[0], pos[1], pos[2],
            self.marker_len, self.marker_len))


    def send_message_rect(self, mark, pos):
        '''
        send message over serial link to AP
        '''
        (u, v), (w, h), _ = pos
        x ,y = 0., 0.
        z = self.alt
        # pos in "mm" in world frame, assuming cam is looking down
        # and alt is the distance to onject
        if self.use_fisheye:
            pts_uv = np.array([[[u, v]]], dtype=np.float32)
            undist = cv2.fisheye.undistortPoints(pts_uv, self.calib_fisheye[0], self.calib_fisheye[1])
            x = z * undist[0][0][0]
            y = z * undist[0][0][1]
        else:
            x = z * (u - self.cam_matrix[0][0]) / self.cam_matrix[0][2]
            y = z * (v - self.cam_matrix[1][1]) / self.cam_matrix[1][2]
        jevois.sendSerial("D3 U{} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} 1.0 1.0 0.0 0.0 0.0".format(mark, x, y, z, w, h))

    def parseSerial(self, cmd):
        str_list = cmd.split(' ')
        str_len = len(str_list)
        if str_len == 2 and str_list[0] == "alt" and str_list[1].isdigit():
            self.alt = max(float(str_list[1]), 1000.)
            return "OK"
        elif str_len == 2 and str_list[0] == "marker_len" and str_list[1].isdigit():
            self.marker_len = int(str_list[1])
            return "OK"
        elif str_len == 2 and str_list[0] == "save":
            self.save = "/jevois/data/images/{}.png".format(str_list[1])
            return self.save
        elif str_len == 7 and str_list[0] == "hsv":
            h_min = [int(str_list[1]), int(str_list[2]), int(str_list[3])]
            h_max = [int(str_list[4]), int(str_list[5]), int(str_list[6])]
            self.rect_aruco.set_hsv_th(h_min, h_max)
            return "OK"
        elif str_len == 5 and str_list[0] == "calib":
            self.cam_matrix = np.array([[float(str_list[1]), 0., float(str_list[3])], [0., float(str_list[2]), float(str_list[4])], [0., 0., 1.]])
            self.cam_dist = np.array([[0.], [0.], [0.], [0.]])
            self.use_fisheye = False
            return "OK"
        elif str_len == 9 and str_list[0] == "calib_fisheye":
            self.cam_matrix = np.array([[float(str_list[1]), 0., float(str_list[3])], [0., float(str_list[2]), float(str_list[4])], [0., 0., 1.]])
            self.cam_dist = np.array([[float(str_list[5])], [float(str_list[6])], [float(str_list[7])], [float(str_list[8])]])
            self.use_fisheye = True
            return "OK"
        return "ERR"

    def supportedCommands(self):
        return "alt - set alt in mm"

