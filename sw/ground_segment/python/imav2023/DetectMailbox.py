import cv2
import cv2 as cv
import numpy as np

def boxPoints(pts):
        if int(cv2.__version__[0]) >= 3:
                    return cv2.boxPoints(pts)
        else:
                    return cv2.cv.BoxPoints(pts)

class MailboxDetector:

    def __init__(self, hsv_th, size, aspect_ratio_th=0.8, area_th=0.7, size_th=(10,300), color="Unknown"):
        self.hsv_th = None
        self.set_hsv_th(hsv_th[0], hsv_th[1])
        self.size2 = size * size # real size in mm
        self.aspect_ratio_th = aspect_ratio_th
        self.area_th = area_th
        self.size_th = size_th
        self.kernel = np.ones((8,8),np.uint8) # create convolution
        self.mask = None
        self.color = color # color name

    def detect(self, img, size_factor=None):

        #blur = cv2.GaussianBlur(img,(5,5),0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = None
        for th in self.hsv_th:
            hsv_min = np.array(th[0])
            hsv_max = np.array(th[1])
            if mask is None:
                mask = cv2.inRange(hsv, hsv_min, hsv_max)
            else:
                mask += cv2.inRange(hsv, hsv_min, hsv_max)

        self.mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel) # opening
        #cv2.imshow('mask '+self.color,mask)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        #self.draw_all(img.copy(),cnts)
        best_res = None
        best_score = 0.
        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            _, (w, h), _ = rect
            min_wh = min(w, h)
            max_wh = max(w, h)
            if min_wh == 0 or max_wh == 0:
                continue
            if min_wh < self.size_th[0] or max_wh > self.size_th[1]:
                #print("not correct size")
                continue # too small or too big
            similarity = min_wh / max_wh
            if similarity < self.aspect_ratio_th:
                #print("not square")
                continue # not enough square
            area = w * h
            score_area = area # if no size factor, keep biggest one
            if size_factor is not None:
                score_area = 1. / max(1., abs(area - self.size2 * size_factor)) # score according to expected size
                #print(self.color, area, self.size2 * size_factor, score_area)
            area_ratio = cv2.contourArea(cnt) / area
            #print(area,cv2.contourArea(cnt),area_ratio)
            if area_ratio < self.area_th:
                #print("not good ratio")
                continue # not enough full of color
            score = area_ratio * similarity * score_area
            #print(score,best_score)
            if score > best_score:
                best_score = score
                best_res = rect
        return best_res

    def draw_all(self, img, cnts):
        for cnt in cnts:
            box = boxPoints(cv2.minAreaRect(cnt))
            ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
            cv2.drawContours(img, [ctr], -1, (0, 255, 0), 4)
        cv2.imshow('contour '+self.color,img)

    def set_hsv_th(self, th_min, th_max):
        if th_min[0] < th_max[0]: # h min < h max, normal case
            self.hsv_th = np.array([[th_min, th_max]])
        else: # split into two parts
            self.hsv_th = np.array([
                [[0        , th_min[1], th_min[2]],[th_max[0], th_max[1], th_max[2]]],
                [[th_min[0], th_min[1], th_min[2]],[179      , th_max[1], th_max[2]]]
                ])

