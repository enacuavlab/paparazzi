import cv2
import cv2 as cv
import numpy as np

def boxPoints(pts):
        if int(cv2.__version__[0]) >= 3:
                    return cv2.boxPoints(pts)
        else:
                    return cv2.cv.BoxPoints(pts)

class RectangleDetector:

    def __init__(self, hsv_th, size, aspect_ratio_th=0.2, area_th=0.5, size_th=0.2, color_name="Unknown"):
        self.hsv_th = None
        self.set_hsv_th(hsv_th[0], hsv_th[1])
        self.size_min = min(size) # in meters
        self.size_max = max(size) # in meters
        self.aspect_ratio = self.size_max / self.size_min
        self.area = self.size_max * self.size_min
        #self.size2 = size * size * aspect_ratio # real size in mm
        self.aspect_ratio_th = aspect_ratio_th
        self.area_th = area_th
        self.size_th = size_th
        self.kernel = np.ones((8,8),np.uint8) # create convolution
        self.mask = None
        self.color_name = color_name # color name

    def detect(self, img, resolution):

        #blur = cv2.GaussianBlur(img,(5,5),0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = None
        for th in self.hsv_th:
            #print('hsv th',th)
            hsv_min = np.array(th[0])
            hsv_max = np.array(th[1])
            if mask is None:
                mask = cv2.inRange(hsv, hsv_min, hsv_max)
            else:
                mask += cv2.inRange(hsv, hsv_min, hsv_max)

        self.mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel) # opening
        #cv2.imshow('mask '+self.color_name,mask)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        #self.draw_all(img.copy(),cnts)
        valid = []
        error = []
        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            _, (w, h), _ = rect
            if w < 5 or h < 5:
                continue # not enough pixels
            min_wh = min(w, h) / float(resolution)
            max_wh = max(w, h) / float(resolution)
            aspect_ratio = max_wh / min_wh
            area = min_wh * max_wh
            error_ar = min(1., abs(aspect_ratio - self.aspect_ratio) / self.aspect_ratio)
            error_area = min(1., abs(area - self.area) / self.area)
            error_size_min = abs(min_wh - self.size_min) / self.size_min
            error_size_max = abs(max_wh - self.size_max) / self.size_max
            area_ratio = cv2.contourArea(cnt) / (area * resolution * resolution)
            score = area_ratio * (1. - error_area) * (1. - error_ar)
            if error_size_min > self.size_th or error_size_max > self.size_th:
                # too small or too big
                error.append((score, 'size', rect, (min_wh, max_wh, error_size_min, error_size_max, self.size_th)))
            elif error_ar > self.aspect_ratio_th:
                # not correct aspect ratio
                error.append((score, 'aspect', rect, (error_ar, aspect_ratio, self.aspect_ratio, self.aspect_ratio_th)))
            #print(area,cv2.contourArea(cnt),area_ratio)
            elif area_ratio < self.area_th:
                # not enough full of color
                error.append((score, 'filling', rect, (area_ratio, self.area_th)))
            else:
                valid.append((score, 'valid', rect, (min_wh, max_wh, error_ar, error_size_min, error_size_max, area_ratio)))
        valid = sorted(valid, key=lambda k: k[0], reverse=True)
        return valid, error

    def draw_all(self, img, cnts):
        for cnt in cnts:
            box = boxPoints(cv2.minAreaRect(cnt))
            ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
            cv2.drawContours(img, [ctr], -1, (0, 255, 0), 4)
        cv2.imshow('contour '+self.color_name,img)

    def set_hsv_th(self, th_min, th_max):
        if th_min[0] < th_max[0]: # h min < h max, normal case
            self.hsv_th = np.array([[th_min, th_max]])
        else: # split into two parts
            self.hsv_th = np.array([
                [[0        , th_min[1], th_min[2]],[th_max[0], th_max[1], th_max[2]]],
                [[th_min[0], th_min[1], th_min[2]],[179      , th_max[1], th_max[2]]]
                ])

