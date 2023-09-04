#!/usr/bin/env python3
import cv2
from DetectMailbox import MailboxDetector
import numpy as np
import rasterio.warp
from rasterio.crs import CRS

def boxPoints(pts):
    if int(cv2.__version__[0]) >= 3:
        return cv2.boxPoints(pts)
    else:
        return cv2.cv.BoxPoints(pts)

DEFAULT_IMAGE_VIEWER = "gwenview"
DEFAULT_IMAGE_OUTPUT = "out_detect.png"
DEFAULT_SCALE_FACTOR = 4
DEFAULT_RESOLUTION = 20 # pixels per meter

mailbox_blue = MailboxDetector([[93, 90, 0],[138, 255, 255]], 1200, color="BLUE")


def get_geo_data(filename):
    import os
    if os.path.splitext(filename)[1] == '.tif':
        try:
            import rasterio
            # Open tif file
            print(f"opening {filename}...")
            ds = rasterio.open(filename)
            return ds
        except:
            print('failed loading rasterio')
            return None
    else:
        print('not a tif file')
        return None

def process_result(img, out, res, label, geo=None):
    center = (int(res[0][0]), int(res[0][1]))
    cv2.circle(out, center, 50, (0, 255, 0, 255), 5)
    #cv2.putText(out, label, (center[0]+60, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
    cv2.putText(out, label, (center[0]+60, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0, 255))

    if geo is not None:
        # get pixel coordinates
        # geo.xy(row, col)
        coord = geo.xy(center[1], center[0])
        if coord is not None:
            # transform to WGS84
            lons, lats = rasterio.warp.transform(geo.crs, CRS.from_epsg(4326), [coord[0]], [coord[1]])
            print(f"{label} {center} {lats[0]:.07f}, {lons[0]:.07f}")
            cv2.putText(out, '{:.7f} {:.7f}'.format(lats[0], lons[0]), (center[0]+60, center[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0, 255))
    box = boxPoints(res)
    ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
    mask = np.zeros((img.shape[0], img.shape[1], 1), np.uint8)
    cv2.drawContours(mask, [ctr], -1, (255,255,255),-1)
    img = cv2.bitwise_and(img,img,mask = cv2.bitwise_not(mask))
    return img, out

def find_mailboxes(img, output=None, scale=DEFAULT_SCALE_FACTOR, res=DEFAULT_RESOLUTION, geo=None):
    out = img.copy()

    scale_factor = pow(res / 1000., 2)


    cnt = 1
    while True:
        res = mailbox_blue.detect(img, scale_factor)
        if res is not None:
            img, out = process_result(img, out, res, f"BLUE_{cnt}", geo)
            cnt += 1
        else:
            break


    if output is None:
        w, h, _ = img.shape
        img_out = cv2.resize(out, (int(h/scale),int(w/scale)))
        cv2.imshow('frame',img_out)
        while True:
            if cv2.waitKey(-1)  & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
    else:
        cv2.imwrite(output, out)

if __name__ == '__main__':
    '''
    When used as a standalone script
    '''
    import argparse
    import subprocess

    parser = argparse.ArgumentParser(description="Search mailboxes in image")
    parser.add_argument('img', help="image path")
    parser.add_argument("-v", "--viewer", help="program used to open the image", default=DEFAULT_IMAGE_VIEWER)
    parser.add_argument("-nv", "--no_view", help="Do not open image after processing", action='store_true')
    parser.add_argument("-o", "--output", help="output file name", default=None)
    parser.add_argument("-s", "--scale", help="resize scale factor", type=int, default=DEFAULT_SCALE_FACTOR)
    parser.add_argument("-r", "--resolution", help="resolution in pixels per meter", type=float, default=DEFAULT_RESOLUTION)
    args = parser.parse_args()

    img = cv2.imread(args.img, cv2.IMREAD_UNCHANGED)
    geo = get_geo_data(args.img)
    find_mailboxes(img, args.output, args.scale, args.resolution, geo)

    if not args.no_view and args.output is not None:
        subprocess.call([args.viewer, args.output])

