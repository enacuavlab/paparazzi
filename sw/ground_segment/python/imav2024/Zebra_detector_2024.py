#!/usr/bin/env python3
import cv2
from DetectRectangle import RectangleDetector
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

mailbox_zebra = RectangleDetector([[0, 0, 230],[179, 15, 255]], (1.7, 0.6), color_name="Zebra")


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

def process_result(img, out, res, label, geo=None, color=(0, 255, 0)):
    center = (int(res[0][0]), int(res[0][1]))
    cv2.circle(out, center, 50, color, 5)
    #cv2.putText(out, label, (center[0]+60, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
    cv2.putText(out, label, (center[0]+60, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))

    if geo is not None:
        if geo.crs is None:
            print(f"{label} {center} | no valid geo data")
        else:
            # get pixel coordinates
            coord = geo.xy(center[0], center[1])
            if coord is not None:
                # transform to WGS84
                lons, lats = rasterio.warp.transform(geo.crs, CRS.from_epsg(4326), [coord[0]], [coord[1]])
                print(f"{label} {center} {lats[0]:.07f}, {lons[0]:.07f}")
                cv2.putText(out, '{:.7f} {:.7f}'.format(lats[0], lons[0]), (center[0]+60, center[1]+30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color)
            else:
                print(f"{label} {center} | no valid geo coordinates")
    return img, out

def find_mailboxes(img, output=None, scale=DEFAULT_SCALE_FACTOR, res=DEFAULT_RESOLUTION, geo=None, show_errors=False):
    out = img.copy()

    results, error = mailbox_zebra.detect(img, res)
    #print('results', len(results))
    for cnt, result in enumerate(results):
        #print(f'ZEBRA_{cnt} | {result[0]} | {result[3]}')
        img, out = process_result(img, out, result[2], f"ZEBRA_{cnt}", geo)
    if show_errors:
        print('number of errors', len(error))
        for cnt, result in enumerate(error):
            s = ' '.join(format(f, '.3f') for f in result[3])
            img, out = process_result(img, out, result[2], f"{result[1]} | {s}", geo, color=(0, 0, 255))

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
    parser.add_argument("-e", "--error", help="show invalid detection", type=bool, default=False)
    args = parser.parse_args()

    img = cv2.imread(args.img)
    geo = get_geo_data(args.img)
    find_mailboxes(img, args.output, args.scale, args.resolution, geo=geo, show_errors=args.error)

    if not args.no_view and args.output is not None:
        subprocess.call([args.viewer, args.output])

