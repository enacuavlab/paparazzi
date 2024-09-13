import cv2
import numpy as np
import rasterio.warp
from rasterio.crs import CRS
import PIL.Image
from ultralytics import YOLO

DEFAULT_IMAGE_OUTPUT = "yolo_out_detect.png"
DEFAULT_SCALE_FACTOR = 1

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

def process_result(out, res, label, geo=None, color=(0, 255, 0)):
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
    return out

def find_zebras(img_name, output=None, scale=DEFAULT_SCALE_FACTOR, geo=None, show_errors=False, show_image=True):
    out = cv2.imread(img_name)
    img = PIL.Image.open(img_name)
    
    # Load a pretrained YOLOv8n model
    model = YOLO("best.pt")

    # YOLO output a warning if the image dimension are not multiple of max stride 32,
    # and rescale it. 
    # Even though the outputed results are converted back to original shape,
    # rescaling can hurt prediction performances, 
    # so I am padding the image to have dimensions that are multiple
    # of YOLO max stride = 32 and prevent rescaling
    padded_img = PIL.Image.new(img.mode, ((img.width // 32 + 1 ) * 32, (img.height // 32 + 1 ) * 32 ))
    padded_img.paste(img, (0, 0)) # origin of the image in top left corner
    
    # Run inference
    results = model.predict(
        source=padded_img,
        conf=0.4,
        imgsz=(padded_img.height, padded_img.width),
        # max_det=3, # maximum number of detection if needed
        )
 
    for res in results:
        for cnt, r in enumerate(res):
            out = process_result(out, r.obb.xywhr, f"ZEBRA_{cnt}", geo)

    if output is not None:
        cv2.imwrite(output, out)

    if show_image:
        w, h, _ = out.shape
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        img_out = cv2.resize(out, (int(h/scale),int(w/scale)))
        cv2.imshow('frame',img_out)
        while True:
            if cv2.waitKey(10)  & 0xFF == ord('q'):
                break
            if cv2.getWindowProperty('frame',cv2.WND_PROP_VISIBLE) < 1:
                break
        cv2.destroyAllWindows()

    #for r in results: # actually len(results) = 1 because only 1 image
    #    print(r.obb) # all results info
    #    print(r.obb.conf) # confidence in the detection
    #    print(r.obb.xywhr) # rotated bounding boxes in [x_center, y_center, width, height, rotation] format
    #    print(r.obb.xyxyxyxy) # rotated bounding boxes in 8-point (xyxyxyxy) coordinate format : 4 points (x, y), starting from the top-left corner and moving clockwise
    #    print(r.obb.xyxy) # axis-aligned bounding boxes in xyxy format  
    #    r.plot(img=img, # plotting results on the original image without padding
    #           save=True,
    #           font_size=20,
    #           line_width=2,
    #           labels=True,
    #           filename="result.tif")


if __name__ == '__main__':
    '''
    When used as a standalone script
    '''
    import argparse

    parser = argparse.ArgumentParser(description="Search zebras in image with YOLO")
    parser.add_argument('img', help="image path")
    parser.add_argument("-nv", "--no_view", help="Do not open image after processing", action='store_true')
    parser.add_argument("-o", "--output", help="output file name", default=None)
    parser.add_argument("-s", "--scale", help="resize scale factor", type=int, default=DEFAULT_SCALE_FACTOR)
    args = parser.parse_args()

    geo = get_geo_data(args.img)
    find_zebras(args.img, args.output, args.scale, geo=geo, show_image=(not args.no_view))

