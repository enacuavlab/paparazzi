# Instructions for yolo zebra detection

## Install python tools for yolo
pip install ultralytics

## test
from IPython import display
display.clear_output()
!yolo checks

# Test prog

```
import PIL.Image
from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO("best.pt")

# img = PIL.Image.open("Muret_07-24_002.tif")
img = PIL.Image.open("anton_expo_basse.tif")
# img = PIL.Image.open("anton_expo_auto.tif)
# img = PIL.Image.open("hawkeye.tif")

# YOLO output a warning if the image dimension are not multiple of max stride 32, and rescale it. 
# Even though the outputed results are converted back to original shape, rescaling can hurt prediction performances, 
# so I am padding the image to have dimensions that are multiple of YOLO max stride = 32 and prevent rescaling
padded_img = PIL.Image.new(img.mode, ((img.width // 32 + 1 ) * 32, (img.height // 32 + 1 ) * 32 ))
padded_img.paste(img, (0, 0)) # origin of the image in top left corner

# Run inference
results = model.predict(
    source=padded_img,

    conf=0.4,
    imgsz=(padded_img.height, padded_img.width),
    # max_det=3, # maximum number of detection if needed
    )

for r in results: # actually len(results) = 1 because only 1 image
    print(r.obb) # all results info
    print(r.obb.conf) # confidence in the detection
    print(r.obb.xywhr) # rotated bounding boxes in [x_center, y_center, width, height, rotation] format
    print(r.obb.xyxyxyxy) # rotated bounding boxes in 8-point (xyxyxyxy) coordinate format : 4 points (x, y), starting from the top-left corner and moving clockwise
    print(r.obb.xyxy) # axis-aligned bounding boxes in xyxy format  
    r.plot(img=img, # plotting results on the original image without padding
           save=True,
           font_size=20,
           line_width=2,
           labels=True,
           filename="result.tif")

```
