#!/usr/bin/env python3
#
# Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

'''
Display and store images from video source when receiving MARK message
'''


import sys
from os import path, getenv
from time import sleep
import argparse
import cv2
import threading

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# parse args
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-ac', '--ac_id', dest='ac_id', type=int, help='Filter on AC_ID')
parser.add_argument('-s', '--source', dest='source_id', type=int, default=1, help="Video source ID")
parser.add_argument('-d', '--dest', dest='dest', default=".", help="Destination folder to save pictures")
args = parser.parse_args()

class ImageCapture():

    def __init__(self):
        # start ivy interface
        self.ivy = IvyMessagesInterface("image_capture")
        # start video capture
        self.cap = cv2.VideoCapture(args.source_id)

        self.new_msg = False
        self.running = True
        self.name = None

        def mark_msg_cb(ac_id, msg):
            if args.ac_id is None or args.ac_id == ac_id:
                self.new_msg = True
                self.name = f'capture_{msg["ac_id"]}_{msg["lat"]}_{msg["long"]}'
        self.ivy.subscribe(mark_msg_cb, PprzMessage("telemetry", "MARK"))

    def run(self):
        if not self.cap.isOpened():
            print("camera not opened")
            self.stop()
            return
        self.cap.grab()
        if self.new_msg:
            print("Got new image", self.name, flush=True)
            _, frame = self.cap.retrieve()
            cv2.imshow(self.name, frame)
            cv2.imwrite(path.join(args.dest,self.name+'.jpg'), frame)
            self.new_msg = False
        cv2.waitKey(1)

    def stop(self):
        self.running = False
        self.ivy.shutdown()
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        print(f"Start image_capture with params {args}", flush=True)
        capture = ImageCapture()
        while capture.running:
            capture.run()
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down image_capture", flush=True)
        capture.stop()

