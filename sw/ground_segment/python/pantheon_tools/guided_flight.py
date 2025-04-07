#!/usr/env python

import os
from os import path, getenv
import sys
import json
#import geojson
import argparse
from time import sleep

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprz_connect import PprzConnect, PprzConfig
from guided_mode import GuidedMode
from settings import PprzSettingsManager
from pprzlink.message import PprzMessage

parser = argparse.ArgumentParser(description="Guided flight from json (cyber attack pantheon)")
parser.add_argument('file', help="(geo)json input file")
parser.add_argument('ac_id', help="PPRZ uav ID", type=int)
parser.add_argument('ac_name', help="Pantheon uav name", type=str)
args = parser.parse_args()

geo = None
with open(args.file) as f:
    geo = json.load(f)

if geo is None:
    print("invalid input file")
    exit()

def draw_circle(ivy, shape_id, center_lat, center_lon, radius, color):
    msg = PprzMessage('ground','SHAPE')
    msg['id'] = shape_id
    msg['linecolor'] = f'"{color}"'
    msg['fillcolor'] = f'"{color}"'
    msg['opacity'] = 0
    msg['shape'] = 0
    msg['latarr'] = [center_lat]
    msg['lonarr'] = [center_lon]
    msg['radius'] = radius
    msg['text'] = f'{shape_id}'
    ivy.send(msg)

origin = geo['origin']
data = geo[args.ac_name]
times = data['times']

pos = data['local_positions']
victims = data['victims']
victims_ids = data['victims_ids']

t0 = times[0]
tf = times[-1] - t0
t = 0
l = len(times)

try:
    connect = PprzConnect()
    sleep(2)
    conf = connect.conf_by_id(str(args.ac_id))
    settings = PprzSettingsManager(conf.settings, conf.id, connect.ivy)
    guided = GuidedMode(connect.ivy)
    settings['auto2'] = 'Guided'

    for i in range(1,l):
        print(f'time {times[i]-t0:.1f} at pos {pos[i]}')
        dt = times[i] - times[i-1]
        guided.goto_enu(conf.id, north=pos[i][0], east=pos[i][1], up=pos[i][2])
        for j, v in enumerate(victims[i]):
            v_id = victims_ids[i][j]
            print(f'found victim {v_id} at pos {v}')
            draw_circle(connect.ivy, v_id, v[1], v[0], 2., 'red')
        sleep(dt)
except KeyError:
    print(f"AC id {args.ac_id} not found")
except KeyboardInterrupt:
    print("Stopping on request")
finally:
    connect.shutdown()

