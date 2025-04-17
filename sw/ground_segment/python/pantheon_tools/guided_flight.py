#!/usr/env python

import os
from os import path, getenv
import sys
import json
import geojson
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
parser.add_argument('traj_file', help="json input file with traj and victims")
parser.add_argument('area_file', help="geojson input file with flight area")
parser.add_argument('ac_id', help="PPRZ uav ID", type=int)
parser.add_argument('ac_name', help="Pantheon uav name", type=str)
args = parser.parse_args()

geo_traj = None
geo_area = None
with open(args.traj_file) as f:
    geo_traj = json.load(f)
with open(args.area_file) as f:
    geo_area = geojson.load(f)

if geo_traj is None:
    print("invalid input file")
    exit()

def draw_circle(ivy, shape_id, center_lat, center_lon, radius, color):
    msg = PprzMessage('ground','SHAPE')
    msg['id'] = shape_id
    msg['linecolor'] = f'"{color}"'
    msg['fillcolor'] = f'"{color}"'
    msg['opacity'] = 0
    msg['shape'] = 0
    msg['status'] = 0
    msg['latarr'] = [ int(1e7 * center_lat) ]
    msg['lonarr'] = [ int(1e7 * center_lon) ]
    msg['radius'] = radius
    msg['text'] = f'"{shape_id}"'
    ivy.send(msg)

def draw_lines(ivy, shape_id, lines_lat, lines_lon, color):
    msg = PprzMessage('ground','SHAPE')
    msg['id'] = shape_id
    msg['linecolor'] = f'"{color}"'
    msg['fillcolor'] = f'"{color}"'
    msg['opacity'] = 0
    msg['shape'] = 2
    msg['status'] = 0
    msg['latarr'] = [ int(1e7 * lat) for lat in lines_lat ]
    msg['lonarr'] = [ int(1e7 * lon) for lon in lines_lon ]
    msg['radius'] = 0.
    msg['text'] = '" "'
    ivy.send(msg)

origin = geo_traj['origin']
data = geo_traj[args.ac_name]
times = data['times']

pos = data['local_positions']
pos_global = data['global_positions']
victims = data['victims']
victims_ids = data['victims_ids']

areas = []
for e in geo_area['features']:
    _type = e['properties']['type']
    if _type == 'Areas':
        areas = e['geometry']['coordinates']
        # only one areas node is expected

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
    settings['max_speed'] = 13.
    lines_lat = [pos_global[0][1]]
    lines_lon = [pos_global[0][0]]
    sleep(2)
    print(areas)
    for i, area in enumerate(areas):
        print(area[0])
        area_lat = [ p[1] for p in area[0] ]
        area_lon = [ p[0] for p in area[0] ]
        print(area_lat)
        print(area_lon)
        draw_lines(connect.ivy, 200+i, area_lat, area_lon, 'orange')

    for i in range(1,l):
        print(f'time {times[i]-t0:.1f} at pos {pos[i]}')
        dt = times[i] - times[i-1]
        guided.goto_enu(conf.id, north=pos[i][1], east=pos[i][0], up=pos[i][2])
        for j, v in enumerate(victims[i-1]):
            v_id = victims_ids[i-1][j]
            print(f'found victim {v_id} at pos {v}')
            draw_circle(connect.ivy, v_id, v[1], v[0], 2., 'red')
        lines_lat.append(pos_global[i][1])
        lines_lon.append(pos_global[i][0])
        draw_lines(connect.ivy, 100, lines_lat, lines_lon, 'lime')
        sleep(dt)
except KeyError:
    print(f"AC id {args.ac_id} not found")
except KeyboardInterrupt:
    print("Stopping on request")
finally:
    connect.shutdown()

