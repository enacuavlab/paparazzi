#!/usr/env python

import json
import geojson
import lxml.etree as etree
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description="Create flight plan from geojson")
parser.add_argument('file', help="geojson input file")
parser.add_argument('-o', '--output', help="output flight plan base name", default="fp")
args = parser.parse_args()

geo = None

with open(args.file) as f:
    geo = geojson.load(f)
    #print(geo)

if geo is None:
    print("invalid input file")
    exit()


START_FP = '''
  <blocks>
    <block name="Wait GPS">
      <call_once fun="NavKillThrottle()"/>
      <while cond="!GpsFixValid()"/>
      <while cond="LessThan(NavBlockTime(), 10)"/>
    </block>
    <block name="Takeoff">
      <exception cond="GetPosAlt() @GT 2.0" deroute="Trajectory"/>
      <call_once fun="NavResurrect()"/>
      <stay vmode="climb" climb="nav.climb_vspeed" wp="HOME"/>
    </block>
  </blocks>
  '''

END_FP = '''
  <blocks>
    <block name="land">
      <exception cond="!nav_is_in_flight()" deroute="landed"/>
      <stay climb="nav.descend_vspeed" vmode="climb" wp="HOME"/>
    </block>
    <block name="landed">
      <attitude pitch="0" roll="0" throttle="0" vmode="throttle" until="FALSE"/>
    </block>
  </blocks>
  '''

nb_drones = geo['features'][0]['properties']['number']
model = geo['features'][0]['properties']['model']
ref_points = geo['features'][0]['geometry']['coordinates']
trajectories = geo['features'][1]['geometry']['coordinates']
#print(ref_points)
print(f'generating flight plans for {nb_drones} drones of type {model}')

for i in range(nb_drones):
    fp = etree.Element("flight_plan")
    fp.set('alt', '120')
    fp.set('ground_alt', '0')
    fp.set('lon0', str(ref_points[i][0]))
    fp.set('lat0', str(ref_points[i][1]))
    fp.set('max_dist_from_home', '10000')
    fp.set('security_height', '50')
    fp.set('name', f'FP from {args.file} drone {i}')
    wpts = etree.SubElement(fp, "waypoints")
    blocks = etree.SubElement(fp, "blocks")
    home = etree.SubElement(wpts, "waypoint")
    home.set('name', 'HOME')
    home.set('x', '0.')
    home.set('y', '0.')
    
    nb_wpts = 1
    for j, wp in enumerate(trajectories[i][1:-1]):
        wpt = etree.SubElement(wpts, "waypoint")
        wpt.set('name', f'P{j+1}')
        wpt.set('lon', str(wp[0]))
        wpt.set('lat', str(wp[1]))
        nb_wpts += 1

    start_fp = etree.fromstring(START_FP)
    for b in start_fp.getchildren():
        blocks.append(b)

    traj = etree.SubElement(blocks, "block")
    traj.set('name', 'Trajectory')
    for j in range(nb_wpts):
        go = etree.SubElement(traj, "go")
        if j == 0:
            go.set('from', 'HOME')
        else:
            go.set('from', f'P{j-1}')
        if j == nb_wpts-1:
            go.set('wp', 'HOME')
        else:
            go.set('wp', f'P{j}')
        go.set('hmode', 'route')

    end_fp = etree.fromstring(END_FP)
    for b in end_fp.getchildren():
        blocks.append(b)

    etree.indent(fp)
    #print(etree.tostring(fp, pretty_print=True).decode())
    fp_file_name = f'{args.output}_{Path(args.file).stem}_{i}.xml'
    print('writing flight plan file', fp_file_name)
    with open(fp_file_name, 'w') as f:
        f.write(etree.tostring(fp, pretty_print=True).decode())

