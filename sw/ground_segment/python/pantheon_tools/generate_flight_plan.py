#!/usr/env python

import os
import json
import geojson
import lxml.etree as etree
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description="Create flight plan from geojson")
parser.add_argument('file', help="geojson input file")
parser.add_argument('-o', '--output', help="output flight plan base name", default="fp")
parser.add_argument('-d', '--delay', help="add a constant delay in seconds between takeoff", type=int, default=0)
args = parser.parse_args()

try:
    from srtm import Srtm1HeightMapCollection
    from srtm.exceptions import NoHeightMapDataException
    srtm_data = Srtm1HeightMapCollection()
    if os.getenv("SRTM1_DIR") is None:
        os.environ["SRTM1_DIR"] = '.'
    use_srtm = True
except:
    print('SRTM lib not found')
    use_srtm = False

geo = None

with open(args.file) as f:
    geo = geojson.load(f)
    #print(geo)

if geo is None:
    print("invalid input file")
    exit()


def START_FP(delay=0):
    return f'''
  <blocks>
    <block name="Wait GPS">
      <call_once fun="NavKillThrottle()"/>
      <while cond="!GpsFixValid()"/>
      <while cond="LessThan(NavBlockTime(), {10 + delay})"/>
    </block>
    <block name="Takeoff">
      <exception cond="GetPosHeight() @GT 10.0" deroute="Trajectory"/>
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
ground_alt = 0.
print(f'generating flight plans for {nb_drones} drones of type {model}')

for i in range(nb_drones):
    lon0, lat0 = ref_points[i][0], ref_points[i][1]
    if use_srtm:
        try:
            ground_alt = srtm_data.get_altitude(longitude=lon0, latitude=lat0)
        except NoHeightMapDataException:
            print('No SRTM data avalaible')
    fp = etree.Element("flight_plan")
    fp.set('alt', str(ground_alt+ref_points[i][2]))
    fp.set('ground_alt', str(ground_alt))
    fp.set('lon0', str(lon0))
    fp.set('lat0', str(lat0))
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
        lon, lat, height = wp[0], wp[1], wp[2]
        galt = 0.
        if use_srtm:
            try:
                galt = srtm_data.get_altitude(longitude=lon, latitude=lat)
            except NoHeightMapDataException:
                print('No SRTM data avalaible')
        wpt = etree.SubElement(wpts, "waypoint")
        wpt.set('name', f'P{j}')
        wpt.set('lon', str(lon))
        wpt.set('lat', str(lat))
        wpt.set('alt', str(galt+height))
        nb_wpts += 1

    start_fp = etree.fromstring(START_FP(i * args.delay))
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
        go.set('vmode', 'glide')

    end_fp = etree.fromstring(END_FP)
    for b in end_fp.getchildren():
        blocks.append(b)

    etree.indent(fp)
    #print(etree.tostring(fp, pretty_print=True).decode())
    fp_file_name = f'{args.output}_{Path(args.file).stem}_{i}.xml'
    print('writing flight plan file', fp_file_name)
    with open(fp_file_name, 'w') as f:
        f.write(etree.tostring(fp, pretty_print=True).decode())

