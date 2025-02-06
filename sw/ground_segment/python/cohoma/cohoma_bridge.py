#!/usr/bin/env python3
#
# Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

import sys
from os import path, getenv
import math
import numpy as np
from dataclasses import dataclass
from enum import Enum

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from flight_plan import FlightPlan


@dataclass
class UAVData:
    '''
    Store UAV information
    '''
    ac_id: str
    name: str
    lat: float = 0.
    lon: float = 0.
    alt: float = 0.
    height: float = 0.
    heading: float = 0.
    vnorth: float = 0.
    veast: float = 0.
    vup: float = 0.
    bat_voltage: float = 0.
    bat_charge: float = 0.
    gps_tow: int = 0
    flight_time: int = 0
    AP_mode: str = 'none'
    FP_block: str = 'none'
    mission_status: list[int] = None
    datalink_lost_time: int = 0
    flight_plan: FlightPlan = None
    start_mission_fp_block: int = None

class MissionInsert(Enum):
    APPEND = 0
    PREPEND = 1
    REPLACE_CURRENT = 2
    REPLACE_ALL = 3


class MissionManager():
    '''
    API to create and manage mission elements
    and start mission mode from flight plan
    Bind to state messages and update UAV data
    '''
    def __init__(self, verbose=False):
        self.verbose = verbose

        self.uavs = {}

        ''' get aircraft config '''
        def connect_cb(conf):
            if not conf.id in self.uavs:
                self.uavs[conf.id] = UAVData(conf.id, conf.name)
            self.uavs[conf.id].flight_plan = FlightPlan.parse(conf.flight_plan)
            try:
                self.uavs[conf.id].start_mission_fp_block = self.uavs[conf.id].flight_plan.get_block('Mission')
                if self.verbose:
                    print("'Mission' block found",self.uavs[conf.id].start_mission_fp_block.no)
            except:
                print("'Mission' block not found in flight plan")
            if self.verbose:
                print(conf)

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=connect_cb)

        ''' bind to messages '''
        self.connect.ivy.subscribe(self.flight_param_cb, PprzMessage("ground", "FLIGHT_PARAM"))
        self.connect.ivy.subscribe(self.ap_status_cb, PprzMessage("ground", "AP_STATUS"))
        self.connect.ivy.subscribe(self.nav_status_cb, PprzMessage("ground", "NAV_STATUS"))
        self.connect.ivy.subscribe(self.engine_status_cb, PprzMessage("ground", "ENGINE_STATUS"))
        self.connect.ivy.subscribe(self.telemetry_status_cb, PprzMessage("ground", "TELEMETRY_STATUS"))
        self.connect.ivy.subscribe(self.mission_status_cb, PprzMessage("telemetry", "MISSION_STATUS"))

    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()

    def flight_param_cb(self, ac_id, msg):
        if ac_id in self.uavs:
            self.uavs[ac_id].lat = float(msg['lat'])
            self.uavs[ac_id].lon = float(msg['long'])
            self.uavs[ac_id].alt = float(msg['alt'])
            self.uavs[ac_id].height = float(msg['agl']) # FIXME height or AGL ?
            self.uavs[ac_id].heading = float(msg['heading'])
            speed = float(msg['speed'])
            course = np.deg2rad(float(msg['course']))
            self.uavs[ac_id].vnorth = speed * math.cos(course)
            self.uavs[ac_id].veast = speed * math.sin(course)
            self.uavs[ac_id].vup = float(msg['climb'])
            self.uavs[ac_id].gps_tow = int(msg['itow']) / 1000 # FIXME s or ms ?

    def ap_status_cb(self, ac_id, msg):
        if ac_id in self.uavs:
            self.uavs[ac_id].AP_mode = msg['ap_mode']
            self.uavs[ac_id].flight_time = msg['flight_time']

    def nav_status_cb(self, ac_id, msg):
        if ac_id in self.uavs:
            block_id = int(msg['cur_block'])
            block = self.uavs[ac_id].flight_plan.get_block(block_id)
            self.uavs[ac_id].FP_block = block.name

    def engine_status_cb(self, ac_id, msg):
        if ac_id in self.uavs:
            self.uavs[ac_id].bat_voltage = float(msg['bat'])
            # TODO compute % assuming 2 or 4 cells

    def telemetry_status_cb(self, ac_id, msg):
        if ac_id in self.uavs:
            self.uavs[ac_id].datalink_lost_time = int(msg['uplink_lost_time'])

    def mission_status_cb(self, ac_id, msg):
        ac_id = str(ac_id)
        if ac_id in self.uavs:
            self.uavs[ac_id].mission_status = [ int(e) for e in msg['index_list'] if int(e) != 0 ] 


    def start_mission(self, ac_id:int=None):
        ''' enter Mission flight block for selected UAV or all if None'''
        def jump_to_block(_ac_id, _block_id):
            msg = PprzMessage("ground", "JUMP_TO_BLOCK")
            msg['ac_id'] = _ac_id
            msg['block_id'] = int(_block_id)
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)

        if ac_id is not None and ac_id in self.uavs:
            jump_to_block(ac_id, self.uavs[ac_id].start_mission_fp_block.no)
        elif ac_id is None:
            for _id in self.uavs:
                jump_to_block(_id, self.uavs[_id].start_mission_fp_block.no)

    def add_mission_point(self, mission_id:int, lat:float, lon:float, alt:float, insert_mode:MissionInsert = MissionInsert.APPEND, ac_id:int=None):
        ''' send MISSION_GOTO_WP_LLA message to a specified uav or all if None
            point is described by lat (deg), lon (deg), alt amsl (m) format
        '''
        def send_point(_ac_id, _lat, _lon, _alt, _index, _insert):
            msg = PprzMessage("datalink", "MISSION_GOTO_WP_LLA")
            msg['ac_id'] = _ac_id
            msg['insert'] = _insert
            msg['duration'] = -1.
            msg['index'] = _index % 256
            msg['wp_lat'] = int(_lat * 1e7)
            msg['wp_lon'] = int(_lon * 1e7)
            msg['wp_alt'] = int(_alt * 1e3)
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)

        if mission_id <= 0 or mission_id > 255:
            print('invalid mission id', mission_id)
            return # TODO raise error

        if ac_id is not None and ac_id in self.uavs:
            send_point(ac_id, lat, lon, alt, mission_id, insert_mode.value)
        elif ac_id is None:
            for _id in self.uavs:
                send_point(_id, lat, lon, alt, mission_id, insert_mode.value)

    def add_mission_path(self, mission_id:int, path:list[(float,float)], alt:float, insert_mode:MissionInsert = MissionInsert.APPEND, ac_id:int=None):
        ''' send MISSION_PATH_LLA message to a specified uav or all if None
            path is described by a list of points in (lat (deg), lon (deg)) + alt amsl (m) format
        '''
        def send_path(_ac_id, _path, _alt, _index, _insert):
            msg = PprzMessage("datalink", "MISSION_PATH_LLA")
            msg['ac_id'] = _ac_id
            msg['insert'] = _insert
            msg['duration'] = -1.
            msg['nb'] = max(len(_path), 5)
            msg['index'] = _index % 256
            msg['path_alt'] = int(_alt * 1e3)
            for i in range(msg['nb']):
                lat, lon = _path[i]
                msg['point_lat_'+str(i+1)] = int(lat * 1e7)
                msg['point_lon_'+str(i+1)] = int(lon * 1e7)
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)

        if len(path) > 5 or mission_id <= 0 or mission_id > 255:
            print('invalid mission id or path len', mission_id, len(path))
            return # TODO raise error

        if ac_id is not None and ac_id in self.uavs:
            send_path(ac_id, path, alt, mission_id, insert_mode.value)
        elif ac_id is None:
            for _id in self.uavs:
                send_path(_id, path, alt, mission_id, insert_mode.value)

    def add_mission_circle(self, mission_id:int, lat:float, lon:float, alt:float, radius:float, insert_mode:MissionInsert = MissionInsert.APPEND, ac_id:int=None):
        ''' send MISSION_CIRCLE_LLA message to a specified uav or all if None
            circle is described by a lat (m), lon (m), alt amsl (m) format and radius (m)
        '''
        def send_circle(_ac_id, _lat, _lon, _alt, _radius, _index, _insert):
            msg = PprzMessage("datalink", "MISSION_CIRCLE_LLA")
            msg['ac_id'] = _ac_id
            msg['insert'] = _insert
            msg['duration'] = -1.
            msg['index'] = _index % 256
            msg['center_lat'] = int(_lat * 1e7)
            msg['center_lon'] = int(_lon * 1e7)
            msg['center_alt'] = int(_alt * 1e3)
            msg['radius'] = _radius
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)

        if mission_id <= 0 or mission_id > 255:
            print('invalid mission id', mission_id)
            return # TODO raise error

        if ac_id is not None and ac_id in self.uavs:
            send_circle(ac_id, lat, lon, alt, radius, mission_id, insert_mode.value)
        elif ac_id is None:
            for _id in self.uavs:
                send_circle(_id, lat, lon, alt, radius, mission_id, insert_mode.value)

    def add_mission_takeoff(self, mission_id:int, insert_mode:MissionInsert = MissionInsert.APPEND, ac_id:int=None):
        ''' send MISSION_CUSTOM message to a specified uav or all if None
            for the takeoff mission at the current position
        '''
        def send_takeoff(_ac_id, _index, _insert):
            msg = PprzMessage("datalink", "MISSION_CUSTOM")
            msg['ac_id'] = _ac_id
            msg['insert'] = _insert
            msg['duration'] = -1.
            msg['index'] = _index % 256
            msg['type'] = 'TKOFF'
            msg['params'] = [0.]
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)

        if mission_id <= 0 or mission_id > 255:
            print('invalid mission id', mission_id)
            return # TODO raise error

        if ac_id is not None and ac_id in self.uavs:
            send_takeoff(ac_id, mission_id, insert_mode.value)
        elif ac_id is None:
            for _id in self.uavs:
                send_takeoff(_id, mission_id, insert_mode.value)

    def add_mission_land(self, mission_id:int, lat:float, lon:float, height:float, insert_mode:MissionInsert = MissionInsert.APPEND, ac_id:int=None):
        ''' send MISSION_CUSTOM message to a specified uav or all if None
            for the landing mission at the position in lat (deg), lon (deg), height above ref point (m) format
        '''
        def send_land(_ac_id, _lat, _lon, _height, _index, _insert):
            msg = PprzMessage("datalink", "MISSION_CUSTOM")
            msg['ac_id'] = _ac_id
            msg['insert'] = _insert
            msg['duration'] = -1.
            msg['index'] = _index % 256
            msg['type'] = 'LAND'
            msg['params'] = [float(_height), float(_lat), float(_lon), 0., 0.]
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)

        if mission_id <= 0 or mission_id > 255:
            print('invalid mission id', mission_id)
            return # TODO raise error

        if ac_id is not None and ac_id in self.uavs:
            send_land(ac_id, lat, lon, height, mission_id, insert_mode.value)
        elif ac_id is None:
            for _id in self.uavs:
                send_land(_id, lat, lon, height, mission_id, insert_mode.value)



if __name__ == '__main__':
    # run test
    from time import sleep

    try:
        mission = MissionManager(verbose=True)
        print("cohoma_bridge test started")
        sleep(3)
        mission.add_mission_takeoff(1, insert_mode=MissionInsert.REPLACE_ALL)
        sleep(1)
        mission.add_mission_point(2, 43., 1.6, 200.)
        sleep(1)
        mission.add_mission_path(3, [(43.1, 1.61),(43.2, 1.62),(43.3, 1.63),(43.4, 1.64),(43.5, 1.65)], 200.)
        sleep(1)
        mission.add_mission_circle(4, 43.5, 1.65, 200, 80)
        sleep(1)
        mission.add_mission_land(5, 43.5, 1.65, 0.)
        sleep(1)
        mission.start_mission()
        print('UAVs:',mission.uavs)

    except(KeyboardInterrupt,SystemExit):
        print("interrupt cohoma_bridge test")
        mission.closing()
    finally:
        print("closing cohoma_bridge test")
        mission.closing()
