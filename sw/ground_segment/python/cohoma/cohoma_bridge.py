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
from typing import Dict
import numpy as np
from dataclasses import dataclass
from enum import Enum
import time
import functools
from threading import Event


# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from flight_plan import FlightPlan, Block


MAX_RETRY = 3
ACK_TIME = 0.5

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
    start_mission_fp_block: Block = None

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
    def __init__(self, ac_id=None, verbose=False):
        self.verbose = verbose

        self.uav_data = None
        self.ac_id = ac_id

        self.events: Dict[int, Event] = {0:Event()}

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=self.connect_cb)

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
    
        
    def connect_cb(self, conf):
        ''' get aircraft config '''
        if self.ac_id is None:
            self.ac_id = int(conf.id)
        
        if self.ac_id == int(conf.id):
            self.uav_data = UAVData(conf.id, conf.name)
            self.uav_data.mission_status = []       # TODO replace that by the correct dataclass initializer
            self.uav_data.flight_plan = FlightPlan.parse(conf.flight_plan)
            try:
                self.uav_data.start_mission_fp_block = self.uav_data.flight_plan.get_block('Mission')
                if self.verbose:
                    print("'Mission' block found",self.uav_data.start_mission_fp_block.no)
            except:
                raise Exception("'Mission' block not found in flight plan")
            if self.verbose:
                print(conf)
            self.events[0].set()
    
    def wait_ready(self, timeout: float | None = None):
        """
        Blocking wait until ready to send mission elements.

        Parameters:
            timeout in seconds, or None for infinite.
        
        Return:
            True if ready, False if not ready after the timeout.
        """
        return self.events[0].wait(timeout)

    def flight_param_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            self.uav_data.lat = float(msg['lat'])
            self.uav_data.lon = float(msg['long'])
            self.uav_data.alt = float(msg['alt'])
            self.uav_data.height = float(msg['agl']) # FIXME height or AGL ?
            self.uav_data.heading = float(msg['heading'])
            speed = float(msg['speed'])
            course = np.deg2rad(float(msg['course']))
            self.uav_data.vnorth = speed * math.cos(course)
            self.uav_data.veast = speed * math.sin(course)
            self.uav_data.vup = float(msg['climb'])
            self.uav_data.gps_tow = int(msg['itow']) / 1000 # FIXME s or ms ?

    def ap_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            self.uav_data.AP_mode = msg['ap_mode']
            self.uav_data.flight_time = msg['flight_time']

    def nav_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            block_id = int(msg['cur_block'])
            block = self.uav_data.flight_plan.get_block(block_id)
            self.uav_data.FP_block = block.name

    def engine_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            self.uav_data.bat_voltage = float(msg['bat'])
            # TODO compute % assuming 2 or 4 cells

    def telemetry_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            self.uav_data.datalink_lost_time = int(msg['uplink_lost_time'])

    def mission_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id:
            self.uav_data.mission_status = [ int(e) for e in msg['index_list'] if int(e) != 0 ]
            # Unblock function waiting for ACK
            for mission_id in self.uav_data.mission_status:
                if mission_id in self.events:
                    self.events[mission_id].set()

    def send_mission_element(forge_mission_msg):
        @functools.wraps(forge_mission_msg)
        def wrapper(self:'MissionManager', mission_id: int, **kwargs):
            if self.uav_data is None:
                return False  # no AC

            # keep mission_id in the range [1;255]
            mission_id = (mission_id-1)%255 + 1

            for _ in range(MAX_RETRY):
                msg = forge_mission_msg(self, mission_id=mission_id, **kwargs)
                self.connect.ivy.send(msg)
                if self.verbose:
                    print(msg)
                # event to be notified as soon as the element is ACK
                e = self.events.setdefault(mission_id, Event())
                e.clear()
                # wait a bit to receive the ACK
                if e.wait(ACK_TIME):
                    del self.events[mission_id]
                    return True
            del self.events[mission_id] 
            if self.verbose:
                print("Fail to add mission element")
            return False

        return wrapper
    
    def start_mission(self):
        ''' enter Mission flight block for selected UAV or all if None'''
        msg = PprzMessage("ground", "JUMP_TO_BLOCK")
        msg['ac_id'] = self.ac_id
        msg['block_id'] = int(self.uav_data.start_mission_fp_block.no)
        self.connect.ivy.send(msg)
        if self.verbose:
            print(msg)

    @send_mission_element
    def add_mission_point(self, lat: float, lon: float, alt: float, mission_id: int, insert:MissionInsert = MissionInsert.APPEND):
        msg = PprzMessage("datalink", "MISSION_GOTO_WP_LLA")
        msg['ac_id'] = self.ac_id
        msg['insert'] = insert.value
        msg['duration'] = -1.
        msg['index'] = mission_id
        msg['wp_lat'] = int(lat * 1e7)
        msg['wp_lon'] = int(lon * 1e7)
        msg['wp_alt'] = int(alt * 1e3)
        return msg

    @send_mission_element
    def add_mission_path(self, mission_id:int, path:list[(float,float)], alt:float, insert_mode:MissionInsert = MissionInsert.APPEND):
        ''' send MISSION_PATH_LLA message to a specified uav or all if None
            path is described by a list of points in (lat (deg), lon (deg)) + alt amsl (m) format
        '''
        msg = PprzMessage("datalink", "MISSION_PATH_LLA")
        msg['ac_id'] = self.ac_id
        msg['insert'] = insert_mode.value
        msg['duration'] = -1.
        msg['nb'] = max(len(path), 5)
        msg['index'] = mission_id
        msg['path_alt'] = int(alt * 1e3)
        for i in range(msg['nb']):
            lat, lon = path[i]
            msg['point_lat_'+str(i+1)] = int(lat * 1e7)
            msg['point_lon_'+str(i+1)] = int(lon * 1e7)
        return msg

    @send_mission_element
    def add_mission_circle(self, mission_id:int, lat:float, lon:float, alt:float, radius:float, insert_mode:MissionInsert = MissionInsert.APPEND):
        ''' send MISSION_CIRCLE_LLA message to a specified uav or all if None
            circle is described by a lat (m), lon (m), alt amsl (m) format and radius (m)
        '''
        msg = PprzMessage("datalink", "MISSION_CIRCLE_LLA")
        msg['ac_id'] = self.ac_id
        msg['insert'] = insert_mode.value
        msg['duration'] = -1.
        msg['index'] = mission_id
        msg['center_lat'] = int(lat * 1e7)
        msg['center_lon'] = int(lon * 1e7)
        msg['center_alt'] = int(alt * 1e3)
        msg['radius'] = radius
        return msg

    @send_mission_element
    def add_mission_poles(self, mission_id:int, lat1:float, lon1:float, lat2:float, lon2:float, height:float, radius:float, insert_mode:MissionInsert = MissionInsert.APPEND):
        ''' send MISSION_CUSTOM message to a specified uav or all if None
            for the navigation between two poles at position lat (deg), lon (deg), height above ref point (m) and radius (m) format
        '''
        msg = PprzMessage("datalink", "MISSION_CUSTOM")
        msg['ac_id'] = self.ac_id
        msg['insert'] = insert_mode.value
        msg['duration'] = -1.
        msg['index'] = mission_id
        msg['type'] = 'POLES'
        msg['params'] = [float(lat1), float(lon1), float(lat2), float(lon2), float(height), float(radius), 1.] # fixed margin of 1.
        return msg

    @send_mission_element
    def add_mission_takeoff(self, mission_id:int, insert_mode:MissionInsert = MissionInsert.APPEND):
        ''' send MISSION_CUSTOM message to a specified uav or all if None
            for the takeoff mission at the current position
        '''
        msg = PprzMessage("datalink", "MISSION_CUSTOM")
        msg['ac_id'] = self.ac_id
        msg['insert'] = insert_mode.value
        msg['duration'] = -1.
        msg['index'] = mission_id
        msg['type'] = 'TKOFF'
        msg['params'] = [0.]
        return msg

    @send_mission_element
    def add_mission_land(self, mission_id:int, lat:float, lon:float, height:float, insert_mode:MissionInsert = MissionInsert.APPEND):
        ''' send MISSION_CUSTOM message to a specified uav or all if None
            for the landing mission at the position in lat (deg), lon (deg), height above ref point (m) format
        '''
        msg = PprzMessage("datalink", "MISSION_CUSTOM")
        msg['ac_id'] = self.ac_id
        msg['insert'] = insert_mode.value
        msg['duration'] = -1.
        msg['index'] = mission_id
        msg['type'] = 'LAND'
        msg['params'] = [float(height), float(lat), float(lon), 0., 0.]
        return msg
    


if __name__ == '__main__':
    # run test
    from time import sleep

    try:
        mission = MissionManager(verbose=True)
        print("cohoma_bridge test started")
        mission.wait_ready()
        mission.add_mission_takeoff(1, insert_mode=MissionInsert.REPLACE_ALL)
        mission.add_mission_point(2, lat=43., lon=1.6, alt=200.)
        mission.add_mission_path(3, path=[(43.1, 1.61),(43.2, 1.62),(43.3, 1.63),(43.4, 1.64),(43.5, 1.65)], alt=200.)
        mission.add_mission_circle(4, lat=43.5, lon=1.65, alt=200, radius=80)
        mission.add_mission_land(5, lat=43.5, lon=1.65, height=0.)
        mission.start_mission()
        print('UAV:',mission.uav_data.name)

    except(KeyboardInterrupt,SystemExit):
        print("interrupt cohoma_bridge test")
        mission.closing()
    finally:
        print("closing cohoma_bridge test")
        mission.closing()
