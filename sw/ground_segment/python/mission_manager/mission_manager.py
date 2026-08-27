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
from typing import Dict,Optional
import numpy as np
from enum import Enum
import functools
from threading import Event
import asyncio

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from flight_plan import FlightPlan, Block, Waypoint
from settings import PprzSettingsManager



MAX_RETRY = 5
ACK_TIME = 2

from uav_data import UAVData
    
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
    def __init__(self, ac_id:Optional[int]=None, verbose=False, ivy_interface=None):
        self.verbose = verbose

        self.uav_data:Optional[UAVData] = None
        self.ac_id:Optional[int] = ac_id

        self.events: Dict[int, Event] = {0:Event()}

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=self.connect_cb, ivy=ivy_interface)
        assert self.connect.ivy is not None

        ''' bind to messages '''
        self.connect.ivy.subscribe(self.flight_param_cb, PprzMessage("ground", "FLIGHT_PARAM"))
        self.connect.ivy.subscribe(self.ap_status_cb, PprzMessage("ground", "AP_STATUS"))
        self.connect.ivy.subscribe(self.nav_status_cb, PprzMessage("ground", "NAV_STATUS"))
        self.connect.ivy.subscribe(self.engine_status_cb, PprzMessage("ground", "ENGINE_STATUS"))
        self.connect.ivy.subscribe(self.telemetry_status_cb, PprzMessage("ground", "TELEMETRY_STATUS"))
        self.connect.ivy.subscribe(self.mission_status_cb, PprzMessage("telemetry", "MISSION_STATUS"))
        self.connect.ivy.subscribe(self.ins_ref_cb, PprzMessage("telemetry", "INS_REF"))
        self.connect.ivy.subscribe(self.nav_ref_cb, PprzMessage("telemetry", "NAVIGATION_REF"))
        self.connect.ivy.subscribe(self.airspeed_cb, PprzMessage("telemetry", "AIRSPEED"))
        self.connect.ivy.subscribe(self.windinfo_cb, PprzMessage("ground", "WIND"))

    ###################### Ivy interface management ####################

    def wait_ready(self, timeout: float | None = None):
        """
        Blocking wait until ready to send mission elements.

        Parameters:
            timeout in seconds, or None for infinite.
        
        Return:
            True if ready, False if not ready after the timeout.
        """
        return self.events[0].wait(timeout)

    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()
        
    ##################### Util getter ####################
        
    def get_home(self) -> Optional[Waypoint]:
            assert self.uav_data is not None, "No UAV!"
            if self.uav_data.flight_plan is None:
                print("WARNING: Flight plan is not available")
                return None
            
            home_wp = None
            for wp in self.uav_data.flight_plan.waypoints:
                if wp.name.lower() == "home":
                    home_wp = wp
                    break
            if home_wp is None:
                print("WARNING: Could not find 'HOME' waypoint")
                return None
            
            if home_wp.lat is None or home_wp.lon is None:
                print("WARNING: Latitude or longitude of HOME is not defined")
                return None
            
            if home_wp.alt is None:
                home_wp.alt = float(self.uav_data.flight_plan.alt)
            
            return home_wp
    
    #################### Callbacks ####################
        
    def connect_cb(self, conf:PprzConfig):
        ''' get aircraft config '''
        if self.ac_id is None:
            self.ac_id = int(conf.id)
        
        if self.ac_id == int(conf.id):
            self.uav_data = UAVData.from_conf(conf,self.connect.ivy)
            try:
                self.uav_data.start_mission_fp_block = self.uav_data.flight_plan.get_block('Mission')
                if self.verbose:
                    print("'Mission' block found",self.uav_data.start_mission_fp_block.no)
            except:
                raise Exception("'Mission' block not found in flight plan")
            if self.verbose:
                print(conf)
            self.events[0].set()
    
    def flight_param_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.lat = float(msg['lat'])
            self.uav_data.lon = float(msg['long'])
            self.uav_data.alt = float(msg['alt'])
            self.uav_data.agl = float(msg['agl'])
            self.uav_data.heading = float(msg['heading'])
            speed = float(msg['speed'])
            course = np.deg2rad(float(msg['course']))
            self.uav_data.vnorth = speed * math.cos(course)
            self.uav_data.veast = speed * math.sin(course)
            self.uav_data.vup = float(msg['climb'])
            self.uav_data.gps_tow = int(msg['itow']) / 1000
            
    def ap_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.AP_mode = msg['ap_mode']
            self.uav_data.flight_time = msg['flight_time']

    def nav_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            block_id = int(msg['cur_block'])
            block = self.uav_data.flight_plan.get_block(block_id)
            self.uav_data.FP_block = block.name

    def engine_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.bat_voltage = float(msg['bat'])
            # TODO compute % assuming 2 or 4 cells

    def telemetry_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.datalink_lost_time = int(msg['uplink_lost_time'])
            self.uav_data.time_since_last_msg = float(msg['time_since_last_msg'])

    def mission_status_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.mission_status = [ int(e) for e in msg['index_list'] if int(e) != 0 ]
            # Unblock function waiting for ACK
            for mission_id in self.uav_data.mission_status:
                if mission_id in self.events:
                    self.events[mission_id].set()
            
    def airspeed_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.airspeed       = msg['airspeed']
            self.uav_data.airspeed_sp    = msg['airspeed_sp']
            self.uav_data.groundspeed_sp = msg['groundspeed_sp']
            
    def windinfo_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            wind_dir = msg['dir']
            wind_speed = msg['wspeed']
            self.uav_data.wind_east     = math.sin(math.pi/180.*(wind_dir-180)) * wind_speed
            self.uav_data.wind_north    = math.cos(math.pi/180.*(wind_dir-180)) * wind_speed

    def ins_ref_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.ins_hsml0 = int(msg['hmsl0'])/1000
            
    def nav_ref_cb(self, ac_id, msg):
        if int(ac_id) == self.ac_id and self.uav_data is not None:
            self.uav_data.navref_utm_east = int(msg['utm_east'])
            self.uav_data.navref_utm_north = int(msg['utm_north'])
            self.uav_data.navref_utm_zone = int(msg['utm_zone'])
            self.uav_data.navref_ground_alt = float(msg['ground_alt'])

    #################### Message sending and checking ####################

    def send_message_and_wait(self,msg:PprzMessage, retry:int=MAX_RETRY, ack_time:float=ACK_TIME):
        if self.uav_data is None:
            raise Exception("No UAV!")
        
        assert self.connect.ivy is not None, "No Ivy interface available!"
        
        # keep mission_id in the range [1;255]
        msg['index'] = (msg['index']-1)%255+1
        mission_id = msg['index']
        
        self.connect.ivy.send(msg)
        # event to be notified as soon as the element is ACK
        e = self.events.setdefault(mission_id, Event())
        if self.verbose:
            print(msg)
        if e.wait(ack_time):
            del self.events[mission_id]
            return

        for _ in range(retry-1):
            self.connect.ivy.send(msg)
            if self.verbose:
                print(msg)
            # wait a bit to receive the ACK
            if e.wait(ack_time):
                del self.events[mission_id]
                return
            
        del self.events[mission_id]
        raise Exception(f"Mission element no {mission_id} not ACKed in {retry*ack_time:.1f}s by {self.uav_data.ac_id}!")
    
    def send_message(self,msg:PprzMessage):
        if self.uav_data is None:
            raise Exception("No UAV!")
        
        assert self.connect.ivy is not None, "No Ivy interface available!"
        
        # keep mission_id in the range [1;255]
        msg['index'] = (msg['index']-1)%255+1
        mission_id = msg['index']
        
        self.connect.ivy.send(msg)
        # event to be notified as soon as the element is ACK
        self.events.setdefault(mission_id, Event())
        
        if self.verbose:
            print(msg)

        
    def check_message(self,msg:PprzMessage|int) -> bool:
        if isinstance(msg,PprzMessage):
            mission_id = msg['index']
        else:
            mission_id = msg
            
        mission_id = (mission_id-1)%255+1
        
        return self.events[mission_id].is_set()
    
    def message_check_and_retry(self,msg:PprzMessage, retry:int=MAX_RETRY, ack_time:float=ACK_TIME) -> bool:
        if self.uav_data is None:
            raise Exception("No UAV!")
        
        assert self.connect.ivy is not None, "No Ivy interface available!"
        
        # keep mission_id in the range [1;255]
        msg['index'] = (msg['index']-1)%255+1
        mission_id = msg['index']
        
        for _ in range(retry):
            if self.events[mission_id].wait(ack_time):
                del self.events[mission_id]
                return True
            else:
                self.connect.ivy.send(msg)
                if self.verbose:
                    print(msg)
                    
        if self.events[mission_id].wait(ack_time):
            del self.events[mission_id]
            return True
        else:
            return False
    
    ##################### Mission management ####################
    
    def start_mission(self):
        ''' enter Mission flight block for selected UAV or all if None'''
        assert self.uav_data is not None, "No UAV!"
        msg = self.make_start_mission()
        self.connect.ivy.send(msg)
        if self.verbose:
            print(msg)
            
    def make_start_mission(self) -> PprzMessage:
        ''' create message to enter Mission flight block for selected UAV or all if None'''
        assert self.uav_data is not None, "No UAV!"
        msg = PprzMessage("ground", "JUMP_TO_BLOCK")
        msg['ac_id'] = self.ac_id
        msg['block_id'] = int(self.uav_data.start_mission_fp_block.no)
        return msg
    
    def next_mission(self):
        msg = make_next_mission_msg(self.ac_id)
        self.connect.ivy.send(msg)
        if self.verbose:
            print(msg)
    
    def end_mission(self):
        msg = make_end_mission_msg(self.ac_id)
        self.connect.ivy.send(msg)
        if self.verbose:
            print(msg)

    ###################### Elementary missions shorthands ####################

    def go_home(self, mission_id:int, insert:MissionInsert = MissionInsert.REPLACE_CURRENT) -> bool:
        if self.ac_id is not None and self.uav_data is not None and self.uav_data.flight_plan is not None:
            home_wp = self.get_home()
            if home_wp is None:
                return False
            else:
                msg = make_mission_point(self.ac_id, mission_id, lat=home_wp.lat,lon=home_wp.lon,alt=home_wp.alt,insert_mode=insert)
                self.send_message_and_wait(msg)
                return True            
        else:
            return False
        
    def circle_home(self, mission_id:int, radius:float, insert:MissionInsert = MissionInsert.REPLACE_CURRENT) -> bool:
        if self.ac_id is not None and self.uav_data is not None and self.uav_data.flight_plan is not None:
            home_wp = self.get_home()
            if home_wp is None:
                return False
            else:
                msg = make_mission_circle(self.ac_id, mission_id, lat=home_wp.lat,lon=home_wp.lon,alt=home_wp.alt, radius=radius, insert_mode=insert)
                self.send_message_and_wait(msg)
                return True            
        else:
            return False
        
    def circle_here_msg(self, mission_id:int, radius:float, duration:float = -1., insert:MissionInsert = MissionInsert.REPLACE_CURRENT) -> PprzMessage:
        if self.uav_data is not None and self.ac_id is not None:
            msg = make_mission_circle(self.ac_id,mission_id, lat=self.uav_data.lat,lon=self.uav_data.lon,alt=self.uav_data.alt, radius=radius, duration=duration, insert_mode=insert)
            return msg
        else:
            raise Exception("No UAV!")
        
    def circle_here(self, mission_id:int, radius:float, duration:float = -1., insert:MissionInsert = MissionInsert.REPLACE_CURRENT) -> bool:
        if self.uav_data is not None and self.ac_id is not None:
            self.send_message_and_wait(self.circle_here_msg(mission_id, radius, duration, insert))
            return True            
        else:
            return False
        
######################################## Mission message creation helpers ########################################
    
def make_next_mission_msg(ac_id:int) -> PprzMessage:
    msg = PprzMessage("datalink", "NEXT_MISSION")
    msg['ac_id'] = ac_id
    return msg

def make_end_mission_msg(ac_id:int) -> PprzMessage:
    msg = PprzMessage("datalink", "END_MISSION")
    msg['ac_id'] = ac_id
    return msg
    
def make_mission_point(ac_id:int, mission_id: int, lat: float, lon: float, alt: float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    msg = PprzMessage("datalink", "MISSION_GOTO_WP_LLA")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['wp_lat'] = int(lat * 1e7)
    msg['wp_lon'] = int(lon * 1e7)
    msg['wp_alt'] = int(alt * 1e3)
    return msg

def make_mission_local_point(ac_id:int, mission_id: int, east: float, north: float, alt: float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    msg = PprzMessage("datalink", "MISSION_GOTO_WP")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['wp_east'] = east
    msg['wp_north'] = north
    msg['wp_alt'] = alt
    return msg


def make_mission_path(ac_id:int, mission_id:int, path:list[tuple[float,float]], alt:float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_PATH_LLA message to a specified uav or all if None
        path is described by a list of at most 5 points in (lat (deg), lon (deg)) + alt amsl (m) format
    '''
    assert len(path) <= 5
    msg = PprzMessage("datalink", "MISSION_PATH_LLA")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['nb'] = min(len(path), 5)
    print(path, len(path),msg['nb'])
    msg['index'] = mission_id
    msg['path_alt'] = int(alt * 1e3)
    for i in range(msg['nb']):
        lat, lon = path[i]
        msg['point_lat_'+str(i+1)] = int(lat * 1e7)
        msg['point_lon_'+str(i+1)] = int(lon * 1e7)
    return msg

def make_mission_local_path(ac_id:int, mission_id:int, path:list[tuple[float,float]], alt:float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_PATH message to a specified uav or all if None
        path is described by a list of at most 5 points in (east (m), north (m)) + alt amsl (m) format with respect to HOME
    '''
    assert len(path) <= 5
    msg = PprzMessage("datalink", "MISSION_PATH")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['nb'] = min(len(path), 5)
    print(path, len(path),msg['nb'])
    msg['index'] = mission_id
    msg['path_alt'] = alt
    for i in range(msg['nb']):
        east, north = path[i]
        msg['point_east_'+str(i+1)] = east
        msg['point_north_'+str(i+1)] = north
    return msg

def make_mission_circle(ac_id:int, mission_id:int, lat:float, lon:float, alt:float, radius:float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_CIRCLE_LLA message to a specified uav or all if None
        circle is described by a lat (m), lon (m), alt amsl (m) format and radius (m)
    '''
    msg = PprzMessage("datalink", "MISSION_CIRCLE_LLA")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['center_lat'] = int(lat * 1e7)
    msg['center_lon'] = int(lon * 1e7)
    msg['center_alt'] = int(alt * 1e3)
    msg['radius'] = radius
    return msg

def make_mission_poles(ac_id:int, mission_id:int, lat1:float, lon1:float, lat2:float, lon2:float, height:float, radius:float, duration:float = -1., nb_laps:int = -1, insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_CUSTOM message to a specified uav or all if None
        for the navigation between two poles at position lat (deg), lon (deg), height above ref point (m) and radius (m) format
    '''
    msg = PprzMessage("datalink", "MISSION_CUSTOM")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['type'] = 'POLES'
    msg['params'] = [float(lat1), float(lon1), float(lat2), float(lon2), float(height), float(radius), 1., float(nb_laps)] # fixed margin of 1.
    return msg

def make_mission_takeoff(ac_id:int, mission_id:int, height:float = -1., duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_CUSTOM message to a specified uav or all if None
        for the takeoff mission at the current position
    '''
    msg = PprzMessage("datalink", "MISSION_CUSTOM")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['type'] = 'TKOFF'
    msg['params'] = [height]
    return msg

def make_mission_land_rotorcraft(ac_id:int, mission_id:int, lat:float, lon:float, height:float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_CUSTOM message to a specified uav or all if ac_id is None
        for landing at the position in lat (deg), lon (deg), height above ref point (m) format
    '''
    msg = PprzMessage("datalink", "MISSION_CUSTOM")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['type'] = 'LAND'
    msg['params'] = [float(height), float(lat), float(lon), 0., 0.]
    return msg

def make_mission_land_fw_here(ac_id:int, mission_id:int, td_alt:float, radius:float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_CUSTOM message to a specified uav or all if ac_id is None
        for landing at the current position for fixed wing aircraft with touch down altitude (m) and spiral radius (m)
    '''
    msg = PprzMessage("datalink", "MISSION_CUSTOM")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['type'] = 'LAND'
    msg['params'] = [float(td_alt), float(radius)]
    return msg

def make_mission_land_fw(ac_id:int, mission_id:int, lat:float, lon:float, td_alt:float, direction:float, dist:float, radius:float, duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    ''' send MISSION_CUSTOM message to a specified uav or all if ac_id is None
        for fixed wing landing with touch down altitude (m) at lat (deg), lon (deg),
        from direction (deg), distance (m), with final turn radius (m) for fixed wing aircraft
    '''
    msg = PprzMessage("datalink", "MISSION_CUSTOM")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['type'] = 'LAND'
    msg['params'] = [float(td_alt), float(lat), float(lon), float(direction), float(dist), float(radius)]
    return msg

def make_mission_custom(ac_id:int,mission_id:int, name:str, params:list[float], duration:float = -1., insert_mode:MissionInsert = MissionInsert.APPEND) -> PprzMessage:
    assert len(name) <= 5
    msg = PprzMessage("datalink", "MISSION_CUSTOM")
    msg['ac_id'] = ac_id
    msg['insert'] = insert_mode.value
    msg['duration'] = duration
    msg['index'] = mission_id
    msg['type'] = name
    msg['params'] = params.copy()
    return msg

######################################## Async manager message sending and checking ########################################

async def send_and_ack_msg(mng:MissionManager, msg:PprzMessage, retry:int=MAX_RETRY, ack_time:float=ACK_TIME):
    if mng.uav_data is None:
        raise Exception("No UAV!")
    
    assert mng.connect.ivy is not None, "No Ivy interface available!"
    
    # keep mission_id in the range [1;255]
    msg['index'] = (msg['index']-1)%255+1
    mission_id = msg['index']
        
    mng.connect.ivy.send(msg)
    # event to be notified as soon as the element is ACK
    e = mng.events.setdefault(mission_id, Event())
    if mng.verbose:
        print(msg)
        
    await asyncio.sleep(ack_time)
    if e.is_set():
        del mng.events[mission_id]
        return

    for _ in range(retry-1):
        mng.connect.ivy.send(msg)
        if mng.verbose:
            print(msg)
        # wait a bit to receive the ACK
        await asyncio.sleep(ack_time)
        if e.is_set():
            del mng.events[mission_id]
            return
            
    del mng.events[mission_id]
    raise TimeoutError(f"Mission element no {mission_id} not ACKed in {retry*ack_time:.1f}s by {mng.uav_data.ac_id}!")

async def send_and_ack_msgs(mng:MissionManager, msgs:list[PprzMessage], retry:int=MAX_RETRY, ack_time:float=ACK_TIME):
    for msg in msgs:
        await send_and_ack_msg(mng, msg, retry, ack_time)

if __name__ == '__main__':
    # run test
    from time import sleep

    LANDPAD = (48.8652, 1.89285)
    P1 = (48.8653, 1.89503)
    P2 = (48.8651, 1.89862)
    P3 = (48.8654, 1.89621)

    mission = MissionManager(verbose=True)
    try:
        print("cohoma_bridge test started")
        mission.wait_ready()
        mission.send_message_and_wait(mission.make_mission_takeoff(1, height=40., insert_mode=MissionInsert.REPLACE_ALL))
        mission.send_message_and_wait(mission.make_mission_path(2, path=[LANDPAD, P2], alt=120))
        mission.send_message_and_wait(mission.make_mission_poles(3, lat1=P2[0], lon1=P2[1], lat2=P3[0], lon2=P3[1], height=40., radius=60., nb_laps=2))
        mission.send_message_and_wait(mission.make_mission_path(4, path=[P2, LANDPAD] , alt=120))
        mission.send_message_and_wait(mission.make_mission_point(5, lat=48.866, lon=1.899, alt=150.))
        mission.send_message_and_wait(mission.make_mission_path(3, path=[(48.865, 1.899),(48.865, 1.898),(48.866, 1.898),(48.866, 1.897),(48.865, 1.897)], alt=150.))
        mission.send_message_and_wait(mission.make_mission_circle(4, lat=48.865, lon=1.898, alt=150, radius=-80, duration=20))
        mission.send_message_and_wait(mission.make_mission_land(6, lat=LANDPAD[0], lon=LANDPAD[1], height=0.))
        mission.start_mission()
        print('UAV:',mission.uav_data.name)

    except(KeyboardInterrupt,SystemExit):
        print("interrupt cohoma_bridge test")
        mission.closing()
    finally:
        print("closing cohoma_bridge test")
        mission.closing()
