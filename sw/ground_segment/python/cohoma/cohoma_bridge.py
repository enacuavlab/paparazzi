import sys
from os import path, getenv
import math
import numpy as np
from dataclasses import dataclass

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from flight_pkan import FlightPlan


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
    mission_status: list[int] = []
    DL_lost_time: int = 0
    flight_plan: FlightPlan = None
    start_mission_fp_block: int = None


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
                self.start_mission_fp_block = self.uavs[conf.id].flight_plan.get_block('Mission')
            except:
                print("'Mission' block not found in flight plan")
            if self.verbose:
                print(conf)

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=connect_cb)

        ''' bind to messages '''
        self.connect.ivy.subscribe(self.flight_param_cb, PprzMessage("ground", "FLIGHT_PARAM"))

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
            block_id = int(self.msg['cur_block'])
            block = self.uavs[ac_id].flight_plan.get_block(block_id)
            self.uavs[ac_id].FP_block = block.name



    def move_wp(self, ac_id, wp_id, mark):
        ''' move waypoint corresponding to a selected aircraft and mark '''
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = wp_id
        msg['lat'] = mark.lat
        msg['long'] = mark.lon
        msg['alt'] = mark.alt
        self.connect.ivy.send(msg)

    def update_shape(self, mark):
        ''' create or update a shape on the GCS map '''
        self.update_pos_label(mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['linecolor'] = mark.color
        msg['fillcolor'] = mark.color
        msg['opacity'] = 1 # fill color
        msg['shape'] = 0 # circle
        msg['status'] = 0 # create or update
        msg['latarr'] = [int(10**7 * mark.lat),0]
        msg['lonarr'] = [int(10**7 * mark.lon),0]
        msg['radius'] = 2.
        msg['text'] = mark.name
        self.connect.ivy.send(msg)

    def clear_shape(self, mark):
        ''' delete a shape on the GCS map '''
        self.clear_pos_label(mark)
        msg = PprzMessage("ground", "SHAPE")
        msg['id'] = mark.id
        msg['linecolor'] = mark.color
        msg['fillcolor'] = mark.color
        msg['opacity'] = 0 # no fill color
        msg['shape'] = 0 # circle
        msg['status'] = 1 # delete
        msg['latarr'] = [0]
        msg['lonarr'] = [0]
        msg['radius'] = 0.
        msg['text'] = 'NULL'
        self.connect.ivy.send(msg)

