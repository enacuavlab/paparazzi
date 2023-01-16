#!/usr/bin/python3

# exemple: ./env_simu.py -ts

import sys
from os import path, getenv
import time
import pyproj
import struct
import panacheur
import argparse

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.dirname(path.abspath(__file__))))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
import pprz_connect

# WORLD_ENV_REQ
# PAYLOAD_COMMAND

spray_file = "spray.bin_n1_d1_t1.nc"
swift_file = "swift_merged.nc"


class EnvSimu:
    def __init__(self, args):
        self.ivy = IvyMessagesInterface("Glover")
        self.ac_ids = []
        self.pprzconnect = pprz_connect.PprzConnect(notify=self.update_config, ivy=self.ivy)
        self.panache = panacheur.Panacheur(args.spray, args.swift, time_scale=args.time_scale)
        # lambert 93 CRS
        crs_lamb93 = pyproj.CRS.from_epsg(2154)
        self.proj_lamb93 = pyproj.Proj(crs_lamb93)
        self.ivy.subscribe_request_answerer(self.world_env_cb, "WORLD_ENV")
        self.start_time = time.time()
        self.request_stop = False

    def __enter__(self):
        self.ivy.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def stop(self):
        self.ivy.shutdown()
    
    def update_config(self, config):
        self.ac_ids.append(config.id)
    
    def to_lambert(self, lat, lon):
        x, y = self.proj_lamb93(lon, lat)
        return x, y

    def world_env_cb(self, sender, request):
        x, y = self.to_lambert(request["lat"], request["long"])
        alt = float(request["alt"])
        coords = (time.time(), alt, y, x)
        self.send_concentration(coords)
        try:
            u, v, w = self.panache.get_vars_safe(("U", "V", "W"), coords)
            # print(u[0], v[0], w[0])
            msg = PprzMessage("ground", "WORLD_ENV")
            msg["wind_east"] = u[0]
            msg["wind_north"] = v[0]
            msg["wind_up"] = w[0]
            # msg["ir_contrast"] = 0
            msg["time_scale"] = 1.0
            msg["gps_availability"] = 1
            # print(msg)
            return msg
        except panacheur.EndOfTime:
            print("end of time, stopping...")
            self.request_stop = True
    
    def send_concentration(self, coords):
        try:
            pm10, = self.panache.get_vars_safe(("PM10",), coords)
            print(f"{pm10[0]:e}")
            msg = PprzMessage("datalink", "PAYLOAD_COMMAND")
            msg["command"] = struct.pack("f", pm10[0])
            for ac_id in self.ac_ids:
                msg["ac_id"] = ac_id
                self.ivy.send(msg)
        except panacheur.EndOfTime:
            print("end of time...")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                    prog = 'Glover',
                    description = 'Panache env simu')
    parser.add_argument('-w', '--swift', default=swift_file)
    parser.add_argument('-p', '--spray', default=spray_file)
    parser.add_argument('-ts', '--time-scale', default=1, type=float)
    args = parser.parse_args()

    with EnvSimu(args) as p:
        while p.request_stop:
            time.sleep(0.2)

