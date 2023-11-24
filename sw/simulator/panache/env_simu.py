#!/usr/bin/env python3

# exemple: ./env_simu.py -ts

import sys,io
from os import path, getenv
import typing
import time
import pyproj
import struct
import panacheur
import argparse
import json

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)),'../../..')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.generated.datalink import PprzMessage_PAYLOAD_COMMAND
from pprzlink.generated.ground import PprzMessage_WORLD_ENV
import sampleParser


PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)),'../../..')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/lib/python")


import pprz_connect

# WORLD_ENV_REQ
# PAYLOAD_COMMAND

spray_file = "spray.bin_n1_d1_t1.nc"
swift_file = "swift_merged.nc"


class EnvSimu:
    def __init__(self, args):
        self.ivy = IvyMessagesInterface("Glover",verbose=True)
        self.ac_ids = []
        self.pprzconnect = pprz_connect.PprzConnect(notify=self.update_config, ivy=self.ivy)
        self.panache = panacheur.Panacheur(args.spray, args.swift, 
                                           time_offset=int(args.time_offset),
                                           time_scale=args.time_scale,
                                           wind_scale=args.wind_scale)
        # lambert 93 CRS
        crs_lamb93 = pyproj.CRS.from_epsg(2154)
        self.proj_lamb93 = pyproj.Proj(crs_lamb93)
        self.ivy.subscribe_request_answerer(self.world_env_cb, "WORLD_ENV")
        self.start_time = time.time()
        self.request_stop = False
        
        self.writer:typing.Optional[sampleParser.SampleWriter] = None
        if args.save is not None:
            _,ext = path.splitext(args.save)
            if ext == '.pkl':
                self.writer = sampleParser.SampleWriter_pickle(args.save)
            else:
                self.writer = sampleParser.SampleWriter_TopoJSON(args.save)
            self.writer:sampleParser.SampleWriter
        

    def __enter__(self):
        self.ivy.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def stop(self):
        if self.writer is not None:
            self.writer.close()
        self.ivy.shutdown()
    
    def update_config(self, config):
        self.ac_ids.append(config.id)
        print(config.id)
    
    def to_lambert(self, lat, lon):
        x, y = self.proj_lamb93(lon, lat)
        return x, y

    def world_env_cb(self, sender, request):
        x, y = self.to_lambert(request["lat"], request["long"])
        alt = float(request["alt"])
        coords = (time.time(), alt, y, x)
        print(f"""({request["lat"]: 2.6f},{request["long"]: 2.6f},{alt: 4.2f}): """,end=' ')
        self.send_concentration(coords)
        
        
        try:
            u, v, w = self.panache.get_vars_safe(("U", "V", "W"), coords)
            # print(u[0], v[0], w[0])
            msg = PprzMessage_WORLD_ENV()
            msg["wind_east"] = u[0]
            msg["wind_north"] = v[0]
            msg["wind_up"] = w[0]
            # msg["ir_contrast"] = 0
            msg["time_scale"] = 1.0
            msg["gps_availability"] = 1
            #print(msg)
            return msg
        except panacheur.EndOfTime:
            print("end of time, stopping...")
            self.request_stop = True
    
    def send_concentration(self, coords):
        try:
            pm10, = self.panache.get_vars_safe(("PM10",), coords)
            print(f"{pm10[0]:e}")
            msg = PprzMessage_PAYLOAD_COMMAND()
            msg.command_ = [b for b in struct.pack("f", pm10[0])]
            # print(msg)
            for ac_id in self.ac_ids:
                msg["ac_id"] = ac_id
                self.ivy.send(msg)
                
            if self.writer is not None:
                self.writer.write(sampleParser.SamplePoint(coords[-1],coords[-2],coords[-3],coords[-4],pm10[0]))
        except panacheur.EndOfTime:
            print("end of time...")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                    prog = 'Glover',
                    description = 'Panache env simu')
    parser.add_argument('-w', '--swift', default=swift_file)
    parser.add_argument('-p', '--spray', default=spray_file)
    parser.add_argument('-ts', '--time-scale', default=1, type=float)
    parser.add_argument('-to', '--time-offset', default=0, type=int,help="Time offset (in seconds), with regards to simulation start.\
        The 'now' timestamp is always included.")
    parser.add_argument('-ws','--wind-scale',default=1, type=float, help="Rescale the wind provided by the simulation.")
    parser.add_argument('--save',dest='save',default=None,type=str,
                        help="If set, save all concentration measurement in the given file.\
                            If the file has extension '.pkl', use the Python pickle protocol for saving.\
                            Otherwise, uses TopoJSON format.")
    args = parser.parse_args()

    with EnvSimu(args) as p:
        while True:#p.request_stop:
            time.sleep(0.2)

