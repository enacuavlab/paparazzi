#!/usr/bin/env python3
#
# Copyright (C) 2023 Mael Feurgard <mael.feurgard@laas.fr>
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

'''
Error measurements regarding guidance vector fields (gvf)
'''
import sys
import numpy as np
import json
import typing
from dataclasses import dataclass
import os,time
from abc import ABC,abstractmethod
import itertools

from time import sleep
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.generated import telemetry

from animatedGVF.pprzInterface import AC_DataCollector

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

############### Individual error log ###############

@dataclass
class ErrorLog():
    ac_id:int
    errors:typing.List[typing.List[float]]
    ws:typing.List[float]
    timestamps:typing.List[float]
    
    def register_msg(self, msg:telemetry.PprzMessage_GVF_PARAMETRIC):
        w = float(msg.config_[0])*float(msg.config_[2])
        error = [float(e) for e in msg.phi_]
        timestamp = float(msg.config_[1])
        
        self.errors.append(error)
        self.ws.append(w)
        self.timestamps.append(timestamp)
        
    def to_numpy(self) -> typing.Tuple[np.ndarray,np.ndarray,np.ndarray]:
        
        np_errors = np.asarray(self.errors)
        np_ws = np.asarray(self.ws)
        np_timestamps = np.asarray(self.timestamps)
        
        return np_errors,np_ws,np_timestamps

############### Coordination error log ###############


@dataclass
class _Neighbor_Coord_Status():
    ac_id:int
    w:float
    w_dot:float
    delta_w:float
    delta_t:int

@dataclass 
class CoordLog():
    ac_id:int
    neighbor_list:typing.List[typing.List[int]]
    status_list:typing.List[typing.Dict[int,typing.List]]
    error_list:typing.List[typing.List[float]]
    timestamps_list:typing.List[int]
        
    
    def register_msg(self, msg:telemetry.PprzMessage_GVF_PAR_COORD):
        
        # Pack status table into packs of 5
        packed_table = [msg.table_[i:i+5] for i in range(0,len(msg.table_),5)]
        packed_coord_status = dict()
        for l in packed_table:
            packed_coord_status[int(float(l[0]))] = _Neighbor_Coord_Status(int(float(l[0])),
                                                                    float(l[1]),
                                                                    float(l[2]),
                                                                    float(l[3]),
                                                                    int(float(l[4]))) 
        
        self.neighbor_list.append([int(float(l[0])) for l in packed_table])
        self.status_list.append(packed_coord_status)
        self.error_list.append([float(e) for e in msg.errors_])
        self.timestamps_list.append(int(float(msg.timestamp_)))
        
    def to_numpy(self) -> typing.Tuple[np.ndarray,
                                       typing.List[typing.List[int]],
                                       typing.Dict[int,np.ndarray],
                                       typing.Dict[int,np.ndarray],
                                       typing.Dict[int,np.ndarray],
                                       typing.Dict[int,np.ndarray],
                                       np.ndarray]:
        
        
        np_errors = np.asarray(self.error_list)
        np_neighbors = np.asarray(self.neighbor_list)
        set_neighbors = [set(l) for l in self.neighbor_list]
        all_neighbors = set.union(*set_neighbors)
        all_neighbors.discard(-1)
        
        np_ws = dict()
        np_w_dots = dict()
        np_delta_ws = dict()
        np_delta_ts = dict()
        
        for n in all_neighbors:
            n_status = [d[n] if n in d.keys() else _Neighbor_Coord_Status(n,np.nan,np.nan,np.nan,np.nan) for d in self.status_list]
            n_ws = np.asarray(s.w for s in n_status)
            n_w_dots = np.asarray(s.w_dot for s in n_status)
            n_delta_ws = np.asarray(s.delta_w for s in n_status)
            n_delta_ts = np.asarray(s.delta_t for s in n_status)
            
            np_ws[n] = n_ws
            np_w_dots[n] = n_w_dots
            np_delta_ws[n] = n_delta_ws
            np_delta_ts[n] = n_delta_ts
            
        return np_errors,np_neighbors,np_ws,np_w_dots,np_delta_ws,np_delta_ts, np.asarray(self.timestamps_list)
            
############### Logging ###############

def color_correction(color:str):
    if type(color) == str:
        if color[0] == '#' and len(color) > len('#0f0f0f80'):
            # If the color uses more than 8 bits per color channel...
            # ... Transform it into a RGB float tuple (naively)
            colorstring = color[1:]
            hexlen = len(colorstring)//3
            color = tuple(int(colorstring[i*hexlen:(i+1)*hexlen],16)/(16**hexlen) for i in range(3))
    return color

class ErrorLogger():
    def __init__(self,ac_ids:typing.List[int], ivy: typing.Union[str, IvyMessagesInterface], coordination:bool=False,
                 export:bool=False) -> None:
        self.ac_ids = ac_ids
        
        
        # Setup Ivy interface
        if isinstance(ivy,IvyMessagesInterface):
            self._ivy_interface = ivy
        else:
            self._ivy_interface = IvyMessagesInterface(ivy,True)
            
            
        # Init logs
        self.logs:typing.Dict[int,ErrorLog] = dict()
        for id in ac_ids:
            self.logs[id] = ErrorLog(id,[],[],[])
            
        # Setup callbacks
        def gvf_par_cb(ac_id, msg:telemetry.PprzMessage_GVF_PARAMETRIC):
            if ac_id in self.ac_ids:# and msg.name == "GVF_PARAMETRIC":
                self.logs[ac_id].register_msg(msg)
        self._ivy_interface.subscribe(gvf_par_cb, telemetry.PprzMessage_GVF_PARAMETRIC())  
        
        
        # Again, but for coordination (if requested)
        self.coordination = coordination
        if coordination:
            self.coord_logs:typing.Dict[int,CoordLog] = dict() 
            for id in self.ac_ids:
                self.coord_logs[id] = CoordLog(id,[],[],[],[])
            
            def gvf_coord_cb(ac_id,msg:telemetry.PprzMessage_GVF_PAR_COORD):
                if ac_id in self.ac_ids:
                    self.coord_logs[ac_id].register_msg(msg)
            self._ivy_interface.subscribe(gvf_coord_cb, telemetry.PprzMessage_GVF_PAR_COORD())
            
        
        # Init data collector
        self.collector = AC_DataCollector(ac_ids,self._ivy_interface,True)
        
        # Export data as .npz
        self.do_export = export
        
        
    def plot(self):
        fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
        ax1:Axes
        ax2:Axes
        
        title = input("Input figure title:")
        
        fig.suptitle(title)
        
        ax1.set_xlabel("Time ellapsed (s)")
        ax1.set_ylabel("Distance (m)")
        ax1.grid(True,'major','x')
        
        ax2.set_xlabel("Time ellapsed (s)")
        ax2.set_ylabel("Virtual coordinate (a.u.)")
        ax2.grid(True,'major','x')
        
        for k,v in self.logs.items():
            color=color_correction(self.collector.ac_dict[k].config.color)
            errors,ws,timestamps = v.to_numpy()
            # print(k,errors)
            ax1.plot(timestamps*1e-3,np.sqrt(np.sum(np.square(errors),1)),label=f"AC {k} euclidean distance to guiding point",
                     color=color)
            ax2.plot(timestamps*1e-3,ws,label=f"AC {k} virtual coordinate",
                     color=color)
            print(f"AC {k} color: {color}")
            
        ax1.legend()
        ax2.legend()
        plt.show()
        
    def export(self):
        filename = input("Input savefile name:")
        
        savez_kwargs = dict()
        for k,v in self.logs.items():
            errors,ws,timestamps = v.to_numpy()
            savez_kwargs["AC_{}__l2err_ws_ts".format(k)] = np.stack([np.sqrt(np.sum(np.square(errors),1)),ws,timestamps])
        
        np.savez_compressed(filename,**savez_kwargs)
            
        
        
    def __coordination_total_error(self) -> np.ndarray:
        assert self.coordination
        
        # Gather local error data and timestamps (SUPPOSE THE ACs CLOCK ARE SYNCHRONIZED)
        
        timed_local_errors = []
        for v in self.coord_logs.values():
            np_v = v.to_numpy()
            errors = np_v[0]
            timestamps = np_v[6]
            # print(f"{v.ac_id} : {timestamps}")
            timed_local_errors.append(np.asarray((np.sum(np.abs(errors),1),timestamps)).transpose())
        
        # Initialise indexes for fusion
        local_indexes = np.zeros(len(timed_local_errors),dtype=int)
        max_indexes = np.asarray([len(l) for l in timed_local_errors])
        
        # Select the first timestamp from which all local errors are defined
        start_timestamp = max(timed_local_errors[:][0][1])
        
        # Actualize indexes accordingly
        for nei in range(len(timed_local_errors)):
            i = local_indexes[nei]
            while i < len(timed_local_errors[nei]) and timed_local_errors[nei][i][1] < start_timestamp:
                i += 1
            local_indexes[nei] = i
        
        # Merge (remember to divide error data by 2, because of the degree/edges formula)
        # print("Local:",local_indexes)
        # print("Max:  ",max_indexes)
        output = []
        while np.any(local_indexes < max_indexes):
            min_time_nei = np.argmin([vals[i][1] if i < max_i else np.inf for i,max_i,vals in zip(local_indexes,max_indexes,timed_local_errors)])
            new_global_error = 0
            for nei,index in enumerate(local_indexes):
                if nei == min_time_nei:
                    new_global_error += timed_local_errors[nei][index][0]
                else:
                    assert index-1 >= 0
                    new_global_error += timed_local_errors[nei][index-1][0]
            output.append((new_global_error/2,timed_local_errors[min_time_nei][local_indexes[min_time_nei]][1]))
            local_indexes[min_time_nei] += 1
            
        return np.asarray(output)
        
        
        
    def coordination_plot(self):
        assert self.coordination
        
        fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
        ax1:Axes
        ax2:Axes
        
        title = input("Input coordination figure title:")
        
        fig.suptitle(title)
        
        ax1.set_xlabel("Time ellapsed (s)")
        ax1.set_ylabel("L1 Coordination error (a.u.)")
        
        ax2.set_xlabel("Time ellapsed (s)")
        ax2.set_ylabel("Virtual coordinate (a.u.)")
        
        for k,v in self.coord_logs.items():
            color=color_correction(self.collector.ac_dict[k].config.color)
            errors,_,_,_,_,_,coord_timestamps = v.to_numpy()
            ax1.plot(coord_timestamps*1e-3,np.sum(np.abs(errors),1),label=f"AC {k}",
            color=color)
        
        total_timed_errors = self.__coordination_total_error()
        ax1.plot(total_timed_errors[:,1]*1e-3,total_timed_errors[:,0],label="Global error")
        
        for k,v in self.logs.items():
            color=color_correction(self.collector.ac_dict[k].config.color)
            _,ws,timestamps = v.to_numpy()
            ax2.plot(timestamps*1e-3,ws,label=f"AC {k}",
            color=color)
            
            
        ax1.legend()
        ax2.legend()
        plt.show()
        
    def coordination_export(self):
        filename = input("Input coordination savefile name:")
        
        savez_kwargs = dict()
        for k,v in self.coord_logs.items():
            errors,_,_,_,_,_,coord_timestamps = v.to_numpy()
            savez_kwargs[f"AC_{k}__l1err_ts"] = np.stack([np.sum(np.abs(errors),1),coord_timestamps])
        
        packed_total_err_ts = self.__coordination_total_error()
        total_errs = np.asarray(packed_total_err_ts[:][0])
        total_ts = np.asarray(packed_total_err_ts[:][1])
        savez_kwargs["Total_err_ts"] = np.stack([total_errs,total_ts])
        
        np.savez_compressed(filename,**savez_kwargs)
        
        
        
    def run(self):
        print(f"Data collection started!\n"
              f"Target(s)   : {self.ac_ids}\n"
              f"Coordination: {self.coordination}\n"
              f"Save at exit: {self.do_export}")
        try:
            print(f"--- Data acquisition started ---\n(Press Ctrl-C to end)")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass 
        self._ivy_interface.shutdown()
        
        # for k,v in self.logs.items():
        #     print(k,v.errors[:10])
        
        self.plot()
        if self.coordination:
            self.coordination_plot()
        
        if self.do_export:
            self.export()
            if self.coordination:
                self.coordination_export()
        
        exit(0)
                 
############### Entrypoint ###############

import argparse

def main():
    parser = argparse.ArgumentParser("GVF error logger",description="A program to monitor and summarize GVF performances")
    parser.add_argument('ids',nargs='+',
                        help="List of aircraft ids' to track")
    
    parser.add_argument('--name',type=str,dest='name',default='GVF_error_logger',
                        help="Name for the underlying Ivy interface collecting messages. Default is 'GVF_error_logger'")
    
    parser.add_argument('-c','--cordination',dest='coordination',action='store_true',default=False,
                        help="Activate coordination error tracking (global metric, L1 norm)")
    
    parser.add_argument('-e','--export',dest='export',action='store_true',default=False,
                        help="Export the data tables as Numpy compressed arrays (npz). Name is asked when stopping collection.")
    
    args = parser.parse_args()
    
    errorLoggger = ErrorLogger([int(e) for e in args.ids],args.name,args.coordination,args.export)
    errorLoggger.run()

if __name__ == "__main__":
    main()