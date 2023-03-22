#!/usr/bin/env python3
#
# Copyright (C) 2023 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#                    Mael Feurgard <mael.feurgard@laas.fr>
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
Modernized version of the Centralized circular formations employing guidance vector fields (gvf)
'''
import sys
import numpy as np
import json
import typing
import dataclasses
import os

from time import sleep
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.generated import telemetry,ground

from settings import PprzSettingsManager
from pprz_connect import PprzConnect,PprzConfig

@dataclasses.dataclass
class Aircraft:
    id:int
    settingManager:PprzSettingsManager
    
    initialized_gvf:bool = False
    initialized_nav:bool = False
    
    X:float = 0. 
    Y:float = 0.
    Xc:float = 0.
    Yc:float = 0.
    a:float = 0.
    b:float = 0.
    s:int = 1
    sigma:float = 0.
    blockId:typing.Optional[int] = None
    
    
class FormationControl:
    def __init__(self,formation_config:dict, freq:float=10., verbose:bool=False, threshold:float=0., elevation:float=0.):
        self.config = formation_config
        self.step:float = 1. / freq
        self.verbose:bool = verbose
        self.ids:typing.List[int] = self.config['ids']
        self.B = np.array(self.config['topology'],dtype=int)
        self.Zdesired = np.array(self.config['desired_intervehicle_angles_degrees'],dtype=float)*np.pi/180
        self.k = float(self.config['gain'])
        self.radius = float(self.config['desired_stationary_radius_meters'])    
        self.threshold:float = threshold
        self.elevation:float = elevation
        
        self._interface = IvyMessagesInterface("Modern_Circular_Formation")
        self._pprz_connect = PprzConnect(ivy=self._interface)
        
        
        self.aircrafts:typing.Dict[int,Aircraft] = dict()
        for i in self.ids:
            success = False
            while not success:
                try:
                    self._pprz_connect.get_config(str(i))
                    conf_msg:PprzConfig = self._pprz_connect.conf_by_id(i)
                    success = True
                except KeyError:
                    print(f"Waiting for aircraft with id {i} ...")
                    sleep(1.)
                
            settings_path = os.path.normpath(conf_msg.settings)
            self.aircrafts[i] = Aircraft(i,PprzSettingsManager(settings_path,i,self._interface))
            try:
                print("Setting 'ell_a': ",self.aircrafts[i].settingManager['ell_a'])
                print("Setting 'ell_b': ",self.aircrafts[i].settingManager['ell_b'])
            except KeyError as e:
                print(f"Could not find ellipsis settings ('ell_a' or 'ell_b') for AC_ID={i}\n\
Have you forgotten to check gvf.xml in your settings?")
                self._interface.shutdown()
                self._interface = None
                raise e
        self.sigmas = np.zeros(len(self.aircrafts))
            
        # bind to NAVIGATION message
        def nav_cb(ac_id, msg:telemetry.PprzMessage_NAVIGATION):
            if ac_id in self.ids:# and msg.name == "NAVIGATION":
                ac = self.aircrafts[ac_id]
                ac.X = msg.pos_x_
                ac.Y = msg.pos_y_
                ac.blockId = msg.cur_block_
                ac.initialized_nav = True
        self._interface.subscribe(nav_cb, telemetry.PprzMessage_NAVIGATION())

        def gvf_cb(ac_id, msg:telemetry.PprzMessage_GVF):
            if ac_id in self.ids:# and msg.name == "GVF":
                if int(msg.get_field(1)) == 1:
                    ac = self.aircrafts[ac_id]
                    param = msg.p_
                    ac.Xc = float(param[0])
                    ac.Yc = float(param[1])
                    ac.a = float(param[2])
                    ac.b = float(param[3])
                    
                    ac.s = msg.s_ if msg.s_ is not None else 1
                    ac.initialized_gvf = True
        self._interface.subscribe(gvf_cb, telemetry.PprzMessage_GVF())

    
    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            
            # If there is a threshold, jump to next block
            if self.threshold > 0.:
                for ac in self.aircrafts.values():
                    msg = ground.PprzMessage_JUMP_TO_BLOCK()
                    msg.ac_id_ = str(ac.id)
                    msg.block_id_ = ac.blockId+1
                    self._interface.send(msg)
            
            self._interface.shutdown()
            self._interface = None   
        
    def circular_formation(self) -> float:
        '''
        circular formation control algorithm
        '''

        ready = True
        for ac in self.aircrafts.values():
            if (not ac.initialized_nav) or (not ac.initialized_gvf):
                if self.verbose:
                    print("Waiting for state of aircrafts ", ac.id)
                ready = False

        if not ready:
            return np.inf

        for i,id in enumerate(self.ids):
            ac = self.aircrafts[id]
            ac.sigma = np.arctan2(ac.Y-ac.Yc, ac.X-ac.Xc)
            self.sigmas[i] = ac.sigma

        inter_sigma = self.B.transpose().dot(self.sigmas)
        error_sigma = ((inter_sigma - self.Zdesired) % (2*np.pi))

        if np.size(error_sigma) > 1:
            error_sigma[error_sigma > np.pi] -= 2*np.pi
            # for i in range(0, np.size(error_sigma)):
            #     if error_sigma[i] > np.pi:
            #         error_sigma[i] = error_sigma[i] - 2*np.pi
            #     elif error_sigma[i] <= -np.pi:
            #         error_sigma[i] = error_sigma[i] + 2*np.pi
        else:
            if error_sigma > np.pi:
                error_sigma = error_sigma - 2*np.pi
            # elif error_sigma <= -np.pi:
            #     error_sigma = error_sigma + 2*np.pi


        u = -self.aircrafts[self.ids[0]].s*self.k*self.B.dot(error_sigma)
        max_error = np.linalg.norm(error_sigma,np.inf)

        if self.verbose:
            print("Inter-vehicle errors: ", str(error_sigma*180.0/np.pi).replace('[','').replace(']',''))
            print(f"Maximal error: {max_error*180.0/np.pi}")
            print(f"Delta Radius vector: {u}")

        for i,id in enumerate(self.ids):
            ac = self.aircrafts[id]
            ac.settingManager['ell_a'] = self.radius + u[i]
            ac.settingManager['ell_b'] = self.radius + u[i]
        
        
        output = max_error*180.0/np.pi
        
        if self.threshold == 0.:
            if self.elevation != 0.:
                for ac in self.aircrafts.values():
                    ac.settingManager['altitude'] = self.elevation + ac.settingManager['altitude'].value
        else:
            if output < self.threshold:
                if self.elevation != 0.:
                    for ac in self.aircrafts.values():
                        ac.settingManager['altitude'] = self.elevation + ac.settingManager['altitude'].value
                else:
                    raise KeyboardInterrupt
            
        
        return output

    def run(self):
        try:
            # The main loop
            max_error = np.inf
            while True:
                # TODO: make better frequency managing
                sleep(self.step)

                # Run the formation algorithm
                max_error = self.circular_formation()

        except KeyboardInterrupt:
            pass
        
        if self.verbose:
            print(f"Circular formation ending...")
            print(f"Threshold: {self.threshold}\nMax error: {max_error}")
        
        self.stop()
        
        
        
def main():
    import argparse

    parser = argparse.ArgumentParser(description="Circular formation")
    parser.add_argument('formation_file', help="JSON configuration file")
    parser.add_argument('-f', '--freq', dest='freq', default=5, type=int, help="control frequency (Hz)")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    parser.add_argument('-t', '--threshold', dest='threshold', default=0.,type=float, help="Error threshold (degrees). \
        If positive, then the program stops as soon as the maximal inter-vehicules error is below said threshold.\
        If elevation is also set (and non-nul), the program continue infinitely.")
    parser.add_argument('-e', '--elevate', dest='elevation', default=0., type=float, help="Wanted vertical airspeed for each drone (m/s).\
        If threshold is set, elevation is asked only when the formation is below the error threshold.")
    args = parser.parse_args()
    
    
    with open(args.formation_file, 'r') as f:
        conf = json.load(f)
        if args.verbose:
            print(json.dumps(conf))

        fc = FormationControl(conf, freq=args.freq, verbose=args.verbose,threshold=args.threshold,elevation=args.elevation)
        fc.run()
    
if __name__ == "__main__":
    main()