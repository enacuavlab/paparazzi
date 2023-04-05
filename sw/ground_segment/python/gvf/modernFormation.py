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
from pprzlink.generated import telemetry,ground,datalink

from settings import PprzSettingsManager
from pprz_connect import PprzConnect,PprzConfig

#################### Aircraft dataclass ####################

@dataclasses.dataclass
class Aircraft:
    id:int
    settingManager:PprzSettingsManager
    
    initialized_gvf:bool = False
    initialized_nav:bool = False
    
    X:float = 0. 
    Y:float = 0.
    alt:float = np.nan
    Xc:float = 0.
    Yc:float = 0.
    a:float = 0.
    b:float = 0.
    s:int = 1
    sigma:float = 0.
    blockId:typing.Optional[int] = None
    
    
#################### Formation controllers ####################

class AbstractFormationController(ABC):
    """
    On abstract class for formation management controllers, based on Python threads
    """
    
    def __init__(self,freq:float,ivy:typing.Union[str,IvyMessagesInterface],verbosity:int,**kwargs) -> None:
        self._freq = freq
        if isinstance(ivy,IvyMessagesInterface):
            self._ivy_interface = ivy
        else:
            self._ivy_interface = IvyMessagesInterface(ivy,True,verbose=(verbosity>0))
        self.verbosity:int = verbosity
        
        super().__init__()
    
    ## Required properties
    
    @property
    def msg_freq(self) -> float:
        """
        High bound on frequency (Hz)
        (i.e. the `step` method will be executed at most once each
        `msg_freq` seconds)
        """
        return self._freq
        
    ## Control functions
    @abstractmethod
    def on_start(self) -> None:
        """
        Executed before starting the main loop (thread just started)
        """
        pass 
    
    @abstractmethod
    def step(self) -> None:
        """
        Main function, is executed repetively
        """
        raise NotImplementedError()
    
    @abstractmethod
    def end_condition(self) -> bool:
        """
        Thread ends when this function returns True
        """
        return False
    
    @abstractmethod
    def on_end(self) -> None:
        """
        Executed when exiting the main loop (thread is ending)
        """
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """
        Clean everything that needs to be cleaned (namely, shutdown the Ivy interface).
        Very last function called.
        """
        self._ivy_interface.shutdown()
    
    def __del__(self) -> None:
        self.stop()
        
    ## Core execution loop
    
    def run(self) -> None:
        self.on_start()
        
        last_step_at = 0.
        try:
            # The main loop
            while not self.end_condition():
                delta_t = time.time() - last_step_at
                if delta_t < 1/self.msg_freq:
                    sleep(1/self.msg_freq - delta_t)
                last_step_at = time.time()
                self.step()

        except KeyboardInterrupt:
            pass
        
        self.on_end()
        self.stop()
        

##### Basic, centralized Circular formation controller #####

class CCircularFormationController(AbstractFormationController):
    def __init__(self, freq: float, ivy:typing.Union[str,IvyMessagesInterface], verbosity: int,
                 formation_file:str,threshold:float=0., elevation:float=0., max_altitude:float=0.) -> None:
            
        self.threshold:float = threshold
        self.elevation:float = elevation
        self.max_altitude:float = max_altitude
        self.formation_error = np.inf
            
        # Gather data from JSON file
        with open(formation_file,'r') as f:
            conf = json.load(f)
            self.ids:typing.List = conf['ids']
            self.B = np.array(conf['topology'],dtype=int)
            self.Zdesired = np.array(conf['desired_intervehicle_angles_degrees'],dtype=float)*np.pi/180
            self.k = float(conf['gain'])
            self.radius = float(conf['desired_stationary_radius_meters'])  
        
        # Setup Ivy interface and PprzConnect
        if isinstance(ivy,IvyMessagesInterface):
            self._ivy_interface = ivy
        else:
            self._ivy_interface = IvyMessagesInterface(ivy,True,verbose=verbosity>1)
            
        self._pprz_connect = PprzConnect(ivy=self._ivy_interface,verbose=verbosity>1)
        
        # Aircrafts' data
        self.aircrafts:typing.Dict[int,Aircraft] = dict()
        self.sigmas = np.zeros(len(self.ids))
        
        super().__init__(freq, self._ivy_interface, verbosity)
        
    def on_start(self) -> None:
        """
        Gather aircrafts' config data and setup associated callbacks
        """
        for i in self.ids:
            success = False
            while not success:
                try:
                    self._pprz_connect.get_config(i)
                    conf_msg:PprzConfig = self._pprz_connect.conf_by_id(i)
                    success = True
                except KeyError:
                    print(f"Waiting for aircraft with id {i} ...")
                    sleep(1.)
                
            settings_path = os.path.normpath(conf_msg.settings)
            self.aircrafts[i] = Aircraft(i,PprzSettingsManager(settings_path,i,self._ivy_interface))
            try:
                print("Setting 'ell_a': ",self.aircrafts[i].settingManager['ell_a'])
                print("Setting 'ell_b': ",self.aircrafts[i].settingManager['ell_b'])
            except KeyError as e:
                print(f"Could not find ellipsis settings ('ell_a' or 'ell_b') for AC_ID={i}\n\
Have you forgotten to check gvf.xml in your settings?")
                self._ivy_interface.shutdown()
                self._ivy_interface = None
                raise e
            
        # bind to NAVIGATION message
        def nav_cb(ac_id, msg:telemetry.PprzMessage_NAVIGATION):
            if ac_id in self.ids:# and msg.name == "NAVIGATION":
                ac = self.aircrafts[ac_id]
                ac.X = msg.pos_x_
                ac.Y = msg.pos_y_
                ac.blockId = msg.cur_block_
                ac.initialized_nav = True
        self._ivy_interface.subscribe(nav_cb, telemetry.PprzMessage_NAVIGATION())

        # bind to GPS message
        def gps_cb(ac_id, msg:telemetry.PprzMessage_GPS):
            if ac_id in self.ids:
                ac = self.aircrafts[ac_id]
                ac.alt = float(msg.alt_ * 1000) # altitude in the msg is in mm above MSL
        self._ivy_interface.subscribe(gps_cb,telemetry.PprzMessage_GPS())

        # bind to GVF message
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
        self._ivy_interface.subscribe(gvf_cb, telemetry.PprzMessage_GVF())
        
        
    def step(self) -> None:
        '''
        circular formation control algorithm
        '''

        ready = True
        for ac in self.aircrafts.values():
            if (not ac.initialized_nav) or (not ac.initialized_gvf):
                if self.verbosity>0:
                    print("Waiting for state of aircrafts ", ac.id)
                ready = False

        if not ready:
            self.formation_error = np.inf
            return None

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

        if self.verbosity>0:
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
            
        self.formation_error = output
        return None
    
    def end_condition(self) -> bool:
        error_cond = self.formation_error < self.threshold
        altitude_cond:bool
        if self.elevation == 0.:
            altitude_cond = True
        else:
            if np.nan in [ac.alt for ac in self.aircrafts.values()]:
                altitude_cond = False
            else:
                if self.elevation > 0:
                    altitude_cond = min(ac.alt for ac in self.aircrafts.values()) > self.max_altitude
                else:# self.elevation < 0:
                    altitude_cond = max(ac.alt for ac in self.aircrafts.values()) < self.max_altitude
        return error_cond and altitude_cond
        
        
    def on_end(self) -> None:
        # If there is a threshold, jump to next block
        if self.threshold > 0. or self.max_altitude != 0.:
            for ac in self.aircrafts.values():
                msg = ground.PprzMessage_JUMP_TO_BLOCK()
                msg.ac_id_ = str(ac.id)
                msg.block_id_ = ac.blockId+1
                self._ivy_interface.send(msg)
                
    def stop(self) -> None:
        return super().stop()

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
        self.formation_error = np.inf
            
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
            self.formation_error = np.inf
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
            
        self.formation_error = output
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


##### Parametric formation controller (distributed; only used to setup the formation) #####


class ParametricFormationController(AbstractFormationController):
    def __init__(self, freq: float, ivy: typing.Union[str, IvyMessagesInterface], verbosity: int,
                formation_file:str, max_w:float=0.) -> None:
        self.max_w:float = max_w
            
        # Gather data from JSON file
        with open(formation_file,'r') as f:
            conf = json.load(f)
            self.ids:typing.List = conf['ids']
            self.B = np.array(conf['topology'],dtype=int)
            self.delta_ws = np.array(conf['desired_intervehicle_w'],dtype=float)
            self.edge_list:typing.Optional[np.ndarray]
            try:
                self.edge_list = np.array(conf['edge_list'],dtype=int)
            except KeyError:
                self.edge_list = None
        
        # Setup Ivy interface and PprzConnect
        if isinstance(ivy,IvyMessagesInterface):
            self._ivy_interface = ivy
        else:
            self._ivy_interface = IvyMessagesInterface(ivy,True,verbose=verbosity>1)
            
        self._pprz_connect = PprzConnect(ivy=self._ivy_interface,verbose=verbosity>1)
        
        # Aircrafts' data
        self.aircrafts:typing.Dict[int,Aircraft] = dict()
        
        super().__init__(freq, self._ivy_interface, verbosity)
        
    def on_start(self) -> None:
        """
        Gather aircrafts' config data and clean their formation data
        """
        for i in self.ids:
            success = False
            while not success:
                try:
                    self._pprz_connect.get_config(i)
                    conf_msg:PprzConfig = self._pprz_connect.conf_by_id(i)
                    success = True
                except KeyError:
                    print(f"Waiting for aircraft with id {i} ...")
                    sleep(1.)
                
            settings_path = os.path.normpath(conf_msg.settings)
            self.aircrafts[i] = Aircraft(i,PprzSettingsManager(settings_path,i,self._ivy_interface))
        
        for i in self.ids:
            # nei_id = 0, special msg to clean the table onboard
            msg = datalink.PprzMessage_GVF_PARAMETRIC_REG_TABLE()
            msg.ac_id_ = i
            msg.nei_id_ = 0
            msg.desired_deltaw_ = 0.
            self._ivy_interface.send(msg)
            
            if self.verbosity>0:
                print(msg)
            
        # bind to GVF_PARAMETRIC_W message
        def gvf_cb(ac_id, msg:telemetry.PprzMessage_GVF_PARAMETRIC_W):
            if ac_id in self.ids:# and msg.name == "GVF_PARAMETRIC_W":
                ac = self.aircrafts[ac_id]
                ac.sigma = msg.w_
        self._ivy_interface.subscribe(gvf_cb, telemetry.PprzMessage_GVF_PARAMETRIC_W())
            
    def step(self) -> None:
        if self.edge_list is None:
            for count, column in enumerate(self.B.T):
                index = np.nonzero(column)
                i = index[0]

                msga = datalink.PprzMessage_GVF_PARAMETRIC_REG_TABLE()
                msga.ac_id_ = int(self.ids[i[0]])
                msga.nei_id_ = int(self.ids[i[1]])
                if len(self.ids) == 2:
                    msga.desired_deltaw_ = (column[index])[0]*float(self.delta_ws)
                else:
                    msga.desired_deltaw_ = (column[index])[0]*float(self.delta_ws[count])

                msgb = datalink.PprzMessage_GVF_PARAMETRIC_REG_TABLE()
                msgb.ac_id_ = int(self.ids[i[1]])
                msgb.nei_id_ = int(self.ids[i[0]])
                if len(self.ids) == 2:
                    msgb.desired_deltaw_ = (column[index])[1]*float(self.delta_ws)
                else:
                    msgb.desired_deltaw_ = (column[index])[1]*float(self.delta_ws[count])
                    
                self._ivy_interface.send(msga)
                self._ivy_interface.send(msgb)

                if self.verbosity>0:
                    print(msga)
                    print(msgb)

        else:
            if len(self.delta_ws) == 1:
                delta_ws = itertools.repeat(self.delta_ws,len(self.edge_list))
            else:
                delta_ws = self.delta_ws
                
            for w,p in zip(delta_ws,self.edge_list):
                msga = datalink.PprzMessage_GVF_PARAMETRIC_REG_TABLE()
                msga.ac_id_ = int(p[0])
                msga.nei_id_ = int(p[1])
                msga.desired_deltaw_ = float(w)
                
                msgb = datalink.PprzMessage_GVF_PARAMETRIC_REG_TABLE()
                msgb.ac_id_ = int(p[1])
                msgb.nei_id_ = int(p[0])
                msgb.desired_deltaw_ = float(-w)
                
                self._ivy_interface.send(msga)
                self._ivy_interface.send(msgb)

                if self.verbosity>0:
                    print(msga)
                    print(msgb)
            

        
    
    def end_condition(self) -> bool:
        return np.all([ac.sigma > self.max_w for ac in self.aircrafts.values()])
    
    def on_end(self) -> None:
        return super().on_end()
    
    def stop(self) -> None:
        return super().stop()

#################### Main entry point ####################

def main():
    import argparse

    parser = argparse.ArgumentParser(description="Circular formation")
    parser.add_argument('formation_file', help="JSON configuration file")
    parser.add_argument('-f', '--freq', dest='freq', default=5, type=int, help="control frequency (Hz)")
    parser.add_argument('-v', '--verbose', dest='verbose', default=0, action='count', help="display debug messages")
    parser.add_argument('-p','--parametric',dest='parametric',default=np.nan,nargs='?',
                        help="Use distributed parametric controller instead (the ground only setup and monitor). If a value is given,\
                        then the controller runs until all drones have their 'virtual coordinate w' above the given value.\
                        Setting this flag nullify `threshold`, `elevate` and `altitude`.")
    parser.add_argument('-t', '--threshold', dest='threshold', default=0.,type=float, help="Error threshold (degrees). \
        If positive, then the program stops as soon as the maximal inter-vehicules error is below said threshold.\
        If elevation is also set (and non-nul), the program may continue infinitely (see `altitutde` parameter).")
    parser.add_argument('-e', '--elevate', dest='elevate', default=0., type=float, help="Wanted vertical airspeed for each drone (m/s).\
        If threshold is set, elevation is asked only when the formation is below the error threshold.")
    parser.add_argument('-a','--altitude', dest='altitude', default=0., type=float,
                        help="Altitude at which to stop control. Elevation must be set (if positive, then the program stops\
                            when all drones are above the altitude, otherwise when all are below).")
    parser.add_argument('-n','--name',default=None,help="Name for the Ivy interface")
    
    args = parser.parse_args()
    
    if args.parametric is None or not np.isnan(args.parametric):
        
        name = args.name if args.name is not None else 'ParametricFormationController'
        max_w = args.parametric if args.parametric is not None else np.inf
        fc = ParametricFormationController(args.freq,name,args.verbose,args.formation_file,max_w=max_w)
    else:
        name = args.name if args.name is not None else 'CircularFormationController'
        fc = CCircularFormationController(args.freq,name,args.verbose,args.formation_file,
                                          args.threshold,args.elevate,args.altitude)
    fc.run()
    
if __name__ == "__main__":
    main()