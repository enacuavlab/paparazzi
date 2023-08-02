import typing
import inspect
from dataclasses import dataclass
from time import sleep
import os 

import numpy as np

from trajectories import GVF, GVF_PARAMETRIC
from trajectories.gvf_trajectories import Trajectory, LineTrajectory,SurfaceTrajectory

from pprzlink.generated import telemetry,ground
from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(
    path.join(path.dirname(path.abspath(__file__)), '../../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(
    path.join(path.dirname(path.abspath(__file__)), '../../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from settings import PprzSettingsManager
from pprz_connect import PprzConnect,PprzConfig

#################### Meta code to import all known trajectories ####################


def __get_GVF_traj_map() -> typing.Dict[int, object]:
    classes = list(filter(lambda t: inspect.isclass(t[1]) 
                          and issubclass(t[1],(LineTrajectory,SurfaceTrajectory))
                          and not inspect.isabstract(t[1]), inspect.getmembers(GVF)))
    output = dict()
    for (n, o) in classes:
        try:
            output[o.class_id()] = o
        except Exception as e:
            print(f"Error with class {n}:\n{o}")
            raise e
    return output


GVF_traj_map = __get_GVF_traj_map()


def __get_GVF_PARAMETRIC_traj_map() -> typing.Dict[int, object]:
    classes = list(filter(lambda t: inspect.isclass(t[1]) 
                          and issubclass(t[1],(LineTrajectory,SurfaceTrajectory))
                          and not inspect.isabstract(t[1]), inspect.getmembers(GVF_PARAMETRIC)))
    output = dict()
    for (n, o) in classes:
        try:
            output[o.class_id()] = o
        except Exception as e:
            print(f"Error with class {n}:\n{o}")
            raise e
    return output


GVF_PARAMETRIC_traj_map = __get_GVF_PARAMETRIC_traj_map()

#################### Aircraft data container ####################

@dataclass
class Aircraft():
    id:int
    settingManager:PprzSettingsManager
    
    attitude:typing.Optional[telemetry.PprzMessage_ATTITUDE] = None
    navigation:typing.Optional[telemetry.PprzMessage_NAVIGATION] = None
    ground_data:typing.Optional[ground.PprzMessage_FLIGHT_PARAM] = None
    gps:typing.Optional[telemetry.PprzMessage_GPS] = None
    gvf:typing.Optional[telemetry.PprzMessage_GVF] = None
    gvf_parametric:typing.Optional[telemetry.PprzMessage_GVF_PARAMETRIC] = None
    config:typing.Optional[PprzConfig] = None
    
    @property
    def XYZ(self) -> np.ndarray:
        if self.gps is None:
            return np.zeros(3)
        
        if self.navigation is None:
            return np.zeros(3)
        
        return np.array([self.navigation.pos_x_,self.navigation.pos_y_, self.gps.alt_ / 1000])
    
    
    @property
    def trajectory(self) -> typing.Optional[Trajectory]:
        if self.gvf_parametric is None and self.gvf is None:
            return None
        
        if self.gvf_parametric is None:
            return GVF_traj_map[self.gvf.traj_].from_message(self.gvf)
        
        else:
            return GVF_PARAMETRIC_traj_map[self.gvf_parametric.traj_].from_message(self.gvf_parametric)
    
    @property
    def w(self) -> typing.Optional[float]:
        if self.gvf_parametric is None:
            return None
        else:
            return float(self.gvf_parametric.config_[0])*float(self.gvf_parametric.config_[2])
    
    @property
    def beta_s(self) -> typing.Optional[float]:
        if self.gvf_parametric is None:
            return None
        else:
            return float(self.gvf_parametric.config_[2])
    
    @property
    def L(self) -> typing.Optional[float]:
        if self.gvf_parametric is None:
            return None
        else:
            return float(self.gvf_parametric.config_[6])
        
    @property
    def gains(self) -> typing.Optional[np.ndarray]:
        if self.gvf_parametric is None:
            return None
        else:
            return np.asarray([float(self.gvf_parametric.config_[8]),float(self.gvf_parametric.config_[9]),float(self.gvf_parametric.config_[10])])
    
    @property
    def name(self) -> str:
        if self.config is None:
            return f"AC {self.id}"
        else:
            return self.config.ac_name
        
        

#################### Aircraft data collector ####################

class AC_DataCollector():
    def __init__(self,ac_id_list:typing.List[int],ivy:typing.Union[str,IvyMessagesInterface]="GVF_collector",setup:bool=True) -> None:
        
        # Vehicle variables
        self.ac_ids:typing.List[int] = ac_id_list
        self.ac_dict:typing.Dict[int,Aircraft] = dict()
        
        # Ivy
        if isinstance(ivy,IvyMessagesInterface):
            self._ivy_interface = ivy
        else:
            self._ivy_interface = IvyMessagesInterface(ivy)
        self._pprz_connect = PprzConnect(ivy=self._ivy_interface)
        
        # Setup
        if setup:
            self.setup()
        
    def setup(self) -> None:
        """
        **Wait** until aircrafts' config data are gathered (and setup associated callbacks)
        """
        for i in self.ac_ids:
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
            self.ac_dict[i] = Aircraft(i,PprzSettingsManager(settings_path,i,self._ivy_interface))
            self.ac_dict[i].config = conf_msg
            
        
        # bind to ATTITUDE message
        def attitude_cb(ac_id, msg:telemetry.PprzMessage_ATTITUDE):
            if ac_id in self.ac_ids:# and msg.name == "ATTITUDE":
                ac = self.ac_dict[ac_id]
                ac.attitude = msg
        self._ivy_interface.subscribe(attitude_cb, telemetry.PprzMessage_ATTITUDE())
            
        # bind to NAVIGATION message
        def nav_cb(ac_id, msg:telemetry.PprzMessage_NAVIGATION):
            if ac_id in self.ac_ids:# and msg.name == "NAVIGATION":
                ac = self.ac_dict[ac_id]
                ac.navigation = msg
        self._ivy_interface.subscribe(nav_cb, telemetry.PprzMessage_NAVIGATION())

        # bind to GPS message
        def gps_cb(ac_id, msg:telemetry.PprzMessage_GPS):
            if ac_id in self.ac_ids:# and msg.name == "GPS":
                ac = self.ac_dict[ac_id]
                ac.gps = msg
        self._ivy_interface.subscribe(gps_cb,telemetry.PprzMessage_GPS())

        # bind to GVF message
        def gvf_cb(ac_id, msg:telemetry.PprzMessage_GVF):
            if ac_id in self.ac_ids:# and msg.name == "GVF":
                ac = self.ac_dict[ac_id]
                ac.gvf = msg
                ac.gvf_parametric = None
        self._ivy_interface.subscribe(gvf_cb, telemetry.PprzMessage_GVF())
        
        # bind to GVF_PARAMETRIC message
        def gvf_par_cb(ac_id, msg:telemetry.PprzMessage_GVF_PARAMETRIC):
            if ac_id in self.ac_ids:# and msg.name == "GVF_PARAMETRIC":
                ac = self.ac_dict[ac_id]
                ac.gvf_parametric = msg
                ac.gvf = None
        self._ivy_interface.subscribe(gvf_par_cb, telemetry.PprzMessage_GVF_PARAMETRIC())
        
    def shutdown(self):
        self._ivy_interface.shutdown()
        
    def __del__(self):
        self.shutdown()