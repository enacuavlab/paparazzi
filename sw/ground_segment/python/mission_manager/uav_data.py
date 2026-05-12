from dataclasses import dataclass,field
from typing import Optional

import sys
from os import path, getenv
# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from flight_plan import FlightPlan, Block, Waypoint
from Dubins import Pose3D,poses_XY_dist,min_XY_dist
from plotting import DictOfPoseTrajectories

import numpy as np

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
    agl: float = 0.
    heading: float = 0.
    vnorth: float = 0.
    veast: float = 0.
    vup: float = 0.
    ref_alt: float = 0.
    bat_voltage: float = 0.
    bat_charge: float = 0.
    gps_tow: int = 0
    flight_time: int = 0
    AP_mode: str = 'none'
    FP_block: str = 'none'
    mission_status: list[int] = field(default_factory=list)
    datalink_lost_time: int = 0
    time_since_last_msg: float = 0.
    flight_plan: Optional[FlightPlan] = None
    start_mission_fp_block: Optional[Block] = None
    airspeed: float = 0.
    airspeed_sp: float = 0.
    groundspeed_sp: float = 0.
    wind_east:Optional[float] = None
    wind_north:Optional[float] = None
    wind_up:Optional[float] = None
    
    @property
    def height(self) -> float:
        return self.alt - self.ref_alt

@dataclass
class TrackingData:
    '''
    Store tracking information for a UAV
    '''
    ac_id: int
    timestamp: int|float
    pose: Pose3D
    expected_pose: Optional[Pose3D] = None
    
    def XY_dist(self) -> Optional[float]:
        assert self.expected_pose is not None, "Expected pose is not set"
        return poses_XY_dist(self.pose, self.expected_pose)
    
    def __le__(self, other: 'TrackingData') -> bool:
        return self.timestamp <= other.timestamp

    def __lt__(self, other: 'TrackingData') -> bool:
        return self.timestamp < other.timestamp
    
    def __ge__(self, other: 'TrackingData') -> bool:
        return not(self < other)
    
    def __gt__(self, other: 'TrackingData') -> bool:
        return not(self <= other)
    
    def to_timed_pose(self) -> tuple[float,Pose3D]:
        return self.timestamp, self.pose

@dataclass
class TrackingLogs:
    '''
    Store tracking logs for multiple UAVs
    '''
    logs: dict[int, list[TrackingData]] = field(default_factory=dict)
    _tpose_lists: dict[int, np.ndarray] = field(default_factory=dict)
    _sorted: set[int] = field(default_factory=set)
    
    def add_tracking_data(self, data: TrackingData):
        if data.ac_id not in self.logs:
            self.logs[data.ac_id] = []
        self.logs[data.ac_id].append(data)
        
        if data.ac_id in self._sorted and self.logs[data.ac_id][-1] < self.logs[data.ac_id][-2]:
            self._sorted.remove(data.ac_id)
        
        self._tpose_lists.pop(data.ac_id, None)
            
    def get_ids(self) -> set[int]:
        return set(self.logs.keys())
    
    def get_all_timestamps(self) -> set[int|float]:
        timestamps = set()
        for log in self.logs.values():
            for data in log:
                timestamps.add(data.timestamp)
        return timestamps
    
    def get_errors(self) -> dict[int, list[tuple[int|float,float]]]:
        errors = {}
        for ac_id, log in self.logs.items():
            errors[ac_id] = [(data.timestamp, data.XY_dist()) for data in log if data.expected_pose is not None]
        return errors
    
    def sort_logs(self):
        for ac_id, log in self.logs.items():
            if ac_id not in self._sorted:
                log.sort()
                self._sorted.add(ac_id)
                
    def get_poses_at_timestamp(self, timestamp: int|float) -> dict[int, Pose3D]:
        output = {}
        self.sort_logs()
        
        
        
        for ac_id, log in self.logs.items():
            try:
                poses = self._tpose_lists[ac_id]
            except KeyError:
                poses = []
                for d in log:
                    p = np.zeros(5,dtype=float)
                    p[0] = d.pose.x
                    p[1] = d.pose.y
                    p[2] = d.pose.z
                    p[3] = d.pose.theta
                    p[4] = d.timestamp
                    poses.append(p)
                poses = np.array(poses)
                self._tpose_lists[ac_id] = poses
            
            x = np.interp(timestamp, poses[:, -1], poses[:, 0])
            y = np.interp(timestamp, poses[:, -1], poses[:, 1])
            z = np.interp(timestamp, poses[:, -1], poses[:, 2])
            heading =np.interp(timestamp, poses[:, -1], poses[:, 3])
            output[ac_id] = Pose3D(x, y, z, heading)
        return output
    
    def get_min_sep_at_timestamp(self, timestamp: int|float) -> tuple[float,int,int]:
        poses = self.get_poses_at_timestamp(timestamp)
        ids_list = list(poses.keys())
        p_list = [poses[id] for id in ids_list]
        min_sep,i1,i2 = min_XY_dist(p_list)
        return min_sep, ids_list[i1], ids_list[i2]
    
    def get_all_min_sep(self) -> list[tuple[float|int,float,int,int]]:
        timestamps = sorted(self.get_all_timestamps())
        return [(t,*self.get_min_sep_at_timestamp(t)) for t in timestamps]
    
    def as_trajectories(self) -> DictOfPoseTrajectories:
        trajectories = DictOfPoseTrajectories()
        for ac_id, log in self.logs.items():
            trajectories[ac_id] = [data.to_timed_pose() for data in log]
        return trajectories

    def ref_trajectories(self) -> DictOfPoseTrajectories:
        trajectories = DictOfPoseTrajectories()
        for ac_id, log in self.logs.items():
            trajectories[ac_id] = [(data.timestamp, data.expected_pose) for data in log if data.expected_pose is not None]
        return trajectories