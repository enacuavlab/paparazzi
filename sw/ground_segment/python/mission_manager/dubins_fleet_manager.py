#!/usr/bin/env python3

from typing import Optional
import pathlib
import subprocess
import dataclasses
import tempfile
import datetime,time
import enum

import pyproj
from pyproj.enums import TransformDirection
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from Dubins import Pose3D,ACStats,FleetPlan,DubinsMove,BasicPath,Path,min_XY_dist,poses_XY_dist
from ioUtils import AC_PP_Problem, write_pathplanning_problem_to_CSV, straight_pp_problems,parse_trajectories_from_JSON
from Formation import Formation,chevron_formation
from FleetPath import FleetKeyframes,formation_oval

from uav_data import UAVData,TrackingData,TrackingLogs
from mission_manager import MissionManager,MissionInsert

from pprzlink.ivy import IvyMessagesInterface

from plotting import plot_several_pose2d_sequences,DictOfPoseTrajectories


PPRZ_DUBINS_TYPE_MAP = {
    'RSR' : 0,
    'LSL' : 1,
    'RSL' : 2,
    'LSR' : 3,
    'RLR' : 4,
    'LRL' : 5,
    'SLS' : 6,
    'SRS' : 7,

    'SRSR' : 8+0,
    'SLSL' : 8+1,
    'SRSL' : 8+2,
    'SLSR' : 8+3,
    'SRLR' : 8+4,
    'SLRL' : 8+5,
    'SSLS' : 8+6,
    'SSRS' : 8+7,

    'RSRS' : 2*8+0,
    'LSLS' : 2*8+1,
    'RSLS' : 2*8+2,
    'LSRS' : 2*8+3,
    'RLRS' : 2*8+4,
    'LRLS' : 2*8+5,
    'SLSS' : 2*8+6,
    'SRSS' : 2*8+7,

    'SRSRS' : 3*8+0,
    'SLSLS' : 3*8+1,
    'SRSLS' : 3*8+2,
    'SLSRS' : 3*8+3,
    'SRLRS' : 3*8+4,
    'SLRLS' : 3*8+5,
    'SSLSS' : 3*8+6,
    'SSRSS' : 3*8+7,
}

def solve_problem(solver:pathlib.Path,pb_loc:pathlib.Path,sol_loc:pathlib.Path,
                   separation:float, wind:tuple[float,float],threads:int,
                   **kwargs) -> subprocess.CompletedProcess[bytes]:
    """ Call the Dubins fleet path planner with the given arguments

    Args:
        solver (pathlib.Path): Path to the solver executable
        pb_loc (pathlib.Path): Problem definition path (CSV)
        sol_loc (pathlib.Path): Problem solution expected path (JSON)
        separation (float): Minimal required separation between aircraft
        wind (tuple[float,float]): Wind speed
        threads (int): Number of threads to use for solving

    Returns:
        subprocess.CompletedProcess[bytes]: 
    """
    cmd = []
    cmd.append(str(solver.resolve()))
    cmd.append(str(pb_loc.resolve()))
    cmd.append(str(sol_loc.resolve()))
    cmd.append(str(separation))
    
    cmd.append(str(wind[0]))
    cmd.append(str(wind[1]))
    
    cmd.append('-t')
    cmd.append(str(threads))
    
    cmd.append('-r')
    cmd.append(str(10))
    
    for k,v in kwargs.items():
        cmd.append(k)
        
        if v is None:
            continue
        
        cmd.append(str(v))
    
    return subprocess.run(cmd)

class FleetManagerEnd(enum.Enum):
    """ Possible final commands when the last keyposes are done:
    - LOOP: go back to first keyposes and loop forever
    - CIRCLE: add the same circle for every aircraft at the end
    - END_MISSION: send an END_MISSION message to every aircraft, let the flight plan take over
    - NOTHING: do nothing special
    - LAND: send a LAND command
    """
    LOOP        = 0
    CIRCLE      = 1
    END_MISSION = 2
    NOTHING     = 3
    LAND        = 4

class FleetManager:
    def __init__(self,
                 fleet_frames:FleetKeyframes,
                 solver_path:str|pathlib.Path,
                 separation:float,
                 flight_alt:float,
                 threads:int=0,
                 transformer:pyproj.Transformer=pyproj.Transformer.from_crs('WGS84','EPSG:9794'), # Default to WGS84 to Lambert93
                 ignore_wind:bool=True,
                 end_strategy:FleetManagerEnd=FleetManagerEnd.NOTHING,
                 speed_ctl:bool = False,
                 verbose:bool=False) -> None:
        """_summary_

        Args:
            solver_path (str | pathlib.Path): _description_
            acs (list[ACStats]): _description_
            separation (float): _description_
            threads (int, optional): _description_. Defaults to 0.
            transformer (_type_, optional): Transformation from WGS84 (GPS coordinates) to a flat coordinates system such that x is easting (in meters) and y is northing (in meters).
                Defaults to pyproj.Transformer.from_crs('WGS84','EPSG:9794'), i.e. GPS to Lambert93
        """
        ivy_interface = IvyMessagesInterface("PprzConnect")
        
        self.fleet_frames = fleet_frames
        self.__fleet_frame_id:int = 0
        self.__current_plan:Optional[FleetPlan] = None
        self.__current_plan_date:Optional[int|float] = None
        self.__next_plan_sent:bool = False
        self.__tracking_logs:TrackingLogs = TrackingLogs()
        self.__last_schedule_time:float = 0.
        self.__reschedule_dt:float = 1. # In seconds
        self.__relative_improvement_threshold:float = 0.15 # Minimum improvement in the plan duration to trigger a reschedule (in percentage of the current plan duration)
        self.end_strategy = end_strategy
        
        self.flight_alt:float = flight_alt
        self.speed_ctl = speed_ctl

        self.solver_path = pathlib.Path(solver_path)
        self.separation:float = separation
        self.threads:int = threads
        self.transformer:pyproj.Transformer = transformer
        
        self.ac_ids:list[int] = [s.id for s in self.fleet_frames.ac_stats]
        self.ac_ids.sort()
        self.acs:dict[int,ACStats] = dict()
        for s in self.fleet_frames.ac_stats:
            self.acs[s.id] = s
        self.managers:dict[int,MissionManager] = dict()
        for id in self.ac_ids:
            self.managers[id] = MissionManager(id,verbose,ivy_interface=ivy_interface)
        self.verbose = verbose
            
        self.ignore_wind:bool = ignore_wind
        self.wind_x:float = 0.
        self.wind_y:float = 0.
        
        self.mission_counter:int = 1
        
        assert self.fleet_frames.ac_num == len(self.ac_ids)
    
    def get_logs(self) -> TrackingLogs:
        return self.__tracking_logs
    
    def wait_ready(self,timeout:float=1):
        """Wait for all aircraft to be ready

        Args:
            timeout (float, optional): Timeout in seconds when waiting for READY message of each aircraft. Defaults to 1s.
        """
        is_ready:dict[int,bool] = dict()
        for id in self.ac_ids:
            is_ready[id] = False
        
        all_ready = False
        while not(all_ready):
            all_ready = True
            for id in self.ac_ids:
                if is_ready[id]: continue
                manager = self.managers[id]
                ready = manager.wait_ready(timeout)
                is_ready[id] = ready
                all_ready = all_ready and ready
    
    def takeoff(self,height:float):
        for mng in self.managers.values():
            mng.add_mission_takeoff(self.mission_counter, height=height, insert_mode=MissionInsert.REPLACE_ALL)
        self.__takeoff_height = height
        self.__takeoff_done = False
        self.mission_counter += 1
        
    def go_home_straight(self,insert_mode:MissionInsert=MissionInsert.REPLACE_ALL):
        for mng in self.managers.values():
            mng.go_home(self.mission_counter, insert=insert_mode)
        self.mission_counter += 1
        
    def circle_home(self,radii:Optional[list[float]]=None, insert_mode:MissionInsert=MissionInsert.REPLACE_ALL):
        if radii is not None:
            assert len(radii) == len(self.ac_ids)
        else:
            radii = [self.acs[id].turn_radius for id in self.ac_ids]
        
        for i,id in enumerate(self.ac_ids):
            radius = radii[i]
            mng = self.managers[id]
            mng.circle_home(self.mission_counter, radius, insert=insert_mode)
        self.mission_counter += 1
            
    def land(self,landpads:list[tuple[float,float,float]],insert_mode:MissionInsert=MissionInsert.REPLACE_ALL):
        assert len(self.ac_ids) == len(landpads)
        
        for i in range(len(self.ac_ids)):
            lat,lon,h = landpads[i]
            id = self.ac_ids[i]
            
            self.managers[id].add_mission_land(self.mission_counter,lat,lon,h,insert_mode=insert_mode)
        self.mission_counter += 1
        
    def land_here(self,ground_alt:float):
        """ Every aircraft land at their current location
        """
        for mng in self.managers.values():
            lat = mng.uav_data.lat
            lon = mng.uav_data.lon
            mng.add_mission_land(self.mission_counter,lat,lon,ground_alt,insert_mode=MissionInsert.REPLACE_ALL)
        self.mission_counter += 1
    
    def start_mission(self):
        """Go to mission block for every aircraft
        """
        for mng in self.managers.values():
            mng.start_mission()
            
    def closing(self):
        """Close Ivy for all mission managers
        """
        for mng in self.managers.values():
            mng.closing()
    
    def main_loop(self):
        while True:
            self.__update_wind()
            poses,times = self.get_poses()
            min_time = min(times.values())
            max_time = max(times.values())
            
            if self.__current_plan is not None and self.__current_plan_date is not None:
                
                if self.__current_plan.duration < 3.:
                    self.__current_plan = None
                    self.__fleet_frame_id += 1
                else:
                    try:
                        self.__current_plan = self.__current_plan.follow_for(max_time-self.__current_plan_date)
                        self.__current_plan_date = max_time
                        for i,id in enumerate(self.ac_ids):
                            _,p = self.__current_plan.get_path(id)
                            t_data = TrackingData(id, times[id], poses[i],p.start)
                            self.__tracking_logs.add_tracking_data(t_data)
                    except KeyError:
                        self.__current_plan = None
                        self.__fleet_frame_id += 1
                        pass
                
            
            if self.verbose:
                val,id1,id2 = min_XY_dist(poses)
                print(f"Minimal current dist is {val}, measured between {self.ac_ids[id1]} and {self.ac_ids[id2]}")
                print(f"(Minimal required is {self.separation})")
                print(f"Max time difference between aircraft is {max_time - min_time} seconds")
                
            # current_missions = self.get_current_mission_ids()
            # try:
            #     current_max_mission_id = max(ids[0] for ids in current_missions.values())
            #     mission_diff_index = current_max_mission_id - 1
            #     self.__fleet_frame_id += mission_diff_index
            # except IndexError:
            #     self.__fleet_frame_id += 1
            

            if self.end_strategy is FleetManagerEnd.LOOP:
                self.__fleet_frame_id = self.__fleet_frame_id % self.fleet_frames.keyposes_num
            else:
                if self.__fleet_frame_id >= self.fleet_frames.keyposes_num:
                    print("Done with fleet frames. Sending last command and stopping")
                    if self.end_strategy is FleetManagerEnd.CIRCLE:
                        self.circle_home()
                    return 
            
            try:
                if (max_time - self.__last_schedule_time) > self.__reschedule_dt or self.__current_plan is None:
                    impr_threshold = None 
                    if self.__current_plan is not None:
                        impr_threshold = self.__current_plan.duration * self.__relative_improvement_threshold
                    self.__schedule(poses, max_time, improvement_threshold=impr_threshold)
                    self.__last_schedule_time = max_time
            except FileNotFoundError:
                print("WARNING: Solver did not find a solution")
                
            time.sleep(0.1)

    def __schedule(self, current_poses:list[Pose3D], current_time:float, improvement_threshold:Optional[float]=None):
        pb = self.fleet_frames.generate_pb_to(self.__fleet_frame_id,current_poses)
        sol = self.__solve_pp_problem(pb)
        
        if improvement_threshold is not None and improvement_threshold < sol.duration:
            # If the threshold is not met, do not apply new plan
            return
        
        self.mission_counter = 1
        self.__send_fleet_mission(sol,current_time,insert_mode=MissionInsert.REPLACE_ALL)
        self.__current_plan = sol
        self.__current_plan_date = current_time
        end_tow = current_time + sol.duration
        
        
        if not(self.__next_plan_sent):
            j = self.__fleet_frame_id + 1
            j_plus_one = j+1
            if self.end_strategy is FleetManagerEnd.LOOP:
                j = j % self.fleet_frames.keyposes_num
                j_plus_one = j_plus_one % self.fleet_frames.keyposes_num
            else:
                if j_plus_one >= self.fleet_frames.keyposes_num:
                    self.__next_plan_sent = True
                    return
                    
            pb = self.fleet_frames.generate_pb_between(j,j_plus_one)
            if straight_pp_problems(pb,1e-3):
                self.__send_direct_path_missions(pb,MissionInsert.APPEND)
                end_tow += poses_XY_dist(pb[0].start,pb[0].end)/pb[0].stats.airspeed
                self.__next_plan_sent = True
            else:
                sol = self.__solve_pp_problem(pb)
                self.__send_fleet_mission(sol,end_tow,MissionInsert.APPEND)
                end_tow += sol.duration
                self.__next_plan_sent = True
    
        
    def __update_wind(self):
        if self.ignore_wind:
            return
        
        avg_east_wind = 0.
        avg_north_wind = 0.
        n = 0
        for val in self.managers.values():
            if val.uav_data.wind_east is None or val.uav_data.wind_north is None:
                continue
            else:
                avg_east_wind += val.uav_data.wind_east
                avg_north_wind += val.uav_data.wind_north
                n += 1
        if n != 0:
            avg_east_wind /= n
            avg_north_wind /= n
            
            self.wind_x = avg_east_wind
            self.wind_y = avg_north_wind
        
    
    def uav_data(self,id:int) -> UAVData:
        return self.managers[id].uav_data
    
    def update_stats(self,id:int):
        """ Only update airspeed; the other characteristics are limitations which are not readily available

        Args:
            id (int): Id of the aircraft to update its statistics
        """
        data = self.uav_data(id)
        stats = self.acs[id]
        stats.airspeed = data.airspeed_sp
    
    def update_all_stats(self):
        for id in self.ac_ids:
            self.update_stats(id)
    
    def get_poses(self) -> tuple[list[Pose3D],dict[int,float]]:
        """Return the list of currently known aircraft poses, with their associated onboard timestamps

        Returns:
            tuple[list[Pose3D],dict[int,float]]: First list contain the poses, the second the senders' timestamp (GPS ToW, in seconds) indexed by ac_id
        """
        output_poses = []
        output_times = dict()
        for id in self.ac_ids:
            mng = self.managers[id]
            data = mng.uav_data
            xx, yy = self.transformer.transform(data.lat,data.lon)
            pose = Pose3D(xx,yy,data.alt,np.pi/2-np.deg2rad(data.heading))
            output_poses.append(pose)
            output_times[id] = data.gps_tow
        return output_poses,output_times
    
    def get_current_mission_ids(self) -> dict[int,list[int]]:
        output_mission_ids = dict()
        for id in self.ac_ids:
            mng = self.managers[id]
            output_mission_ids[id] = mng.uav_data.mission_status
        return output_mission_ids
    
    def __make_pp_problems(self,poses:list[Pose3D],dests:list[Pose3D]) -> list[AC_PP_Problem]:
        assert len(dests) == len(self.ac_ids)
        assert len(poses) == len(self.ac_ids)
        
        output = []
        for i in range(len(dests)):
            start = poses[i]
            end = dests[i]
            id = self.ac_ids[i]
            stats = self.acs[id]
            output.append(AC_PP_Problem(stats,start,end))
        return output
    
    def __solve_pp_problem(self,pb:list[AC_PP_Problem]) -> FleetPlan:
        tempdir = tempfile.gettempdir()
        now = datetime.datetime.now()
        input_file = pathlib.Path(tempdir) / f"{now.strftime('%y-%m-%d_%H:%M:%S.%f')}_input.csv"
        output_file = pathlib.Path(tempdir) / f"{now.strftime('%y-%m-%d_%H:%M:%S.%f')}_output.json"
        write_pathplanning_problem_to_CSV(input_file,pb)
        try:
            solve_problem(
                self.solver_path,
                input_file,
                output_file,
                self.separation,
                (0.,0.) if self.ignore_wind else (self.wind_x,self.wind_y),
                self.threads,
            )
                
            ## Parse the result and merge
            return parse_trajectories_from_JSON(output_file)
            
        except Exception as e:
            print(f"EXCEPTION: {e}")
            raise e
    
    def __send_direct_path_missions(self,pbs:list[AC_PP_Problem],insert_mode:MissionInsert):
        for pb in pbs:
            stats = pb.stats
            id = stats.id
            manager = self.managers[id]
            
            home = manager.get_home()
            assert home is not None
            xhome,yhome = self.transformer.transform(home.lat,home.lon)
            xstart = pb.start.x - xhome
            ystart = pb.start.y - yhome
            
            xend = pb.end.x - xhome
            yend = pb.end.y - yhome
            
            manager.add_mission_local_path(self.mission_counter,[(xstart,ystart),(xend,yend)],pb.end.z,insert_mode=insert_mode)
        self.mission_counter += 1
            
    
    def __send_goto_missions(self,dests:dict[int,Pose3D],insert_mode:MissionInsert):
        for k,v in dests.items():
            manager = self.managers[k]
            
            home = manager.get_home()
            assert home is not None
            xhome,yhome = self.transformer.transform(home.lat,home.lon)
            xdest = v.x - xhome
            ydest = v.y - yhome
            
            manager.add_mission_local_point(self.mission_counter,xdest,ydest,v.z,insert_mode=insert_mode)
        self.mission_counter += 1
    
    def __send_fleet_mission(self,plan:FleetPlan,times:dict[int,float]|float,insert_mode:MissionInsert):
        mission_id_incr = 0
        for stats,path in plan.trajectories:
            t = times[stats.id] if isinstance(times,dict) else times
            incr = self.__make_path_mission(stats.id,path,t,insert_mode)
            mission_id_incr = max(mission_id_incr,incr)
        self.mission_counter += mission_id_incr
        
    def __make_path_mission(self,ac_id:int,p:Path,start_time:float,insert_mode:MissionInsert) -> int:
        manager = self.managers[ac_id]
        speed = manager.uav_data.groundspeed_sp if self.ignore_wind else manager.uav_data.airspeed_sp
        if speed == 0.: # Speed set point is not known
            for e in self.fleet_frames.ac_stats:
                if e.id == ac_id:
                    speed = e.airspeed
            assert speed != 0.
        
        home = manager.get_home()
        assert home is not None
        xhome,yhome = self.transformer.transform(home.lat,home.lon)
        
        params = [
            p.start.x-xhome,
            p.start.y-yhome,
            p.start.theta,
            p.end.x-xhome,
            p.end.y-yhome,
            p.end.theta,
            p.end.z,
            start_time,
            start_time+p.total_length/speed if self.speed_ctl else 0,
            float(PPRZ_DUBINS_TYPE_MAP[p.abbr()]),
            p.max_turn_radius(),
            p.extra_length()
        ]
        
        print(f"{ac_id} : params: {params}")
        
        manager.add_mission_custom(
            self.mission_counter,
            'DUBIN',
            params,
            insert_mode=insert_mode
        )

        return 1
            

if __name__ == '__main__':
    import argparse
    
    color_dict = {
        17 : 'red',
        18 : 'green',
        19 : 'yellow'
    }
    
    stat_list = [ACStats(i,15,10/60,50) for i in [17,18,19]]
    separation = 30
    formation = chevron_formation(len(stat_list),separation*1.2)
    
    transformer = pyproj.Transformer.from_crs('WGS84','EPSG:9794') # Default to WGS84 to Lambert93
    home_lat = 43.46223
    home_lon = 1.27289
    home_height = 260-185
    home_alt = 260
    home_x,home_y = transformer.transform(home_lat,home_lon)
    
    start = Pose3D(home_x,home_y,home_alt,0.)
    fleet_plan = formation_oval(stat_list,start,250,200,formation)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('dubins_solver',help='Path to Dubins Fleet Planner')
    parser.add_argument('-v',action='store_true',help='Verbose',default=False)
    parser.add_argument('-t','--takeoff',action='store_true',help='Send take off mission order first')
    parser.add_argument('--speed-ctl',action='store_true',dest='speed_ctl',help='If set, enable speed control for aircraft')
    
    args = parser.parse_args()
    
    manager = FleetManager(
        fleet_plan,
        args.dubins_solver,
        separation,
        home_alt,
        transformer=transformer,
        end_strategy=FleetManagerEnd.LOOP,
        speed_ctl=args.speed_ctl,
        verbose=args.v
    )
    
    try:
        manager.wait_ready()
        if args.takeoff:
            print("Take off requested!")
            manager.takeoff(home_height)
        manager.circle_home(insert_mode=MissionInsert.APPEND)
        manager.start_mission()
        input("Press any key to start fleet path planning...")
        manager.main_loop()
    except(KeyboardInterrupt,SystemExit):
        print("Intettupted!")
    finally:
        print("Closing")
        manager.closing()
        trajs = manager.get_logs().as_trajectories()
        refs = manager.get_logs().ref_trajectories()
        errs = manager.get_logs().get_all_min_sep()
    
    fig,axes = plt.subplots(2,1)
    
    ax = axes[0]
    ax.set_aspect('equal')
    plot_several_pose2d_sequences(ax,trajs,color_dict,
        label=True,endpoints=True)
    _,d = plot_several_pose2d_sequences(ax,refs,color_dict,
        label=True,endpoints=True,linestyle='dashed')
    for e in d.values():
        l = e.get_label()
        e.set_label(l + " (ref)")
    
    ax.legend()
    
    err_ax:Axes = axes[1]
    ts = [t[0] for t in errs]
    vals = [t[1] for t in errs]
    err_ax.plot(ts,vals,label=f"{id}")
    err_ax.set_xlabel("Time")
    err_ax.set_ylabel("Minimum Separation")
    err_ax.hlines(separation,ts[0],ts[-1],colors='k',linestyles='dashed',label='Required Separation')
    err_ax.legend()
    plt.show()
    