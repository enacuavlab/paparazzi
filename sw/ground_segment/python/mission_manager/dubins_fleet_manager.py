#!/usr/bin/env python3

import typing
import pathlib
import subprocess
import dataclasses
import tempfile
import datetime,time
import enum

import pyproj
from pyproj.enums import TransformDirection
import numpy as np

from Dubins import Pose3D,ACStats,FleetPlan,DubinsMove,BasicPath,Path
from ioUtils import AC_PP_Problem, write_pathplanning_problem_to_CSV, parse_pathplanning_problem_from_CSV,parse_trajectories_from_JSON
from Formation import Formation,chevron_formation
from FleetPath import FleetKeyframes,formation_oval

from mission_manager import UAVData,MissionManager,MissionInsert

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig

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
                 threads:int=0,
                 transformer:pyproj.Transformer=pyproj.Transformer.from_crs('WGS84','EPSG:9794'), # Default to WGS84 to Lambert93
                 ignore_wind:bool=True,
                 end_strategy:FleetManagerEnd=FleetManagerEnd.NOTHING,
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
        self.fleet_frames = fleet_frames
        self.__fleet_frame_id:int = 0
        self.end_strategy = end_strategy
        
        self.__takeoff_done:bool = True
        self.__takeoff_alt:float = 0.
        
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
            self.managers[id] = MissionManager(id,verbose)
        self.verbose = verbose
            
        self.ignore_wind:bool = ignore_wind
        self.wind_x:float = 0.
        self.wind_y:float = 0.
        
        self.mission_counter:int = 1
        
        assert self.fleet_frames.ac_num == len(self.ac_ids)
    
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
    
    def takeoff(self,altitude:float):
        for mng in self.managers.values():
            mng.add_mission_takeoff(self.mission_counter, altitude, insert_mode=MissionInsert.REPLACE_ALL)
        self.__takeoff_alt = altitude
        self.__takeoff_done = False
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
            poses = self.get_poses()
            
            if not(self.__takeoff_done):
                takeoff_done = True
                for p in poses:
                    if p.z < self.__takeoff_alt * 0.95:
                        takeoff_done = False
                        break
                self.__takeoff_done = takeoff_done
                if self.verbose and takeoff_done:
                    print("Takeoff done!")
            else:
                pb = self.fleet_frames.generate_pb_to(self.__fleet_frame_id,poses)
                sol = self.__solve_pp_problem(pb)
                self.__send_fleet_mission(sol)
            time.sleep(5)
            
        
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
    
    def get_poses(self) -> list[Pose3D]:
        output = []
        for id in self.ac_ids:
            data = self.uav_data(id)
            xx, yy = self.transformer.transform(data.lat,data.lon)
            pose = Pose3D(xx,yy,data.alt,data.heading)
            output.append(pose)
        return output
    
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
        
    def __send_fleet_mission(self,plan:FleetPlan):
        mission_id_incr = 0
        for stats,path in plan.trajectories:
            incr = self.__make_path_mission(stats.id,path)
            mission_id_incr = max(mission_id_incr,incr)
        self.mission_counter += mission_id_incr
        
    def __make_path_mission(self,ac_id:int,p:Path) -> int:
        manager = self.managers[ac_id]
        speed = manager.uav_data.groundspeed_sp if self.ignore_wind else manager.uav_data.airspeed_sp
        first = True
        for i,s in enumerate(p.sections):
            this_mission_id = self.mission_counter + i
            if first:
                first = False
                if s.type == DubinsMove.STRAIGHT:
                    end = s.end()
                    lat,lon = self.transformer.transform(end.x,end.y,direction=TransformDirection.INVERSE)
                    manager.add_mission_point(lat,lon,end.z,this_mission_id,s.length/speed,insert=MissionInsert.REPLACE_ALL)
                    continue
                
            if s.type == DubinsMove.STRAIGHT:
                start = s.start()
                slat,slon = self.transformer.transform(start.x,start.y,direction=TransformDirection.INVERSE)
                end = s.end()
                elat,elon = self.transformer.transform(end.x,end.y,direction=TransformDirection.INVERSE)
                manager.add_mission_path(this_mission_id,
                                         [(slat,slon),(elat,elon)],
                                         end.z,s.length/speed,
                                         MissionInsert.REPLACE_ALL if first else MissionInsert.APPEND)
            else:
                lat,lon = self.transformer.transform(s.x,s.y,direction=TransformDirection.INVERSE)
                radius = (1 if s.type == DubinsMove.RIGHT else -1) * s.radius() # Radius is positive if turning clockwise, negative otherwise
                manager.add_mission_circle(this_mission_id,
                                           lat,lon,s.z,
                                           radius,
                                           s.length/speed,
                                           MissionInsert.REPLACE_ALL if first else MissionInsert.APPEND)
        return len(p.sections)
            

if __name__ == '__main__':
    import argparse
    
    stat_list = [ACStats(i,15,10/60,40) for i in [2,3,4]]
    separation = 80
    formation = chevron_formation(len(stat_list),separation*1.2)
    
    transformer = pyproj.Transformer.from_crs('WGS84','EPSG:9794') # Default to WGS84 to Lambert93
    home_lat = 48.8652734
    home_lon = 1.8928199
    home_alt = 40
    home_x,home_y = transformer.transform(home_lat,home_lon)
    
    start = Pose3D(home_x,home_y,home_alt,0.)
    fleet_plan = formation_oval(stat_list,start,400,100,formation)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('dubins_solver',help='Path to Dubins Fleet Planner')
    
    args = parser.parse_args()
    
    manager = FleetManager(
        fleet_plan,
        args.dubins_solver,
        separation,
        transformer=transformer,
        end_strategy=FleetManagerEnd.NOTHING
    )
    
    try:
        manager.wait_ready()
        manager.takeoff(home_alt)
        manager.start_mission()
        manager.main_loop()
    except(KeyboardInterrupt,SystemExit):
        print("Intettupted!")
        manager.closing()
    finally:
        print("Closing")
        manager.closing()
    