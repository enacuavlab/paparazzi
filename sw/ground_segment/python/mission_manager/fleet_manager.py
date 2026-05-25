#!/usr/bin/env python3

from typing import Optional
import pathlib
import subprocess
import os
import pickle,copy
import tempfile
import datetime,time
import enum,dataclasses
import asyncio

import pyproj
import numpy as np

import matplotlib.pyplot as plt

from Dubins import Pose3D,ACStats,FleetPlan,DubinsMove,BasicPath,Path,min_XY_dist,poses_XY_dist,path_extra_length
from ioUtils import AC_PP_Problem, write_pathplanning_problem_to_CSV, parse_trajectories_from_JSON
from Formation import Formation,chevron_formation
from FleetPath import FleetKeyframes,plot_keyframes,formation_oval,formation_rectangle

from uav_data import UAVData,TrackingData,TrackingLogs
from mission_manager import MissionManager,MissionInsert,send_and_ack_msg,PprzMessage
from tracking_logs_analyser import plot_trackingdata

from pprzlink.ivy import IvyMessagesInterface

DEBUG = True

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

def _make_pb_cmd(solver:pathlib.Path,pb_loc:pathlib.Path,sol_loc:pathlib.Path,
                   separation:float, wind:tuple[float,float],threads:int,
                   start_extensions:list[float] = [],
                   end_extensions:list[float] = [],
                   **kwargs) -> list[str]:
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
    
    cmd.append('-l')
    
    if len(start_extensions) > 0:
        cmd.append('--straights-only')
        cmd.append('--extend-start')
        for e in start_extensions:
            cmd.append(str(e))
    
    if len(end_extensions) > 0:
        cmd.append('--extend-end')
        for e in end_extensions:
            cmd.append(str(e))
    
    for k,v in kwargs.items():
        cmd.append(k)
        
        if v is None:
            continue
        
        cmd.append(str(v))
    return cmd

def solve_problem(solver:pathlib.Path,pb_loc:pathlib.Path,sol_loc:pathlib.Path,
                   separation:float, wind:tuple[float,float],threads:int,
                   start_extensions:list[float] = [],
                   end_extensions:list[float] = [],
                   **kwargs) -> subprocess.CompletedProcess[bytes]:
    """ Call the Dubins fleet path planner with the given arguments

    Args:
        solver (pathlib.Path): Path to the solver executable
        pb_loc (pathlib.Path): Problem definition path (CSV)
        sol_loc (pathlib.Path): Problem solution expected path (JSON)
        separation (float): Minimal required separation between aircraft
        wind (tuple[float,float]): Wind speed
        threads (int): Number of threads to use for solving
        start_extensions (list[float], optional): List of start extensions. Defaults to [].
        end_extensions (list[float], optional): List of end extensions. Defaults to [].

    Returns:
        subprocess.CompletedProcess[bytes]: 
    """
    cmd = _make_pb_cmd(solver, pb_loc, sol_loc, separation, wind, threads, start_extensions, end_extensions, **kwargs)
    
    return subprocess.run(cmd,stdout=subprocess.DEVNULL)

def subprocess_solve_problem(solver:pathlib.Path,
                             pb_loc:pathlib.Path,sol_loc:pathlib.Path,
                             separation:float, wind:tuple[float,float],threads:int,
                             start_extensions:list[float] = [],
                             end_extensions:list[float] = [],
                             **kwargs) -> subprocess.Popen:
    
    cmd = _make_pb_cmd(solver, pb_loc, sol_loc, separation, wind, threads, start_extensions, end_extensions, **kwargs)
    
    return subprocess.Popen(cmd,stdout=subprocess.DEVNULL)

async def async_subprocess_solve(solver:pathlib.Path,
                             pb_loc:pathlib.Path,sol_loc:pathlib.Path,
                             separation:float, wind:tuple[float,float],threads:int,
                             start_extensions:list[float] = [],
                             end_extensions:list[float] = [],
                             **kwargs) -> int:
    # Create command to run
    cmd = _make_pb_cmd(solver, pb_loc, sol_loc, separation, wind, threads, start_extensions, end_extensions, **kwargs)

    # Create subprocess (asynchronously)
    proc = await asyncio.create_subprocess_exec(*cmd,stdout=asyncio.subprocess.DEVNULL)
    
    # Wait for it to finish and return the subprocess return code
    retcode = await proc.wait()
    return retcode
    
@dataclasses.dataclass
class SolverInstance:
    process:subprocess.Popen
    start_time:float
    improvement:float
    result_loc:pathlib.Path
    
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
                 full_dubins:bool = False,
                 tracking_error:float = 20,
                 verbosity:int=0) -> None:
        """_summary_

        Args:
            solver_path (str | pathlib.Path): _description_
            acs (list[ACStats]): _description_
            separation (float): _description_
            threads (int, optional): _description_. Defaults to 0.
            transformer (_type_, optional): Transformation from WGS84 (GPS coordinates) to a flat coordinates system such that x is easting (in meters) and y is northing (in meters).
                Defaults to pyproj.Transformer.from_crs('WGS84','EPSG:9794'), i.e. GPS to Lambert93
        """
        self.ivy_interface = IvyMessagesInterface("PprzConnect")
        
        self.fleet_frames = fleet_frames
        self.__fleet_frame_id:int = 0
        self.__current_timestamp_plan:Optional[tuple[FleetPlan,float]] = None
        self.end_of_plan_carrot:float = 10. # In seconds, how much time before the end of the current plan to send the next one
        self.reschedule_dt:float = 15. # In seconds
        self.relative_improvement_threshold:float = 0.9 # Minimum improvement in the plan duration to trigger a reschedule (in percentage of the current plan duration)
        self.__tracking_error:float = 0.    # Current maximal individual tracking error (Try an other metric, like cumulated error accross aircraft?)
        self.__tracking_err_threshold:float = tracking_error # In meters, if the tracking error is above this threshold, consider the plan as not followed and trigger a reschedule
        self.end_strategy = end_strategy
        self.full_dubins = full_dubins
        self.__schedule_time = 0
        
        self.msg_retry:int      = 5
        self.msg_ack_time:float = 1
        
        self.flight_alt:float = flight_alt
        self.speed_ctl = speed_ctl
        self.extra_straight_length = 120 # In meters #TODO: Synchronize with matching DL_SETTING

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
            self.managers[id] = MissionManager(id,verbosity>1,ivy_interface=self.ivy_interface)
        self.verbosity = verbosity
            
        self.ignore_wind:bool = ignore_wind
        self.wind_x:float = 0.
        self.wind_y:float = 0.
        
        self.mission_counter:int = 1
        
        assert self.fleet_frames.ac_num == len(self.ac_ids)
        
        self.__tracking_logs:TrackingLogs = TrackingLogs(
            fleet_frames,
            tracking_error_threshold=self.__tracking_err_threshold,
            separation_threshold=self.separation)

    
    #################### Util methods ####################
    
    def get_logs(self) -> TrackingLogs:
        return self.__tracking_logs
    
    
    def get_timed_poses(self) -> dict[int,tuple[float,Pose3D]]:
        """Return the list of currently known aircraft poses, with their associated onboard timestamps

        Returns:
            tuple[list[Pose3D],dict[int,float]]: First list contain the poses, the second the senders' timestamp (GPS ToW, in seconds) indexed by ac_id
        """
        output = dict()
        for id in self.ac_ids:
            mng = self.managers[id]
            data = mng.uav_data
            pose = self.__uavdata_to_pose(data)
            output[id] = (data.gps_tow,pose)
        return output
    
    def __latlon_to_pose(self,lat:float,lon:float,alt:float,heading_deg:float) -> Pose3D:
        """ Convert from latlon heading to projected coordinates using register transform

        Args:
            lat (float): Latitude
            lon (float): Longitude
            alt (float): Altitude above MSL (meters)
            heading_deg (float): Heading in degrees, such that 0° is headed North, 90° is East

        Returns:
            Pose3D: Pose in the local frame
        """
        xx, yy = self.transformer.transform(lat,lon)
        pose = Pose3D(xx,yy,alt,np.pi/2-np.deg2rad(heading_deg))
        return pose
    
    def __uavdata_to_pose(self,data:UAVData) -> Pose3D:
        return self.__latlon_to_pose(data.lat,data.lon,data.alt,data.heading)
    
    def interpolate_poses(self,at:float) -> dict[int,Pose3D]:
        """Return the interpolated positions of the tracked aircraft based on a simple linear interpolation (assume constant heading)

        Args:
            at (float): Time (as GPS ToW, in seconds)

        Returns:
            dict[int,Pose3D]: Estimated aircraft positions
        """
        output = dict()
        for id in self.ac_ids:
            mng = self.managers[id]
            data = mng.uav_data
            xx, yy = self.transformer.transform(data.lat,data.lon)
            tref = data.gps_tow
            dt = at - tref
            pose = Pose3D(xx+data.veast*dt,yy+data.vnorth,data.alt+data.vup,np.pi/2-np.deg2rad(data.heading))
            output[id] = pose
        return output
    
    def get_current_mission_ids(self) -> dict[int,list[int]]:
        output_mission_ids = dict()
        for id in self.ac_ids:
            mng = self.managers[id]
            output_mission_ids[id] = mng.uav_data.mission_status
        return output_mission_ids
    
    async def __send_managers_messages_with_ack(self,mng_msg:list[tuple[MissionManager,PprzMessage]],retry:int,ack_time:float):
        coros = [send_and_ack_msg(el[0],el[1],retry=retry,ack_time=ack_time) for el in mng_msg]
        await asyncio.gather(*coros)
    
    #################### Synchronous orders ####################
    
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
        mng_msg = []
        for mng in self.managers.values():
            mng_msg.append((mng,mng.make_mission_takeoff(self.mission_counter, height=height, insert_mode=MissionInsert.REPLACE_ALL)))

        asyncio.run(self.__send_managers_messages_with_ack(mng_msg,self.msg_retry,self.msg_ack_time),debug=DEBUG)
                
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

        mng_msg = []
        
        for i in range(len(self.ac_ids)):
            lat,lon,h = landpads[i]
            id = self.ac_ids[i]
            
            msg = self.managers[id].make_mission_land(self.mission_counter,lat,lon,h,insert_mode=insert_mode)
            mng_msg.append((self.managers[id],msg))
        self.mission_counter += 1

        asyncio.run(self.__send_managers_messages_with_ack(mng_msg,self.msg_retry,self.msg_ack_time),debug=DEBUG)
        
        
    def land_here(self,ground_alt:float):
        """ Every aircraft land at their current location
        """

        mng_msg = []
        for mng in self.managers.values():
            lat = mng.uav_data.lat
            lon = mng.uav_data.lon
            msg = mng.make_mission_land(self.mission_counter,lat,lon,ground_alt,insert_mode=MissionInsert.REPLACE_ALL)
            mng_msg.append((mng,msg))
        self.mission_counter += 1

        asyncio.run(self.__send_managers_messages_with_ack(mng_msg,self.msg_retry,self.msg_ack_time),debug=DEBUG)
    
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
    
    #################### Main async loop ####################
    
    async def main(self):
        await asyncio.gather(
            self.log_data_loop(),
            self.fleet_ctrl_loop()
        )
    
    
    async def fleet_ctrl_loop(self,dt:float=0.1):
        assert dt >= 0, "Waiting time must be nonnegative!"
        while True:
            now = time.time()
            end = now + dt
            await self.fleet_ctrl()
            wait = end - time.time()
            
            # Yield control only if there is some time to wait
            if wait > 0:
                await asyncio.sleep(wait)
                
    async def fleet_ctrl(self):
        timed_poses = self.get_timed_poses()
        max_time = max(t[0] for t in timed_poses.values())
        
        ### Progress plan
        if self.__current_timestamp_plan is not None:
            curr_plan,curr_plan_date = self.__current_timestamp_plan
            updated_date = max_time
            updated_plan = curr_plan.follow_for(max_time-curr_plan_date)
            self.__current_timestamp_plan = (updated_plan,updated_date)

        ### Check for reschuling
        reschule = False
        impr_threshold = np.inf
        
        ## If no plan running, schedule one
        if self.__current_timestamp_plan is None:
            reschule = True
            if self.verbosity > 0:
                print("Scheduling: no plan")
        else:
            ## Otherwise, inspect current situation
            curr_plan,curr_plan_date = self.__current_timestamp_plan
            
            ## If the end of plan is near, reschedule for the next step
            if curr_plan.duration < self.end_of_plan_carrot:
                reschule = True
                self.__fleet_frame_id += 1
                if self.verbosity > 0:
                    print("Scheduling: end of plan")
            else:
                ## Check for dynamic reschudling
                if time.time() - self.__schedule_time > self.reschedule_dt:
                    ## If tracking error, reschedule no matter what
                    if self.__tracking_error > self.__tracking_err_threshold:
                        reschule = True
                        if self.verbosity > 0:
                            print("Scheduling: tracking error plan")
                    else:
                        ## Otherwise, reschule only if significant improvement
                        reschule = True
                        impr_threshold = curr_plan.duration * self.relative_improvement_threshold
                        if self.verbosity > 0:
                            print("Scheduling: try to improve")
            
        
            
        poses = self.interpolate_poses(max_time)
        l_poses = [poses[id] for id in self.ac_ids]
        
        if reschule:
            await self.schedule_path_planning(self.__fleet_frame_id+1,l_poses,max_time,impr_threshold)
                
    ########## Logging ##########
        
    async def log_data(self):        
        avg_east_wind = 0.
        avg_north_wind = 0.
        n = 0
        
        if self.__current_timestamp_plan is not None:
            current_plan,current_plan_date = self.__current_timestamp_plan
            
            max_tracking_error_id   = 0
            max_tracking_error_val  = 0
            
        else:
            current_plan,current_plan_date = None,None
            max_tracking_error_id   = 0
            max_tracking_error_val  = 0
            
        
        for id,mng in self.managers.items():
            # Position logging
            pose = self.__uavdata_to_pose(mng.uav_data)
            t = mng.uav_data.gps_tow
            expected_speed = self.acs[id].airspeed
            
            # Reference position logging
            if current_plan is None or current_plan_date is None:
                tracking_data = TrackingData(id, t, pose, mng.uav_data.airspeed, None, expected_speed)
            else:
                _,p = current_plan.get_path(id)
                ref_pose = p.pose_at(t-current_plan_date)
                tracking_data = TrackingData(id, t, pose, mng.uav_data.airspeed, ref_pose, expected_speed)
                dist = poses_XY_dist(pose,ref_pose)
                if dist > max_tracking_error_val:
                    max_tracking_error_val = dist
                    max_tracking_error_id = id

            self.__tracking_logs.add_tracking_data(tracking_data)
            
            # Wind update
            if not(self.ignore_wind) and mng.uav_data.wind_east is not None and mng.uav_data.wind_north is not None:
                avg_east_wind   += mng.uav_data.wind_east
                avg_north_wind  += mng.uav_data.wind_north
                n += 1
            
        if not(self.ignore_wind) and n > 0:            
            self.wind_x = avg_east_wind/n
            self.wind_y = avg_north_wind/n
            
        if self.verbosity > 0:
            if current_plan is None:
                print(f"No current plan!")
            else:
                print(f"Tracking error detected for {max_tracking_error_id}: {max_tracking_error_val:.2f}")
                
        self.__tracking_error = max_tracking_error_val
        
    async def log_data_loop(self,dt:float=0.1):
        assert dt >= 0, "Waiting time must be nonnegative!"
        while True:
            now = time.time()
            end = now + dt
            await self.log_data()
            wait = end - time.time()
            
            # Yield control only if there is some time to wait
            if wait > 0:
                await asyncio.sleep(wait)
    
    ########## Scheduling ##########
    
    async def schedule_path_planning(self,frame_id:int,current_poses:list[Pose3D], current_time:float, improvement_threshold:float):
        
        if self.end_strategy is FleetManagerEnd.LOOP:
            frame_id = frame_id % self.fleet_frames.keyposes_num
        else:
            if frame_id >= self.fleet_frames.keyposes_num:
                raise StopIteration("No more fleet instruction to perform")
        
        pb = self.fleet_frames.generate_pb_to(frame_id,current_poses)
        
        tempdir = tempfile.gettempdir()
        now = datetime.datetime.now()
        input_file = pathlib.Path(tempdir) / f"{now.strftime('%y-%m-%d_%H:%M:%S.%f')}_input.csv"
        output_file = pathlib.Path(tempdir) / f"{now.strftime('%y-%m-%d_%H:%M:%S.%f')}_output.json"
        write_pathplanning_problem_to_CSV(input_file,pb)
        
        r = await async_subprocess_solve(
                self.solver_path,
                input_file,
                output_file,
                self.separation,
                (0.,0.) if self.ignore_wind else (self.wind_x,self.wind_y),
                self.threads,
                [self.extra_straight_length] if self.extra_straight_length > 0 else [],
                [self.extra_straight_length] if self.extra_straight_length > 0 else []
            )
        if r == 0:
            sol = parse_trajectories_from_JSON(output_file)
            
            if self.verbosity > 0:
                print(f"New plan duration is {sol.duration:.2f} s (threshold is {improvement_threshold:.2f} s)")
            
            if improvement_threshold < sol.duration:
                # If the threshold is not met, do not apply new plan
                return False
            
            await self.__send_fleet_mission(sol,current_time,insert_mode=MissionInsert.REPLACE_ALL)
            self.__tracking_logs.replanning_timestamps.append(current_time)
            self.__schedule_time = time.time()
            self.__current_timestamp_plan = (sol,current_time)
            
            return True
        else:
            print(f"Solver error! Code: {r}")
            return False        

    
    async def __send_fleet_mission(self,plan:FleetPlan,times:dict[int,float]|float,insert_mode:MissionInsert):
        mng_msg = []
        mission_id_incr = 0
        for stats,path in plan.trajectories:
            t = times[stats.id] if isinstance(times,dict) else times
            if self.full_dubins:
                mng_msg.append(self.__make_path_mission(stats.id,path,t,insert_mode))
                incr = 1
            else:
                incr = 0
                first = True
                for i,e in enumerate(path.sections):
                    if e.length < 1e-3:
                        continue
                    mng_msg.append(self.__make_dubel_mission(stats.id,i,e,insert_mode if first else MissionInsert.APPEND))
                    incr += 1
                    first = False
            mission_id_incr = max(mission_id_incr,incr)
        self.mission_counter += mission_id_incr
        await self.__send_managers_messages_with_ack(mng_msg,self.msg_retry,self.msg_ack_time)

        
    def __make_dubel_mission(self,ac_id:int, el_id:int, e:BasicPath, insert_mode:MissionInsert) -> tuple[MissionManager,PprzMessage]:
        manager = self.managers[ac_id]
        
        home = manager.get_home()
        assert home is not None
        xhome,yhome = self.transformer.transform(home.lat,home.lon)
        
        start = e.start()
        px = start.x
        py = start.y
        pz = e.end().z
        ptheta = start.theta
        radius = e.radius()
        if e.type == DubinsMove.STRAIGHT:
            radius = 0.
        elif e.type == DubinsMove.LEFT:
            radius = abs(radius)
        elif e.type == DubinsMove.RIGHT:
            radius = -abs(radius)
        else:
            raise ValueError(f"Unknown Dubins move type {e.type}")


        params = [
            px - xhome,
            py - yhome,
            ptheta,
            radius,
            e.length,
            pz,
            ]
        
        return manager,manager.make_mission_custom(
            self.mission_counter+el_id,
            'DUBEL',
            params,
            insert_mode=insert_mode
        )
        
    def __make_path_mission(self,ac_id:int,p:Path,start_time:float,insert_mode:MissionInsert) -> tuple[MissionManager,PprzMessage]:
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
        
        if self.verbosity > 0:
            print(f"{ac_id} : Path type : {p.abbr()}")
            for i,el in enumerate(p.sections):
                print(f"{ac_id} : Element {i} : Length {el.length:.3f} ; Radius {el.radius():.3f} ; Start ({el.start().x-xhome:.3f},{el.start().y-yhome:.3f},{el.start().theta:.3f})")
            print('')
        
        
        if self.extra_straight_length > 0:
            shifted_path = copy.deepcopy(p)
            shifted_path = shifted_path.follow_for(shifted_path.sections[0].duration() + 1e-9)
            assert shifted_path is not None
            
            params = [
                p.start.x-xhome,
                p.start.y-yhome,
                p.start.theta,
                p.end.x-xhome,
                p.end.y-yhome,
                p.end.theta,
                p.end.z,
                p.total_length - 2*self.extra_straight_length,
                start_time+p.total_length/speed if self.speed_ctl else 0,
                float(PPRZ_DUBINS_TYPE_MAP[shifted_path.abbr()[:-1]]),
                shifted_path.max_turn_radius(),
                path_extra_length(p.sections[1:-1])
            ]
        
        else:
            params = [
                p.start.x-xhome,
                p.start.y-yhome,
                p.start.theta,
                p.end.x-xhome,
                p.end.y-yhome,
                p.end.theta,
                p.end.z,
                p.total_length,
                start_time+p.total_length/speed if self.speed_ctl else 0,
                float(PPRZ_DUBINS_TYPE_MAP[p.abbr()]),
                p.max_turn_radius(),
                path_extra_length(p)
            ]
        
        return manager,manager.make_mission_custom(
            self.mission_counter,
            'DUBIN',
            params,
            insert_mode=insert_mode
        )
            

if __name__ == '__main__':
    import argparse
    
    color_dict = {
        17 : 'red',
        18 : 'green',
        20 : 'blue'
    }
    
    stat_list = [ACStats(i,15.1,10/60,60) for i in [17,18,20]]
    separation = 30
    formation = chevron_formation(len(stat_list),separation*1.2)
    
    transformer = pyproj.Transformer.from_crs('WGS84','EPSG:9794') # Default to WGS84 to Lambert93
    home_lat = 43.46223
    home_lon = 1.27289
    home_height = 260-185
    home_alt = 260
    home_x,home_y = transformer.transform(home_lat,home_lon)
    
    start = Pose3D(home_x,home_y,home_alt,0.)
    # fleet_plan = formation_oval(stat_list,start,250,200,formation)
    fleet_plan = formation_rectangle(stat_list,start,250,200,formation)
    
    # fig,ax = plt.subplots()
    # plot_keyframes(fleet_plan,ax,['red','green','blue','yellow','cyan','magenta'])
    # ax.set_aspect('equal')
    # plt.show()
    
    parser = argparse.ArgumentParser()
    parser.add_argument('dubins_solver',help='Path to Dubins Fleet Planner')
    parser.add_argument('--verbose', '-v', action='count', default=0)
    parser.add_argument('-t','--takeoff',action='store_true',help='Send take off mission order first')
    parser.add_argument('--speed-ctl',action='store_true',dest='speed_ctl',help='If set, enable speed control for aircraft')
    parser.add_argument('--dubins-el',action='store_true',dest='dubins_el',help='If set, send Dubins elements (DUBEK mission) instead of full Dubins path (DUBIN mission)')
    parser.add_argument('--autostart',type=float,help='If set, automatically launch the main after the given value (in seconds). Otherwise, wait for user input to start the main loop',
                        default=None)
    parser.add_argument('--carrot',type=float, help='Anticipation time (in seconds) for declaring the current plan end. Default to 5s',
                        default=5)
    
    args = parser.parse_args()
    
    if args.carrot < 0.:
        raise ValueError(f"--carrot argument must be non-negative! Current value is: {args.carrot}")

    
    manager = FleetManager(
        fleet_plan,
        args.dubins_solver,
        separation,
        home_alt,
        transformer=transformer,
        end_strategy=FleetManagerEnd.LOOP,
        speed_ctl=args.speed_ctl,
        verbosity=args.verbose,
        full_dubins=not(args.dubins_el),
        tracking_error=16
    )
    
    manager.end_of_plan_carrot = args.carrot
    
    try:
        manager.wait_ready()
        if args.takeoff:
            print("Take off requested!")
            manager.takeoff(home_height)
        manager.circle_home(insert_mode=MissionInsert.APPEND)
        manager.start_mission()
        if args.autostart is not None:
            print(f"Wait for {args.autostart} seconds before starting main loop...",end=' ',flush=True)
            time.sleep(args.autostart)
            print("Starting now!")
        else:
            input("Press any key to start fleet path planning...")
        asyncio.run(manager.main(),debug=DEBUG)
    except(KeyboardInterrupt,SystemExit):
        print("Intettupted!")
    finally:
        print("Closing")
        manager.closing()
        print("Saving logs")
        
    logs = manager.get_logs()
    
    os.makedirs("logs",exist_ok=True)
    now_str = datetime.datetime.fromtimestamp(time.time()).strftime('%y-%m-%d_%H:%M:%S')
    with open(f"logs/logs_{now_str}.pkl","wb") as f:
        pickle.dump(logs, f)
            
    fig,traj_ax,axes,selectors = plot_trackingdata(logs,color_dict)
    fig.set_size_inches(16,9)
    fig.tight_layout()
    plt.show()
    