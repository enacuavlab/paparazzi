# Copyright (C 2025 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of DubinsFleetPlanner.
# 
# DubinsFleetPlanner is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DubinsFleetPlanner is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DubinsFleetPlanner.  If not, see <https://www.gnu.org/licenses/>.

import json,csv,dataclasses,pathlib,typing,copy
import numpy as np

from Dubins import Pose3D,ACStats,Path,BasicPath,FleetPlan,ListOfTimedPoses,TimedPosesLine,DictOfPoseTrajectories,DubinsMove,mod2pi,central_angle


######################################## Dubins problem writing and parsing ########################################

_CSV_problem_statement_header = "ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt".split(',')

@dataclasses.dataclass
class AC_PP_Problem:
    """
    Compact way to specify an individual problem (ie a line of a CSV specifying a path planning problem)
    """
    stats:ACStats   # Holds ac_id,airspeed,climb and turn_radius.
    start:Pose3D    # Start pose
    end:Pose3D      # End pose
    dt:float=0.     # Time difference with respect to the previous aircraft
    timeslots:list[float] = dataclasses.field(default_factory=list)
    
    def __post_init__(self):
        if isinstance(self.start,np.ndarray):
            self.start = Pose3D.from_array(self.start)
        if isinstance(self.end,np.ndarray):
            self.end = Pose3D.from_array(self.end)
    
    @staticmethod
    def from_nparrays(stats:ACStats,start:np.ndarray,end:np.ndarray,dt:float=0.,timeslots:list[float]=[]):
        return AC_PP_Problem(stats,Pose3D.from_array(start),Pose3D.from_array(end),dt,timeslots)
    
    @staticmethod
    def header(timeslot_num:int=0) -> typing.Sequence[str]:
        output:typing.Sequence[str] = copy.deepcopy(_CSV_problem_statement_header)
        for i in range(timeslot_num):
            output.append(f"Timeslot {i}") # type: ignore
        return output
    
    @staticmethod
    def parse_from_strlist(args:typing.Sequence[str]):
        ac_id       = int(args[0])
        start_x     = float(args[1])
        start_y     = float(args[2])
        start_z     = float(args[3])
        start_theta = float(args[4])
        end_x       = float(args[5])
        end_y       = float(args[6])
        end_z       = float(args[7])
        end_theta   = float(args[8])
        airspeed    = float(args[9])
        climb       = float(args[10])
        turn_radius = float(args[11])
        dt          = float(args[12])
        timeslots   = []
        for e in args[13:]:
            timeslots.append(float(e))
        
        stats   = ACStats(ac_id,airspeed,climb,turn_radius)
        start   = Pose3D(start_x,start_y,start_z,start_theta)
        end     = Pose3D(end_x,end_y,end_z,end_theta)
        
        return AC_PP_Problem(stats,start,end,dt,timeslots)

def straight_pp_problems(l:list[AC_PP_Problem],tol:float=1e-6) -> bool:
    """ Returns True if all individual path planning problems can be solved by the same translation, False otherwise"""
    # Check if a rotation is needed    
    for p in l:
        if abs(central_angle(p.start.theta - p.end.theta)) > tol:
            return False
        
    # No rotation: compute a translation and check if it works for the others
    pb0 = l[0]
    tr_x = pb0.end.x - pb0.start.x
    tr_y = pb0.end.y - pb0.start.y
    
    for p in l[1:]:
        dest_x = p.start.x + tr_x
        dest_y = p.start.y + tr_y
        if (dest_x-p.end.x)**2 + (dest_y-p.end.y)**2 > tol*tol:
            return False
        
    return True

def write_pathplanning_problem_to_CSV(file,data:typing.Sequence[AC_PP_Problem],overwrite:bool=False):
    max_timeslots = max(len(d.timeslots) for d in data)
    
    with open(file, newline='', mode='w' if overwrite else 'x') as outcsv:
        writer = csv.writer(outcsv,delimiter=';')
        
        writer.writerow(AC_PP_Problem.header(max_timeslots))
        
        for p in data:
            
            ac_id       = p.stats.id
            start_x     = p.start.x
            start_y     = p.start.y
            start_z     = p.start.z
            start_theta = p.start.theta
            end_x       = p.end.x
            end_y       = p.end.y
            end_z       = p.end.z
            end_theta   = p.end.theta
            airspeed    = p.stats.airspeed
            climb       = p.stats.climb
            turn_radius = p.stats.turn_radius
            dt          = p.dt
            
            filled_timeslots = [""] * max_timeslots
            for i,v in enumerate(p.timeslots):
                filled_timeslots[i] = str(v)
            
            writer.writerow([ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt]+filled_timeslots)


def parse_pathplanning_problem_from_CSV(file:pathlib.Path) -> list[AC_PP_Problem]:
    output = []
    with open(file) as f:
        reader = csv.reader(f,delimiter=';')
        
        header = next(reader)
        for h,r in zip(header,_CSV_problem_statement_header):
            assert h == r
        
        for l in reader:
            output.append(AC_PP_Problem.parse_from_strlist(l))
            
    return output
        

######################################## Dubins results parsing ########################################

############################## JSON parsing ##############################

def parse_section_from_dict(d:dict) -> BasicPath:
    try:
        del d["m"]
    except KeyError:
        pass
    return BasicPath(**d)

def straight_sections_compression(paths:list[BasicPath]) -> list[BasicPath]:
    """ If two straight basic paths follow one another, merge them into one. Return a (deep)copy of the list after this transformation.
    THIS DOES NOT CHECK IF THE FOLLOWING STRAIGHTS ARE ACTUALLY COLLINEAR.

    Args:
        paths (list[BasicPath])

    Returns:
        list[Path]
    """
    output = []
    i = 0
    n = len(paths)
    while i < n:
        output.append(copy.deepcopy(paths[i]))
        j = i+1
        if paths[i].type == DubinsMove.STRAIGHT:
            while j < n and paths[j].type == DubinsMove.STRAIGHT:
                j += 1
            output[-1].length += sum(p.length for p in paths[i+1:j])
            i = j
        else:
            i += 1
    return output

def parse_trajectory_from_dict(d:dict, straights_compression:bool=False) -> tuple[ACStats,Path]:
    stats_dict  = d["stats"]
    stats = ACStats(**stats_dict)
    
    path_dict   = d["path"]
    sections = [parse_section_from_dict(t) for t in path_dict["sections"]]
    
    if straights_compression:
        sections = straight_sections_compression(sections)

    path = Path(
        path_dict["total_length"],
        Pose3D(**path_dict["start"]),
        Pose3D(**path_dict["end"]),
        sections
    )
    
    return stats,path

def parse_trajectories_from_JSON(file, straights_compression:bool=False) -> FleetPlan:
    with open(file) as jsonfile:
        raw_data = json.load(jsonfile)
        
        paths = [parse_trajectory_from_dict(t, straights_compression) for t in raw_data["trajectories"]]
        output = FleetPlan(
            raw_data["separation"],
            raw_data["z_alpha"],
            raw_data["wind_x"],
            raw_data["wind_y"],
            raw_data["duration"],
            # raw_data["AC_num"],
            paths
        )
        
    return output
    
def print_FleetPlan_to_JSON(file,plan:FleetPlan,subset:typing.Optional[typing.Iterable[int]]=None,overwrite:bool=False):
    with open(file,mode='w' if overwrite else 'x') as jsonfile:
        json.dump(plan.asdict(subset),jsonfile)


def join_trajectories_JSONs(output_file,*input_files):
    base_plan = parse_trajectories_from_JSON(input_files[0])
    
    for f in input_files[1:]:
        other_plan = parse_trajectories_from_JSON(f)
        base_plan.join(other_plan)
        
    print_FleetPlan_to_JSON(output_file,base_plan)

############################## CSV Parsing ##############################

def parse_trajectories_from_CSV(file) -> ListOfTimedPoses:
    with open(file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=';', quotechar='|')
        header = next(reader)
        ids = []
        
        # Check header
        assert(header[0].lower() == "time")
        
        for i in range(1,len(header),4):
            assert(header[i].lower()[0] == "x" and\
                header[i+1].lower()[0] == "y" and\
                header[i+2].lower()[0] == "z" and\
                header[i+3].lower()[:5] == "theta")
            ids.append(int(header[i][2:]))
        
        output:ListOfTimedPoses = list()
        for row in reader:
            ts = float(row[0])
            l = dict()
            for i in range(1,len(row),4):
                id = ids[(i-1)//4]
                x = float(row[i])
                y = float(row[i+1])
                z = float(row[i+2])
                theta = float(row[i+3])
                l[id] = Pose3D(x,y,z,theta)
                
            output.append((ts,l))
            
            
        return output
    
def transpose_list_of_trajectories(l:ListOfTimedPoses) -> DictOfPoseTrajectories:
    output:DictOfPoseTrajectories = dict()
    for r in l:
        ts = r[0]
        for ac_id,pose in r[1].items():
            try:
                output[ac_id].append((ts,pose))
            except KeyError:
                output[ac_id] = [(ts,pose)]
    
    return output

def make_CSV_trajectories_header(dataline:TimedPosesLine) -> list[str]:
    output = ["time"]
    key_list = list(dataline[1].keys())
    key_list.sort()
    for id in key_list:
        output.extend([f"X_{id}",f"Y_{id}",f"Z_{id}",f"theta_{id}"])
    return output

def print_trajectories_to_CSV(output_file,data:ListOfTimedPoses,time_offset:float=0.):
    with open(output_file, newline='', mode='x') as outcsv:
        writer = csv.writer(outcsv,delimiter=';')
        
        # Write header
        header = make_CSV_trajectories_header(data[0])
        writer.writerow(header)
        
        key_list = list(data[0][1].keys())
        key_list.sort()
        
        for line in data:
            row = [line[0]+time_offset]
            for id in key_list:
                p = line[1][id]
                row.extend([p.x,p.y,p.z,p.theta])
            writer.writerow(row)


def join_trajectories_CSVs(output_file,*input_files):
    
    last_time = 0.
    previous_header = None
    with open(output_file, newline='', mode='x') as outcsv:
        writer = csv.writer(outcsv,delimiter=';')
        
        key_list = list()
        
        for file in input_files:
            data = parse_trajectories_from_CSV(file)
            if previous_header is None:
                previous_header = make_CSV_trajectories_header(data[0])
                writer.writerow(previous_header)
                key_list = list(data[0][1].keys())
                key_list.sort()
            else:
                assert previous_header == make_CSV_trajectories_header(data[0])
                assert (data[0][1].keys() <= set(key_list)) and (data[0][1].keys() >= set(key_list))
                
            for line in data:
                row = [line[0]+last_time]
                for id in key_list:
                    p = line[1][id]
                    row.extend([p.x,p.y,p.z,p.theta])
                writer.writerow(row)
                
            last_time += data[-1][0]
          
######################################## Path planning summary parsing ########################################

@dataclasses.dataclass
class CaseSummary:
    srcfile:pathlib.Path
    success:bool
    false_positive:bool
    iterations:int
    duration_ns:int
    threads:int
    possibls_paths_per_ac:int
    initial_guess_time:float
    travel_time:float
    worst_rate:float
    
    @staticmethod
    def from_strlist(args:list[str]):
        return CaseSummary(
            pathlib.Path(args[0]),
            True if args[1].lower() == 'true' else False,
            True if args[2].lower() == 'true' else False,
            int(args[3]),
            int(args[4]),
            int(args[5]),
            int(args[6]),
            float(args[7]),
            float(args[8]),
            float(args[9])
        )
        
    def as_strlist(self) -> list[str]:
        return [
            str(self.srcfile),
            'true' if self.success else 'false',
            'true' if self.false_positive else 'false',
            str(self.iterations),
            str(self.duration_ns),
            str(self.threads),
            str(self.possibls_paths_per_ac),
            str(self.initial_guess_time),
            str(self.travel_time),
            str(self.worst_rate)
        ]
            
    @staticmethod
    def header():
        #TODO : Header modified in C++, need to correct it here and in the matching dataclass
        return "Test input;Success;False positive;Iterations;Duration(ns);Threads;Possible paths;Initial guessed time;Final obtained time;Worst rate(u/s)".split(';')
            

def parse_result_summary(summary_loc:pathlib.Path) -> dict[str,CaseSummary]:
    output = dict()
    
    with open(summary_loc) as f:
        reader = csv.reader(f,delimiter=';')
        
        header = next(reader)
        
        for s,h in zip(header,CaseSummary.header()):
            assert s.lower() == h.lower()
        
        for l in reader:
            line = CaseSummary.from_strlist(l)
            key = str(line.srcfile.resolve())
            
            if key in output:
                print(f"WARNING: In file {summary_loc}, the following case is duplicated:\n\t{l[0]}")
                
            output[key] = line

    return output


def write_summary(summary_loc:pathlib.Path,data:dict[str,CaseSummary]):
    with open(summary_loc,mode='w') as f:
        writer = csv.writer(f,delimiter=';')
        
        writer.writerow(CaseSummary.header())
        
        for d in data.values():
            writer.writerow(d.as_strlist())


######################################## Program entry point ########################################       
            
if __name__ == '__main__':
    import argparse
    import pathlib
    
    parser = argparse.ArgumentParser('Data manip','Combine and modify Dubins Fleet Path Plan files.')
    parser.add_argument('files',type=str,nargs='+',help='File(s) from which to parse data')
    parser.add_argument('-j','--join',type=str,help='Join several files into one by putting trajectories end-to-end',default=None)
    
    
    args = parser.parse_args()
    
    paths = [pathlib.Path(f) for f in args.files]
    ext = paths[0].suffix.lower()

    for p in paths:
        try:
            assert(p.is_file())
        except AssertionError:
            raise ValueError(f"The following path does not indicate a file:\n{p}")
        
        try:
            assert(p.suffix.lower() == ext)
        except AssertionError:
            raise ValueError(f"The following file does not have the same extension as the others (got: '{p.suffix.lower()}', expected: '{ext}')\n{p}")
        
    
    if ext == ".csv" and args.join is not None:
        join_trajectories_CSVs(args.join,*paths)
        exit(0)
    elif ext == ".json" and args.join is not None:
        join_trajectories_JSONs(args.join,*paths)
        exit(0)
    
        
        
        
    print("No action selected... Do nothing and exit...")
        
                
            
            
            
        