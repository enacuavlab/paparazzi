import dataclasses,typing
import numpy as np

from Dubins import ACStats,Pose3D
from Formation import Formation
from ioUtils import AC_PP_Problem

@dataclasses.dataclass
class FleetKeyframes:
    ac_stats:list[ACStats]
    keyposes:list[np.ndarray]
    
    @property
    def ac_num(self) -> int:
        return len(self.ac_stats)
    
    @property
    def keyposes_num(self) -> int:
        return len(self.keyposes)
    
    def generate_pb_to(self,key_index:int,starts:list[Pose3D]) -> list[AC_PP_Problem]:
        return [AC_PP_Problem(self.ac_stats[i],starts[i],self.keyposes[key_index][i]) for i in range(self.ac_num)]
    
    def generate_pb_between(self,start_key:int,end_key:typing.Optional[int]=None) -> list[AC_PP_Problem]:
        if end_key is None:
            end_key = start_key+1
        return [AC_PP_Problem(self.ac_stats[i],self.keyposes[start_key][i],self.keyposes[end_key][i]) for i in range(self.ac_num)]
    
def formation_oval(ac_stats:list[ACStats],start:Pose3D,length:float,width:float,formation:Formation) -> FleetKeyframes:
    """Given a formation, make a flat oval sequence from it
    
    |--- [1] <--- length --- start <-|
    |                                |
    |                                |
    |                              width
    |                                |
    |                                |
    |--> [2] ---------------> [3] ---|

    Args:
        ac_stats (list[ACStats]): Aircraft statistics
        start (Pose3D): Reference pose for the initial formation location and orientation
        length (float): Long arm length of the oval
        width (float): Distance between the oval sides
        formation (Formation): Reference formation to use

    Returns:
        FleetKeyframes
    """
    assert len(ac_stats) == formation.agent_num
    
    formation.center[0] = start.x
    formation.center[1] = start.y
    formation.center[2] = start.z
    formation.orientation = start.theta
    formation.to_barycentric_coords()
    
    keyposes = [formation.get_abs_positions()]
    
    formation.center[0] += np.cos(start.theta)*length
    formation.center[1] += np.sin(start.theta)*length
    
    keyposes.append(formation.get_abs_positions())
    
    theta = start.theta + np.pi/2
    formation.center[0] += np.cos(start.theta)*width
    formation.center[1] += np.sin(start.theta)*width
    theta += np.pi/2
    
    keyposes.append(formation.get_abs_positions())
    
    formation.center[0] += np.cos(start.theta)*length
    formation.center[1] += np.sin(start.theta)*length
    
    keyposes.append(formation.get_abs_positions())
    
    return FleetKeyframes(ac_stats,keyposes)
