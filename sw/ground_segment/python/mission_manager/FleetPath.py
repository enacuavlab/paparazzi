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
    
def formation_oval(ac_stats:list[ACStats],start:Pose3D,length:float,width:float,formation:Formation, half_points:bool=False) -> FleetKeyframes:
    """Given a formation, make a flat oval sequence from it
    
    |--- [1] <--- length --- start <-|
    |                                |
    |                                |
   [2]                              [5]  width
    |                                |
    |                                |
    |--> [3] ---------------> [4] ---|

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
    
    # Start
    keyposes = [formation.get_abs_positions().copy()]
    
    # First long
    formation.center[0] += np.cos(start.theta)*length
    formation.center[1] += np.sin(start.theta)*length
    
    keyposes.append(formation.get_abs_positions().copy())
    
    if half_points:
        # First half turn
        theta = start.theta + np.pi/4
        formation.center[0] += np.cos(theta)*width/np.sqrt(2)
        formation.center[1] += np.sin(theta)*width/np.sqrt(2)
        formation.orientation += np.pi/2
        
        keyposes.append(formation.get_abs_positions().copy())
        
        # End of of first turn
        theta += np.pi/2
        formation.center[0] += np.cos(theta)*width/np.sqrt(2)
        formation.center[1] += np.sin(theta)*width/np.sqrt(2)
        formation.orientation += np.pi/2
        
        keyposes.append(formation.get_abs_positions().copy())
    else:
        # End of of first turn
        theta = start.theta + np.pi/2
        formation.center[0] += np.cos(theta)*width
        formation.center[1] += np.sin(theta)*width
        formation.orientation += np.pi
        
        keyposes.append(formation.get_abs_positions().copy())
    
    # End of second long
    if half_points:
        theta += np.pi/4
    else:
        theta += np.pi/2
    formation.center[0] += np.cos(theta)*length
    formation.center[1] += np.sin(theta)*length
    
    keyposes.append(formation.get_abs_positions().copy())
    
    if half_points:
        # Second halg turn
        theta += np.pi/4
        formation.center[0] += np.cos(theta)*width/np.sqrt(2)
        formation.center[1] += np.sin(theta)*width/np.sqrt(2)
        formation.orientation += np.pi/2
        
        keyposes.append(formation.get_abs_positions().copy())
    
    # import matplotlib.pyplot as plt
    
    # colormap = ['r','g','b','y','k','purple']
    
    # for i,el in enumerate(keyposes):
    #     plt.scatter(el[:,0],el[:,1],color=colormap[i])
        
    # plt.gca().set_aspect('equal')
    # plt.show()
    
    return FleetKeyframes(ac_stats,keyposes)
