# Copyright (C) 2025 Mael FEURGARD <mael.feurgard@enac.fr>
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

from __future__ import annotations

import typing
import copy

from dataclasses import dataclass,field
import enum

import numpy as np

from Dubins import Pose3D

from scipy.spatial import distance_matrix


#################### Util ####################

def quasi_square_upper(n:int) -> tuple[int,int]:
    lower_sqrt = int(np.sqrt(n))
    upper_sqrt = lower_sqrt+1
    
    lower_square = lower_sqrt*lower_sqrt
    middle_quasi_square = lower_sqrt*upper_sqrt
    
    if lower_square == n:
        return lower_sqrt,lower_sqrt
    elif middle_quasi_square >= n:
        return upper_sqrt,lower_sqrt
    else:
        return upper_sqrt,upper_sqrt

#################### Formation objects ####################

class SidePoint(enum.Enum):
    """
    Enum representing three locations on a serie of values:
    minimum, middle and maximum.
    """
    MIN = -1
    MID = 0
    MAX = 1
    
    def get_val(self,vals:np.ndarray):
        if self == SidePoint.MIN:
            return np.min(vals)
        elif self == SidePoint.MAX:
            return np.max(vals)
        elif self == SidePoint.MID:
            return (np.min(vals)+np.max(vals))/2
        else:
            raise ValueError(f"Unknown value for SidePoint: {self}")
    
@dataclass
class Corner:
    """
    Specify a junction point for a 3D point cloud by specifying a location with respect to
    the X,Y and Z values.
    """
    x_side:SidePoint
    y_side:SidePoint
    z_side:SidePoint
    
    def get_corner(self,pts:np.ndarray) -> np.ndarray:
        x_r = self.x_side.get_val(pts[:,0])
        y_r = self.y_side.get_val(pts[:,1])
        z_r = self.z_side.get_val(pts[:,2])
        
        return np.array([x_r,y_r,z_r])
        

@dataclass
class Formation:
    """ Represent a formation as a set of 3D poses (stored in a numpy array) in a local referential
    """
    name:str                # Formation name, for labelling
    positions:np.ndarray    # Formations positions with respect to the centerpoint. List of points, 4D (x,y,z,theta), theta being the XY orientation
    symmetry_lr:bool    = False     # Present a Left-Right symmetry, meaning that a right turn can be used to solve a left turn by mirroring. Default to False
    center:np.ndarray   = field(default_factory=lambda:np.zeros(3))     # Center point location in the global referential
    orientation:float   = 0.                                            # Formation XY orientation with respect to the global referential
    
    
    @property
    def agent_num(self) -> int:
        """ Number of agents in the formation """
        return self.positions.shape[0]
    
    def reorder(self,indices:np.ndarray|list) -> Self:
        """ Reorder the positions in the formation given a permutation

        Args:
            indices (np.ndarray): Array of different ints (a permutation) of length equal to `len(self.positions)`

        Returns:
            typing.Self: This formation, permuted on place
        """
        self.positions = self.positions[indices]
        return self
    
    def get_abs_positions(self) -> np.ndarray:
        """ Compute the formation in the global referential, 
        by applying the given rotation (set by `orientation`) and translation (set by `center`)

        Returns:
            np.ndarray: Formation poses in the global referential
        """
        rotation_matrix = np.array([
            [ np.cos(self.orientation), np.sin(self.orientation)],
            [-np.sin(self.orientation), np.cos(self.orientation)]
        ])
        
        global_positions = self.positions.copy()
        global_positions[:,-1] += self.orientation                          # Shift angle
        global_positions[:,:2] = global_positions[:,:2] @ rotation_matrix   # Apply rotation matrix
        global_positions[:,:-1] += self.center                              # Translate
        
        return global_positions
    
    
    def barycenter(self) -> np.ndarray:
        """ Compute the formation XYZ barycenter (no angles)

        Returns:
            np.ndarray: A 3D point, the barycenter
        """
        return np.average(self.positions[:,:-1],axis=0)
    
    def to_barycentric_coords(self,shift_center:bool=False) -> typing.Self:
        """ Modifies the relative positions (and possibly the center) such that the formation is written in barycentric coordinates
        

        Args:
            shift_center (bool, optional): If set, also shift the `center` by the barycenter, such that the poses are not moved
                with respect to the global referential. Defaults to False.
        """
        bary = self.barycenter()
        
        if shift_center:
            self.center += bary
            
        self.positions[:,:-1] -= bary
        
        return self
        
    def apply_rotation(self) -> typing.Self:
        """ Apply the rotation on the poses, thus moving them and reseting `orientation`
        """
        rotation_matrix = np.array([
            [ np.cos(self.orientation), np.sin(self.orientation)],
            [-np.sin(self.orientation), np.cos(self.orientation)]
        ])
        
        self.positions[:,-1] += self.orientation                        # Shift angle
        self.positions[:,:2] = self.positions[:,:2] @ rotation_matrix   # Apply rotation matrix
        
        self.orientation = 0. # Reset orientation
        
        return self
        
        
    def add_poses(self,new_poses:np.ndarray) -> typing.Self:
        """ Add new poses to existing

        Args:
            new_poses (np.ndarray): _description_
        """
        self.positions = np.concatenate((self.positions,new_poses),axis=0)
        
        return self
        
    def join_poses(self,new_poses:np.ndarray,this_corner:Corner,other_corner:Corner,delta:np.ndarray) -> typing.Self:
        """ Add new poses to current formation by joining the current corner with the new one, then apply the delta
        vector for separation

        Args:
            new_poses (np.ndarray): List of poses (4D, XYZ and XY orientation) to be added to the current formation 
            this_corner (Corner): Junction point for the current formation (based on the encompassing cuboid)
            other_corner (Corner): Junction point for the points to be added (based on the encompassing cuboid)
            delta (np.ndarray): Shift to apply one the junctions are made in order to have some relative fine control
        """
        
        # Get corner points
        current_cornerpoint = this_corner.get_corner(self.positions)
        new_cornerpoint = other_corner.get_corner(new_poses)
        
        # Align the new points of the current based on corners and shift by delta
        new_poses[:,:-1] += (current_cornerpoint-new_cornerpoint+delta)
        
        # Add them to formation
        return self.add_poses(new_poses)
    
    def move(self,p:Pose3D) -> typing.Self:
        self.positions[:,0] += p.x
        self.positions[:,1] += p.y
        self.positions[:,2] += p.z
        self.orientation += p.theta
        
        return self
    
    def sort_x(self) -> typing.Self:
        self.positions = self.positions[self.positions[:,0].argsort()]
        
        return self
    
    def sort_y(self) -> typing.Self:
        self.positions = self.positions[self.positions[:,1].argsort()]
        
        return self
        
        

#################### Define some special formations ####################

##### Circle #####

def circle_formation_from_radius(N:int,radius:float,clockwise:bool=False) -> Formation:
    angular_pos = np.linspace(0,2*np.pi,N,endpoint=False)
    orientations = angular_pos + (-np.pi/2 if clockwise else np.pi/2)
    xs  = radius*np.cos(angular_pos)
    ys  = radius*np.sin(angular_pos)
    zs  = np.zeros(N)
    
    return Formation("Circle_"+("Clockwise" if clockwise else "Anticlockwise"),
                     np.vstack([xs,ys,zs,orientations]).T,
                     False)


def circle_formation_from_sep(N:int,sep:float,clockwise:bool=False) -> Formation:
    radius = sep/np.sqrt(2*(1-np.cos(2*np.pi/N)))
    return circle_formation_from_radius(N,radius,clockwise)

##### Line #####

def general_line_formation(N:int,sep:float,orientation:float) -> Formation:
    """ Generate a formation along the X axis with the given orientation and separation.
    This final formation is rotated such that every aircraft heads right (final orientation is 0°, headed toward X+)

    Args:
        N (int): Number of aircraft
        sep (float): Separation
        orientation (float): XY Orientation (same for every aircraft )

    Returns:
        Formation: A formation centered on 0 with extra no rotation.
    """
    xs = (np.arange(N)-(N-1)/2)*sep
    ys = np.zeros(N)
    zs = np.zeros(N)
    orientations = np.repeat(orientation,N)
    
    return Formation("GenericLine", np.vstack([xs,ys,zs,orientations]).T,False,orientation=-orientation,)

def line_formation(N:int,sep:float) -> Formation:
    """ Putting aircraft in line formation (flying parallel, side by side)
    
    A A A A A A 
    | | | | | |

    Args:
        N (int): Number of aircraft
        sep (float): Separation between aircraft

    Returns:
        Formation
    """
    output = general_line_formation(N,sep,np.pi/2)
    output.name = "Line"
    output.symmetry_lr = True
    return output

def column_formation(N:int,sep:float) -> Formation:
    """ Putting aircraft in column (or trail), following one another

    -> -> -> -> -> ->

    Args:
        N (int): Number of aircraft
        sep (float): Separation between aircraft

    Returns:
        Formation
    """
    output = general_line_formation(N,sep,0.)
    output.name = "Column"
    output.symmetry_lr = True
    return output

def echelon_left_formation(N:int,sep:float,angle:float=np.pi/4) -> Formation:
    """Putting aircraft in left echelon, that is on diagonal with the 'leader' being rightmost
    
    / / / / / /
    
    (Or, if rotated by 45°)
    
              A
            A |
          A |
        A |
      A |
    A |
    |

    Args:
        N (int): Number of aircraft
        sep (float): Separation between aircraft
        angle (float): In radian, magnitude of the angle from aircraft to aircraft. Default to Pi/4 (45°)

    Returns:
        Formation
    """
    
    output = general_line_formation(N,sep,np.pi-angle)
    output.name = "LeftEchelon"
    return output

def echelon_right_formation(N:int,sep:float,angle:float=np.pi/4) -> Formation:
    """ Putting aircraft in right echelon, that is on diagonal with the 'leader being leftmost

    \\ \\ \\ \\ \\ \\ 

    (Or, rotated by 45°)
    
    A
    | A
      | A
        | A
          | A
            | A
              |

    Args:
        N (int): Number of aircraft
        sep (float): Separation between aircraft
        angle (float): In radian, magnitude of the angle from aircraft to aircraft. Default to Pi/4 (45°)

    Returns:
        Formation
    """
    
    output = general_line_formation(N,sep,angle)
    output.name = "RightEchelon"
    return output


def v_formation(N:int,sep:float,angle:float=np.pi/4,N_right:int|None=None) -> Formation:
    """ Generate a V formation, left major (the 'lead' aircraft is considered belonging to the left wing)
    By default (`N_right` set to None), split the formation in two equal wings (the formation is 'balanced' when N is odd).
    If `N_right` is specified, then `N  is the number of aircraft on the left wing (including the leading one).
    
    Formation drawing with Left wing and Righ wing (example for N=7 or N=8):
     
          (R)
    L     R
     L   R
      L R
       L
          
    Args:
        N (int): Number of aircraft in the whole formation is `N_right` is None. Otherwise, number of aircraft on the left part (+ leader)
        sep (float): Separation between aircraft
        angle (float): In radian, magnitude of the angle from aircraft to aircraft. Default to Pi/4 (45°)
        N_right (int | None, optional): If specified, number of aircraft on the right part. Defaults to None.

    Returns:
        Formation
    """   
    
    if N_right is None:
        right_N = N // 2
        left_N = N - right_N
    else:
        right_N = N_right
        left_N = N
    
    ech_left = echelon_left_formation(left_N,sep,angle)
    ech_right = echelon_right_formation(right_N,sep,angle)
    
    if ech_left.agent_num == ech_right.agent_num+1:
        symmetric = True
    else:
        symmetric = False
    
    ech_left.apply_rotation()
    ech_right.apply_rotation()
    
    left_junc = Corner(SidePoint.MIN,SidePoint.MIN,SidePoint.MIN)
    right_junc = Corner(SidePoint.MIN,SidePoint.MAX,SidePoint.MIN)
    
    shift_vec = np.array([sep,-sep,0.])/np.sqrt(2)
    
    ech_left.join_poses(ech_right.positions,left_junc,right_junc,shift_vec)
    ech_left.to_barycentric_coords(False)
    
    ech_left.name = "Vee"
    ech_left.symmetry_lr = symmetric
    
    return ech_left

def chevron_formation(N:int,sep:float,angle:float=np.pi/4,N_right:int|None=None) -> Formation:
    """ Generate a chevron formation, left major (the 'lead' aircraft is considered belonging to the left wing)
    By default (`N_right` set to None), split the formation in two equal wings (the formation is 'balanced' when N is odd).
    If `N_right` is specified, then `N  is the number of aircraft on the left wing (including the leading one).
    
    Formation drawing with Left wing and Righ wing (example for N=7 or N=8):
    
       L
      L R
     L   R
    L     R
          (R)
          
    Args:
        N (int): Number of aircraft in the whole formation is `N_right` is None. Otherwise, number of aircraft on the left part (+ leader)
        sep (float): Separation between aircraft
        angle (float): In radian, magnitude of the angle from aircraft to aircraft. Default to Pi/4 (45°)
        N_right (int | None, optional): If specified, number of aircraft on the right part. Defaults to None.

    Returns:
        Formation
    """    
    
    if N_right is None:
        right_N = N // 2
        left_N = N - right_N
    else:
        right_N = N_right
        left_N = N
    
    # Note: for a chevron, the LEFT echelon goes on the RIGHT side and vice versa !
    
    ech_left = echelon_left_formation(right_N,sep,angle)
    ech_right = echelon_right_formation(left_N,sep,angle)
    
    if ech_left.agent_num == ech_right.agent_num+1:
        symmetric = True
    else:
        symmetric = False
    
    ech_left.apply_rotation()
    ech_right.apply_rotation()
    
    left_junc = Corner(SidePoint.MAX,SidePoint.MAX,SidePoint.MIN)
    right_junc = Corner(SidePoint.MAX,SidePoint.MIN,SidePoint.MIN)
    
    shift_vec = np.array([sep,sep,0.])/np.sqrt(2)
    
    ech_left.join_poses(ech_right.positions,left_junc,right_junc,shift_vec)
    ech_left.to_barycentric_coords(False)
    
    ech_left.name = "Chevron"
    ech_left.symmetry_lr = symmetric
    
    return ech_left



##### Square and the like #####

def generic_rectangle_formation(lines:int,columns:int,Xsep:float,Ysep:float,orientation:float) -> Formation:
    N = lines * columns
    
    line_xs = np.arange(lines) * Xsep
    # xs = np.tile(line_xs,columns)

    col_ys = np.arange(columns) * Ysep
    # ys = np.tile(col_ys,lines)
    
    xs_m,ys_m = np.meshgrid(line_xs,col_ys)
    
    xs = xs_m.flatten()
    ys = ys_m.flatten()
    
    zs = np.zeros(N)
    
    orientations = np.repeat(orientation,N)
    
    return Formation("GenericBattle",np.vstack([xs,ys,zs,orientations]).T,orientation=-orientation)

def rectangle_formation(lines:int,columns:int,sep:float,orientation:float) -> Formation:
    
    output = generic_rectangle_formation(lines,columns,sep,sep,orientation)
    output.name = "Rectangle"
    output.symmetry_lr = True
    return output


def diamond_formation(N_side:int,sep:float) -> Formation:
    output = rectangle_formation(N_side,N_side,sep,np.pi/4)
    output.name = "Diamond"
    output.symmetry_lr = True
    return output


def staggered_formation(lines:int,columns:int,sep:float,orientation:float) -> Formation:    
    stagg_lines = lines // 2
    fair_lines = lines - stagg_lines
    
    fair_rect = generic_rectangle_formation(fair_lines,columns,np.sqrt(3)*sep,sep,orientation)
    stag_rect = generic_rectangle_formation(stagg_lines,columns,np.sqrt(3)*sep,sep,orientation)
    
    topleft = Corner(SidePoint.MAX,SidePoint.MAX,SidePoint.MID)
    
    shift = np.array([-np.sqrt(3)*sep/2,-sep/2,0.])
    # shift= np.zeros(3)
    
    output = fair_rect.join_poses(stag_rect.positions,topleft,topleft,shift)
    output.name = "Staggered"
    output.symmetry_lr = True
    return output

def stacked_chevrons_formation(lines:int,columns:int,chev_sep:float,stack_sep:float,angle:float=np.pi/4) -> Formation:
    
    std_formation = chevron_formation(columns,chev_sep,angle)
    
    symmetry = std_formation.symmetry_lr
    
    output_formation = copy.deepcopy(std_formation)
    
    lines -= 1
    
    for _ in range(lines):
        rightmid = Corner(SidePoint.MAX,SidePoint.MID,SidePoint.MID)
        
        shift = np.array([stack_sep,0.,0.])
        
        output_formation.join_poses(std_formation.positions,rightmid,rightmid,shift)
    
    output_formation.name = "StackedChevrons"
    output_formation.symmetry_lr = symmetry
    return output_formation

def stacked_echelon_formation(lines:int,columns:int,chev_sep:float,stack_sep:float,angle:float=np.pi/4,right:bool=True) -> Formation:
    
    std_formation = echelon_right_formation(columns,chev_sep,angle) if right else echelon_left_formation(columns,chev_sep,angle)

    symmetry = std_formation.symmetry_lr
    
    
    output_formation = copy.deepcopy(std_formation)
    
    lines -= 1
    
    corner = Corner(SidePoint.MAX,SidePoint.MAX,SidePoint.MID) if right else Corner(SidePoint.MIN,SidePoint.MAX,SidePoint.MID)
    shift_dir = np.array([np.cos(angle),np.sin(angle),0.]) if right else np.array([np.cos(np.pi-angle),np.sin(np.pi-angle),0.])
    shift = stack_sep*shift_dir
    
    for _ in range(lines):        
        
        output_formation.join_poses(std_formation.positions,corner,corner,shift)
    
    output_formation.name = "StackedChevrons"
    output_formation.symmetry_lr = symmetry
    return output_formation


def list_all_formations(ncols:int,nlines:int,n:int,sep:float) -> list[Formation]:
    output:list[Formation] = []
    
    
    ratio = min(nlines,ncols)/max(nlines,ncols)
    
    if ratio >= 1:
        ratio = 0.4
    ratio = ratio if ratio < 0.5 else 1-ratio
    
    short = int(n*ratio)
    long = n - short
    
    if short == 0:
        short = 1
        long -= 1
    
    circ_clockwise = circle_formation_from_sep(n,sep,True)
    output.append(circ_clockwise)
    circ_anticlock = circle_formation_from_sep(n,sep,False)
    output.append(circ_anticlock)
    
    
    hline       = line_formation(n,sep)
    output.append(hline)
    column      = column_formation(n,sep)
    output.append(column)
    right_ech   = echelon_right_formation(n,sep)
    output.append(right_ech)
    left_ech    = echelon_left_formation(n,sep)
    output.append(left_ech)
    
    
    right_ech_trd   = echelon_right_formation(n,sep,np.pi/3)
    right_ech_trd.name += "Trd"
    output.append(right_ech_trd)
    left_ech_trd    = echelon_left_formation(n,sep,np.pi/3)
    left_ech_trd.name += "Trd"
    output.append(left_ech_trd)


    vee     = v_formation(n,sep)
    output.append(vee)
    # vee_p1  = v_formation(N+1,sep)
    # vee_p1.name += "PlusOne"
    # output.append(vee_p1)
    vee_leftmost    = v_formation(long,sep,N_right=short)
    vee_leftmost.name += "LeanLeft"
    output.append(vee_leftmost)
    vee_rightmost   = v_formation(short,sep,N_right=long)
    vee_rightmost.name += "LeanRight"
    output.append(vee_rightmost)


    chevron     = chevron_formation(n,sep)
    output.append(chevron)
    # chevron_p1  = chevron_formation(N+1,sep)
    # chevron_p1.name += "PlusOne"
    # output.append(chevron_p1)
    chevron_leftmost    = chevron_formation(long,sep,N_right=short)
    chevron_leftmost.name += "LeanLeft"
    output.append(chevron_leftmost)
    chevron_rightmost   = chevron_formation(short,sep,N_right=long)
    chevron_rightmost.name += "LeanRight"
    output.append(chevron_rightmost)


    # Get rid of square-like formations when line or column is 1 (devolve into a line formation as seen before)
    if ncols > 1 and nlines > 1:
        rectangle   = rectangle_formation(nlines,ncols,sep,0.).to_barycentric_coords()
        output.append(rectangle)
        co_rect     = rectangle_formation(nlines,ncols,sep,np.pi/2).to_barycentric_coords()
        co_rect.name += "Bis"
        output.append(co_rect)
        diamond     = rectangle_formation(nlines,ncols,sep,np.pi/4).to_barycentric_coords()
        diamond.name = "Diamond"
        output.append(diamond)
        diamond_pi3rd = rectangle_formation(nlines,ncols,sep,np.pi/3).to_barycentric_coords()
        diamond_pi3rd.name += "Trd"
        output.append(diamond_pi3rd)


        staggered   = staggered_formation(nlines,ncols,sep,0.).to_barycentric_coords()
        output.append(staggered)
        stacked_chevrons = stacked_chevrons_formation(nlines,ncols,sep,2*sep).to_barycentric_coords()
        output.append(stacked_chevrons)
        stacked_right_echelon = stacked_echelon_formation(nlines,ncols,sep,2*sep).to_barycentric_coords()
        stacked_right_echelon.name += "Right"
        output.append(stacked_right_echelon)
        stacked_left_echelon = stacked_echelon_formation(nlines,ncols,sep,2*sep,right=False).to_barycentric_coords()
        stacked_left_echelon.name += "Left"
        output.append(stacked_left_echelon)

    return output

if __name__ == '__main__':
    import argparse
    import matplotlib.pyplot as plt
    from matplotlib.axes import Axes
    from matplotlib import colormaps
    
    def plot_formation(ax:Axes, form:Formation, title:str, label:str='', shortest:bool=True):
    
        cmap = colormaps['cool']
    
        positions = form.get_abs_positions()
        xs = positions[:,0]
        ys = positions[:,1]
        
        dxs = np.cos(positions[:,-1])
        dys = np.sin(positions[:,-1])
        
        ax.set_title(title)
        ax.set_aspect('equal')
        
        colors = cmap(np.linspace(0,1,xs.shape[0],endpoint=True))
        
        if shortest:
            distances = distance_matrix(positions[:,:2],positions[:,:2])
            min_dist = np.inf
            min_pair = (None,None)
            for i in range(len(positions)):
                for j in range(i+1,len(positions)):
                    if distances[i,j] < min_dist:
                        min_dist = distances[i,j]
                        min_pair = (i,j)
            
            ax.plot([xs[min_pair[0]],xs[min_pair[1]]],
                    [ys[min_pair[0]],ys[min_pair[1]]],
                    linestyle='--',color='k',label=f"Minimal distance: {min_dist:.3f}")
        
        return ax.quiver(xs,ys,dxs,dys,color=colors,
                angles='xy',
                scale=5.,
                scale_units='inches',
                units='inches',
                label=label)
    
    parser = argparse.ArgumentParser("Formations generator tester")
    parser.add_argument('cols'  ,type=int,  
        help='Number of columns in the formation (if relevant, otherwise the number of aircraft is `cols x lines`)')
    
    parser.add_argument('lines' ,type=int,  
        help='Number of lines in the formation (if relevant, otherwise the number of aircraft is `cols x lines`)')
    
    parser.add_argument('-s','--sep', dest='sep', type=float, default=0.5,
        help='Positive. Minimal separation required between aircrafts')
    
    parser.add_argument('--ysep',type=float,default=-1.,
        help='Positive. Y separation, if it makes sense for the formation. Default to `2*sep`')
    
    parser.add_argument('--fcircles',action='store_true',help='Plot circle-like formations')
    parser.add_argument('--flines',action='store_true',help='Plot line-like formations')
    parser.add_argument('--fchev',action='store_true',help='Plot chevron-like formations')
    parser.add_argument('--fsquare',action='store_true',help='Plot square-like formations')
    
    parser.add_argument('--fall',action='store_true',help='Plot all formations (override other flags)')
    parser.add_argument('--ftest',action='store_true',help='Plot formations used for testing')
    
    args = parser.parse_args()
    
    ## Parameters
    
    ncols    = args.cols
    nlines   = args.lines
    
    N = ncols * nlines
    
    sep    = args.sep
    ysep   = args.ysep if args.ysep > 0 else 2*sep
    
    ## Formation generation
    if args.ftest:
        formations = list_all_formations(ncols,nlines,N,sep)
        
        n_formations = len(formations)
        
        row_num = n_formations//4 if (n_formations//4)*4 == n_formations else (1+n_formations//4)
        
        fig,ax = plt.subplots(4,row_num,sharex='all',sharey='all',figsize=(16,9))
        ax:list[list[Axes]]
        
        for i in range(n_formations):
            j = i // row_num
            ibis = i % row_num
            
            a = ax[j][ibis]
            
            fname = formations[i].name.lower()
            if 'circle' in fname:
                f = formations[i]
            else:
                if 'rectangle' in fname or 'diamond' in fname or 'stacked' in fname:
                    f = formations[i]
                else:
                    f = formations[i].sort_y()
            
            plot_formation(a,f,f.name,shortest=False)
            
        fig.suptitle(f"Formations {ncols}x{nlines} used in testing")
        fig.tight_layout()
        plt.show()
        exit(0)
    
    
    # Circles
    plotCircles = args.fall if args.fall else args.fcircles
    
    circ_clockwise = circle_formation_from_sep(N,sep,True)
    circ_anticlock = circle_formation_from_sep(N,sep,False)
    
    if plotCircles:
        fig,ax = plt.subplots(1,2,sharex='all',sharey='all')
        ax:list[Axes]
        
        plot_formation(ax[0],circ_clockwise,"Clockwise circle")
        plot_formation(ax[1],circ_anticlock,"Anti-clockwise circle")
        
        ax[0].legend()
        ax[1].legend()
        fig.suptitle("Circle-like formations")
    
        plt.show()
    
    # Lines
    plotLines = args.fall if args.fall else args.flines
    
    hline       = line_formation(N,sep)
    column      = column_formation(N,sep)
    right_ech   = echelon_right_formation(N,sep)
    left_ech    = echelon_left_formation(N,sep)
    
    right_ech_trd   = echelon_right_formation(N,sep,np.pi/3)
    left_ech_trd    = echelon_left_formation(N,sep,np.pi/3)
    
    if plotLines:
        fig,ax = plt.subplots(2,3,sharex='all',sharey='all')
        ax:list[list[Axes]]
        
        plot_formation(ax[0][0],hline   ,"Line")
        plot_formation(ax[1][0],column  ,"Column")
        
        plot_formation(ax[0][1],right_ech   ,"Echelon right, 45°")
        plot_formation(ax[1][1],left_ech    ,"Echelon left, 45°")
        
        plot_formation(ax[0][2],right_ech_trd   ,"Echelon right, 30°")
        plot_formation(ax[1][2],left_ech_trd    ,"Echelon left, 30°")
        
        for l in ax:
            for a in l:
                a.legend()
                
        fig.suptitle("Line-like formations")
    
        plt.show()
        
    # Vee and Chevron
    plotChevrons = args.fall if args.fall else args.fchev
    
    vee     = v_formation(N,sep)
    vee_p1  = v_formation(N+1,sep)
    vee_leftmost    = v_formation(ncols,sep,N_right=nlines)
    vee_rightmost   = v_formation(nlines,sep,N_right=ncols)
    
    chevron     = chevron_formation(N,sep)
    chevron_p1  = chevron_formation(N+1,sep)
    chevron_leftmost    = chevron_formation(ncols,sep,N_right=nlines)
    chevron_rightmost   = chevron_formation(nlines,sep,N_right=ncols)
    
    if plotChevrons:
        fig,ax = plt.subplots(2,4,sharex='all',sharey='all')
        ax:list[list[Axes]]
        
        plot_formation(ax[0][0],vee     ,f"V, {N} agents")
        plot_formation(ax[0][1],vee_p1  ,f"V, {N+1} agents")
        plot_formation(ax[0][2],vee_leftmost    ,f"V, {ncols} left for {nlines} right")
        plot_formation(ax[0][3],vee_rightmost   ,f"V, {nlines} left for {ncols} right")
        
        plot_formation(ax[1][0],chevron     ,f"Chevron, {N} agents")
        plot_formation(ax[1][1],chevron_p1  ,f"Chevron, {N+1} agents")
        plot_formation(ax[1][2],chevron_leftmost    ,f"Chevron, {ncols} left for {nlines} right")
        plot_formation(ax[1][3],chevron_rightmost   ,f"Chevron, {nlines} left for {ncols} right")
        
        for l in ax:
            for a in l:
                a.legend()
        
        fig.suptitle("Chevron-like formations")
    
        plt.show()
    
    # Multi-lines formations
    plotRectangles = args.fall if args.fall else args.fsquare
    
    rectangle   = rectangle_formation(nlines,ncols,sep,0.).to_barycentric_coords()
    co_rect     = rectangle_formation(nlines,ncols,sep,np.pi/2).to_barycentric_coords()
    diamond     = rectangle_formation(nlines,ncols,sep,np.pi/4).to_barycentric_coords()
    diamond_pi3rd = rectangle_formation(nlines,ncols,sep,np.pi/3).to_barycentric_coords()
    
    staggered   = staggered_formation(nlines,ncols,sep,0.).to_barycentric_coords()
    stacked_chevrons = stacked_chevrons_formation(nlines,ncols,sep,ysep).to_barycentric_coords()
    stacked_right_echelon = stacked_echelon_formation(nlines,ncols,sep,ysep).to_barycentric_coords()
    stacked_left_echelon = stacked_echelon_formation(nlines,ncols,sep,ysep,right=False).to_barycentric_coords()
    
    if plotRectangles:
        fig,ax = plt.subplots(2,4,sharex='all',sharey='all')
        ax:list[list[Axes]]
        
        plot_formation(ax[0][0],rectangle               ,f"Rectangle, {nlines}x{ncols}")
        plot_formation(ax[0][1],co_rect                 ,f"Rectangle, {ncols}x{nlines}")
        plot_formation(ax[0][2],diamond                 ,f"Diamond, {nlines} left, {ncols} right")
        plot_formation(ax[0][3],diamond_pi3rd           ,f"Diamond (pi/3), {nlines} left, {ncols} right")
        plot_formation(ax[1][0],staggered               ,f"Staggered rows, {nlines}x{ncols}")
        plot_formation(ax[1][1],stacked_chevrons        ,f"Stacked chevrons, {nlines}x{ncols}")
        plot_formation(ax[1][2],stacked_right_echelon   ,f"Stacked right echelons, {nlines}x{ncols}")
        plot_formation(ax[1][3],stacked_left_echelon    ,f"Stacked left echelons, {nlines}x{ncols}")
        
        for l in ax:
            for a in l:
                a.legend()
        
        fig.suptitle("Square-like formations")
    
        plt.show()