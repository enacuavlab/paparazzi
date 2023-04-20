#!/usr/bin/env python3

# Core Python imports 
from __future__ import annotations

import sys
import argparse
import typing
from dataclasses import dataclass,field,InitVar
import inspect
import itertools

# Paparazzi interaction
from .pprzInterface import AC_DataCollector,Aircraft
from trajectories.gvf_trajectories import Trajectory,LineTrajectory

# Computations
import numpy as np

# Graphing with Matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure,SubFigure
from matplotlib.lines import Line2D
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.quiver import Quiver
from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection,Line3D

#################### Aircraft figures ####################

@dataclass
class AC_drawings():
    """
    Class holding the different lines plotted for an aircraft
    """
    id:int # Aircraft ID
    name:str # Aircraft Name

    a3d:InitVar[Axes3D] # 3D axis, used for building the initial 3D lines
    axy:InitVar[Axes] # 2D axis, used for building the initial 2D lines
    axz:InitVar[Axes] # 2D axis, used for building the initial 2D lines
    ayz:InitVar[Axes] # 2D axis, used for building the initial 2D lines
    
    color:InitVar[str] = 'green' # Color associated to this aircraft
    pos_fmt:InitVar[str] = 'o' # Matplotlib formating string for the aircraft POSITION point
    carrot_fmt:InitVar[str] = 'p' # Matplotlib formating string for the aircraft CARROT point
    traj_fmt:InitVar[str] = ':' # Matplotlib formating string for the aircraft INTENDED TRAJECTORY line
    past_fmt:InitVar[str] = '-' # Matplotlib formating string for the aircraft EFFECTIVE TRAJECTORY line
    
    quiver_dist:InitVar[float] = 10
    quiver_res:InitVar[int] = 10
    quiver_kwargs:InitVar[typing.Dict] = {'pivot':'middle','norm':Normalize(-90,90)} # Additional Matplotlib arguments for the GVF Quiver 2D plots
    
    ac_pos_3d:Line3D = field(init=False) # Aircraft position 'line' (actually a point) in 3D space
    ac_pos_xy:Line2D = field(init=False) # Aircraft position 'line' (actually a point) in XY projection
    ac_pos_xz:Line2D = field(init=False) # Aircraft position 'line' (actually a point) in XZ projection
    ac_pos_yz:Line2D = field(init=False) # Aircraft position 'line' (actually a point) in YZ projection
    
    carrot_pos_3d:Line3D = field(init=False) # Carrot position 'line' (actually a point) in 3D space
    carrot_pos_xy:Line2D = field(init=False) # Carrot position 'line' (actually a point) in XY projection
    carrot_pos_xz:Line2D = field(init=False) # Carrot position 'line' (actually a point) in XZ projection
    carrot_pos_yz:Line2D = field(init=False) # Carrot position 'line' (actually a point) in YZ projection
    
    ac_traj_3d:Line3D = field(init=False) # Intended trajectory in 3D space
    ac_traj_xy:Line2D = field(init=False) # Intended trajectory in XY projection
    ac_traj_xz:Line2D = field(init=False) # Intended trajectory in XZ projection
    ac_traj_yz:Line2D = field(init=False) # Intended trajectory in YZ projection
    
    ac_past_3d:Line3D = field(init=False) # Effective trajectory in 3D space
    ac_past_xy:Line2D = field(init=False) # Effective trajectory in XY projection
    ac_past_xz:Line2D = field(init=False) # Effective trajectory in XZ projection
    ac_past_yz:Line2D = field(init=False) # Effective trajectory in YZ projection
    
    # gvf_3d:Line3DCollection = field(init=False) # Guiding Vector Field (Quiver plot) in 3D space (UNUSED AS UNREADABLE)
    gvf_xy:Quiver = field(init=False) # Guiding Vector Field (Quiver plot) in XY projection
    gvf_xz:Quiver = field(init=False) # Guiding Vector Field (Quiver plot) in XZ projection
    gvf_yz:Quiver = field(init=False) # Guiding Vector Field (Quiver plot) in YZ projection
    
    def __post_init__(self,a3d:Axes3D,axy:Axes,axz:Axes,ayz:Axes,
                      color:str,pos_fmt:str,carrot_fmt:str,traj_fmt:str,past_fmt:str,
                      quiver_dist:float,quiver_res:int,quiver_kwargs:typing.Dict):
        """Setup the different lines from the Init only arguments
        (axes from which to build the lines, format specifications)
        """
        
        if self.name is None:
            self.name = str(self.id)
        
        self.ac_pos_3d = a3d.plot([],[],[],pos_fmt,color=color)[0]
        self.carrot_pos_3d = a3d.plot([],[],[],carrot_fmt,color=color)[0]
        self.ac_traj_3d = a3d.plot([],[],[],traj_fmt,color=color)[0]
        self.ac_past_3d = a3d.plot([],[],[],past_fmt,color=color)[0]
        
        self.ac_pos_xy = axy.plot([],[],pos_fmt,color=color)[0]
        self.carrot_pos_xy = axy.plot([],[],carrot_fmt,color=color)[0]
        self.ac_traj_xy = axy.plot([],[],traj_fmt,color=color)[0]
        self.ac_past_xy = axy.plot([],[],past_fmt,color=color)[0]
        
        self.ac_pos_xz = axz.plot([],[],pos_fmt,color=color)[0]
        self.carrot_pos_xz = axz.plot([],[],carrot_fmt,color=color)[0]
        self.ac_traj_xz = axz.plot([],[],traj_fmt,color=color)[0]
        self.ac_past_xz = axz.plot([],[],past_fmt,color=color)[0]
        
        self.ac_pos_yz = ayz.plot([],[],pos_fmt,color=color)[0]
        self.carrot_pos_yz = ayz.plot([],[],carrot_fmt,color=color)[0]
        self.ac_traj_yz = ayz.plot([],[],traj_fmt,color=color)[0]
        self.ac_past_yz = ayz.plot([],[],past_fmt,color=color)[0]
        
        XY_meshgrid = np.asarray(np.meshgrid(np.linspace(-quiver_dist, quiver_dist, quiver_res),
                                np.linspace(-quiver_dist, +quiver_dist, quiver_res),
                                0,
                                indexing='ij'))
        
        XZ_meshgrid = np.asarray(np.meshgrid(np.linspace(-quiver_dist, quiver_dist, quiver_res),
                                0,
                                np.linspace(-quiver_dist, quiver_dist, quiver_res),
                                indexing='ij'))
        
        YZ_meshgrid = np.asarray(np.meshgrid(0,
                                np.linspace(-quiver_dist, quiver_dist, quiver_res),
                                np.linspace(-quiver_dist, quiver_dist, quiver_res),
                                indexing='ij'))
        
        
        null_arrows = np.zeros((quiver_res,quiver_res))
        
        # self.gvf_3d = a3d.quiver([],[],[],[],[],[],**quiver3d_kwargs)
        self.gvf_xy = axy.quiver(XY_meshgrid[0][:,:,0],XY_meshgrid[1][:,:,0],null_arrows,null_arrows,null_arrows,**quiver_kwargs)
        self.gvf_xz = axz.quiver(XZ_meshgrid[0][:,0,:],XZ_meshgrid[2][:,0,:],null_arrows,null_arrows,null_arrows,**quiver_kwargs)
        self.gvf_yz = ayz.quiver(YZ_meshgrid[1][0,:,:],YZ_meshgrid[2][0,:,:],null_arrows,null_arrows,null_arrows,**quiver_kwargs)
        
    @staticmethod
    def from_Aircraft(a:Aircraft,a3d:Axes3D,axy:Axes,axz:Axes,ayz:Axes,
                      quiver_dist:float,quiver_res:int,**kwargs)-> AC_drawings:
        """
        Simplify construction by gathering some arguments from the 'Aircraft' structure
        (namely the name and color)
        """
        id = a.id
        try:
            name = a.config.ac_name
            color = a.config.color
        except AttributeError as e:
            print(f"Could not find 'CONFIG' message for aircraft of id {id}:")
            print(e)
            name = str(id)
            color = f'C{id % 10}'
            
        return AC_drawings(id,name,a3d,axy,axz,ayz,color=color,quiver_dist=quiver_dist,quiver_res=quiver_res,**kwargs)
            
#################### Main display ####################

class Trajectory3DMap():
    def __init__(self,ac_ids:typing.List[int],pprzinterface_name:str='GVF_collector',
                 w_dist:float = 400, 
                 show_field:bool = False, gvf_dist: float = 300, 
                 resolution: int = 10,
                 history_size:int = 10,
                 fps:int = 20,
                 ):
        
        # Paparazzi interface
        self.pprzinterface:AC_DataCollector
        if pprzinterface_name is None:
            self.pprzinterface = AC_DataCollector(ac_ids,'GVF_collector')
        else:
            self.pprzinterface = AC_DataCollector(ac_ids,pprzinterface_name)
            
        # Plotting parameters
        assert w_dist >= 0
        self.w_dist:float = w_dist
        
        assert isinstance(show_field,bool)
        self.show_field:bool = show_field
        
        assert gvf_dist >= 0
        self.gvf_dist:float = gvf_dist
        
        assert resolution >= 0
        self.resolution:int = resolution
        
        assert history_size >= 0
        self.history_size:int = history_size
            
        # Matplotlib figure
        self.main_fig = plt.figure()
        
        self.a3d:Axes3D = self.main_fig.add_subplot(2,2,1,projection='3d')
        self.axy:Axes = self.main_fig.add_subplot(2,2,2)
        self.axz:Axes = self.main_fig.add_subplot(2,2,3)
        self.ayz:Axes = self.main_fig.add_subplot(2,2,4)
        
        self.a3d.set_title('3D Map')
        self.axy.set_title('XY Map')
        self.axz.set_title('XZ Map')
        self.ayz.set_title('YZ Map')
        
        self.a3d.axis('equal')
        self.axy.axis('equal')
        self.axz.axis('equal')
        self.ayz.axis('equal')
        
        # Init lines for each aircraft
        self.drawings:typing.Dict[int,AC_drawings] = dict()
        for id in self.ac_ids:
            ac_data = self.ac_data_dict[id]
            self.drawings[id] = AC_drawings.from_Aircraft(ac_data,self.a3d,self.axy,self.axz,self.ayz,
                                                          self.gvf_dist,self.resolution)
            
        # Matplotlib Animator setup
        self.frame_interval = int(1000/fps)
        
        def animation_function(frame) -> typing.Iterable:
            return self.__update_all_AC_drawings(frame)
        
        def animation_init() -> typing.Iterable:
            return self.__all_updatable_AC_artists()
        
        
        self.animator = animation.FuncAnimation(self.main_fig,
                                                animation_function,
                                                #init_func=animation_init,
                                                blit=False,
                                                interval=self.frame_interval,
                                                save_count=5*fps)
        
    @property
    def ac_ids(self) -> typing.List[int]:
        return self.pprzinterface.ac_ids
    
    @property
    def ac_data_dict(self) -> typing.Dict[int,Aircraft]:
        return self.pprzinterface.ac_dict
    
    def get_aircraft(self,id:int) -> Aircraft:
        return self.pprzinterface.ac_dict[id]
    
    def __updatable_AC_artists(self,ac:typing.Union[int,Aircraft]) -> typing.Iterable:
        if not isinstance(ac,Aircraft):
            ac = self.get_aircraft(ac)
        
        drawings = self.drawings[ac.id]
        
        output = iter([drawings.ac_pos_3d,drawings.ac_pos_xy,drawings.ac_pos_xz,drawings.ac_pos_yz])
        if self.history_size > 1:
            past_output = iter([drawings.ac_past_3d,drawings.ac_past_xy,drawings.ac_past_xz,drawings.ac_past_yz])
            output = itertools.chain(output,past_output)
            
        carrot_output = iter([drawings.carrot_pos_3d,drawings.carrot_pos_xy,drawings.carrot_pos_xz,drawings.carrot_pos_yz])
        output = itertools.chain(output,carrot_output)
            
        traj_output = iter([drawings.ac_traj_3d,drawings.ac_traj_xy,drawings.ac_traj_xz,drawings.ac_traj_yz])
        output = itertools.chain(output,traj_output)
        
        if self.show_field:
            gvf_output = iter([drawings.gvf_xy,drawings.gvf_xz,drawings.gvf_yz])
            output = itertools.chain(output,gvf_output)
            
        return output
    
    def __all_updatable_AC_artists(self) -> typing.Iterable:
        output = iter([])
        for id in self.ac_ids:
            ac = self.get_aircraft(id)
            output = itertools.chain(output,self.__updatable_AC_artists(ac))
        
        return output
    
    def __update_AC_drawings(self,frame,ac:typing.Union[int,Aircraft]) -> typing.Iterable:
        if not isinstance(ac,Aircraft):
            ac = self.get_aircraft(ac)
        
        drawings = self.drawings[ac.id]
        
        # Position:
        XYZ = ac.XYZ
        
        drawings.ac_pos_3d.set_data_3d([XYZ[0]],[XYZ[1]],[XYZ[2]])
        drawings.ac_pos_xy.set_data([XYZ[0]],[XYZ[1]])
        drawings.ac_pos_xz.set_data([XYZ[0]],[XYZ[2]])
        drawings.ac_pos_yz.set_data([XYZ[1]],[XYZ[2]])
        
        output = iter([drawings.ac_pos_3d,drawings.ac_pos_xy,drawings.ac_pos_xz,drawings.ac_pos_yz])
        
        # Past:
        if self.history_size > 1:
            # 3D
            xs,ys,zs = drawings.ac_past_3d.get_data_3d()
            if len(xs) < self.history_size:
                xs = np.append(xs,XYZ[0])
            else:
                xs = np.roll(xs,-1)
                xs[-1] = XYZ[0]
            
            if len(ys) < self.history_size:
                ys = np.append(ys,XYZ[1])
            else:
                ys = np.roll(ys,-1)
                ys[-1] = XYZ[1]
                
            if len(zs) < self.history_size:
                zs = np.append(zs,XYZ[2])
            else:
                zs = np.roll(zs,-1)
                zs[-1] = XYZ[2]
            
            drawings.ac_past_3d.set_data_3d(xs,ys,zs)
            drawings.ac_past_xy.set_data(xs,ys)
            drawings.ac_past_xz.set_data(xs,zs)
            drawings.ac_past_yz.set_data(ys,zs)
            
            past_output = iter([drawings.ac_past_3d,drawings.ac_past_xy,drawings.ac_past_xz,drawings.ac_past_yz])
            output = itertools.chain(output,past_output)
            
        # Trajectory-related
        traj = ac.trajectory
        if traj is not None:
            # Carrot
            w = ac.w
            if w is not None:
                CARROT = traj.param_point(w)
                
                drawings.carrot_pos_3d.set_data_3d([CARROT[0]],[CARROT[1]],[CARROT[2]])
                drawings.carrot_pos_xy.set_data([CARROT[0]],[CARROT[1]])
                drawings.carrot_pos_xz.set_data([CARROT[0]],[CARROT[2]])
                drawings.carrot_pos_yz.set_data([CARROT[1]],[CARROT[2]])
                
                carrot_output = iter([drawings.carrot_pos_3d,drawings.carrot_pos_xy,drawings.carrot_pos_xz,drawings.carrot_pos_yz])
                output = itertools.chain(output,carrot_output)
            
            else:
                # For closed trajectory/non-parametric compatibility
                w = 0    
                
            # Trajectory
            # Work only with Genus-1 trajectories (that is a 'line', only one parameter, mappable on R)
            assert isinstance(traj,LineTrajectory)
            
            traj_points = traj.calc_traj(np.linspace(w-self.w_dist,w+self.w_dist,self.resolution*10))
            drawings.ac_traj_3d.set_data_3d(traj_points[:,0], traj_points[:,1], traj_points[:,2])
            drawings.ac_traj_xy.set_data(traj_points[:,0],traj_points[:,1])
            drawings.ac_traj_xz.set_data(traj_points[:,0],traj_points[:,2])
            drawings.ac_traj_yz.set_data(traj_points[:,1],traj_points[:,2])
            
            traj_output = iter([drawings.ac_traj_3d,drawings.ac_traj_xy,drawings.ac_traj_xz,drawings.ac_traj_yz])
            output = itertools.chain(output,traj_output)
            
            # GVF
            if self.show_field:
                # XYZ_meshgrid = np.asarray(np.meshgrid(np.linspace(XYZ[0]-self.gvf_dist, XYZ[0]+self.gvf_dist, self.resolution),
                #                       np.linspace(XYZ[1]-self.gvf_dist, XYZ[1]+self.gvf_dist, self.resolution),
                #                       np.linspace(XYZ[2]-self.gvf_dist, XYZ[2]+self.gvf_dist, self.resolution),indexing='ij'))
                # vector_field = traj.calc_field(XYZ_meshgrid.transpose(), w).transpose()
                
                XY_meshgrid = np.asarray(np.meshgrid(np.linspace(XYZ[0]-self.gvf_dist, XYZ[0]+self.gvf_dist, self.resolution),
                                      np.linspace(XYZ[1]-self.gvf_dist, XYZ[1]+self.gvf_dist, self.resolution),
                                      XYZ[2],
                                      indexing='ij'))
                
                XY_vectorfield = traj.calc_field(XY_meshgrid.transpose(), w).transpose()
                
                XZ_meshgrid = np.asarray(np.meshgrid(np.linspace(XYZ[0]-self.gvf_dist, XYZ[0]+self.gvf_dist, self.resolution),
                                      XYZ[1],
                                      np.linspace(XYZ[2]-self.gvf_dist, XYZ[2]+self.gvf_dist, self.resolution),
                                      indexing='ij'))
                
                XZ_vectorfield = traj.calc_field(XZ_meshgrid.transpose(), w).transpose()
                
                YZ_meshgrid = np.asarray(np.meshgrid(XYZ[0],
                                      np.linspace(XYZ[1]-self.gvf_dist, XYZ[1]+self.gvf_dist, self.resolution),
                                      np.linspace(XYZ[2]-self.gvf_dist, XYZ[2]+self.gvf_dist, self.resolution),
                                      indexing='ij'))
                
                YZ_vectorfield = traj.calc_field(YZ_meshgrid.transpose(), w).transpose()
            
                
                
                
                z_angles_field = np.rad2deg(np.arctan2(XY_vectorfield[2][:,:,0],np.sqrt(np.square(XY_vectorfield[0][:,:,0])+np.square(XY_vectorfield[1][:,:,0]))))
                drawings.gvf_xy.set_offsets([XYZ[0],XYZ[1]])
                drawings.gvf_xy.set_UVC(XY_vectorfield[0][:,:,0],XY_vectorfield[1][:,:,0],z_angles_field)
                
                y_angles_field = np.rad2deg(np.arctan2(XZ_vectorfield[1][:,0,:],np.sqrt(np.square(XZ_vectorfield[0][:,0,:])+np.square(XZ_vectorfield[2][:,0,:]))))
                drawings.gvf_xz.set_offsets([XYZ[0],XYZ[2]])
                drawings.gvf_xz.set_UVC(XZ_vectorfield[0][:,0,:],XZ_vectorfield[2][:,0,:],y_angles_field)
                
                x_angles_field = np.rad2deg(np.arctan2(YZ_vectorfield[0][0,:,:],np.sqrt(np.square(YZ_vectorfield[1][0,:,:])+np.square(YZ_vectorfield[2][0,:,:]))))
                drawings.gvf_yz.set_offsets([XYZ[1],XYZ[2]])
                drawings.gvf_yz.set_UVC(YZ_vectorfield[1][0,:,:],YZ_vectorfield[2][0,:,:],x_angles_field)
            
                gvf_output = iter([drawings.gvf_xy,drawings.gvf_xz,drawings.gvf_yz])
                output = itertools.chain(output,gvf_output)
            
        return output    
        
        
    def __update_all_AC_drawings(self,frame) -> typing.Iterable:
        output = iter([])
        for id in self.ac_ids:
            ac = self.get_aircraft(id)
            output = itertools.chain(output,self.__update_AC_drawings(frame,ac))
            
        #self.a3d.autoscale()
        #self.axy.autoscale()
        #self.axz.autoscale()
        #self.ayz.autoscale()
        return output
            
        
    def start(self) -> None:
        plt.show()

def main():
    print("Test")
    map = Trajectory3DMap([5])
    map.start()