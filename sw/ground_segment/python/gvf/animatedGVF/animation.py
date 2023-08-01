#!/usr/bin/env python3

# Core Python imports 
from __future__ import annotations

import sys
import argparse
import typing
from dataclasses import dataclass,field,InitVar
import inspect
import itertools
import functools

# Paparazzi interaction
from .pprzInterface import AC_DataCollector,Aircraft
from trajectories.gvf_trajectories import Trajectory,LineTrajectory
from trajectories.GVF_PARAMETRIC import ParametricLineTrajectory

# Computations
import numpy as np

# Graphing with Matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.artist import Artist
from matplotlib.figure import Figure,SubFigure
from matplotlib.legend_handler import HandlerTuple
from matplotlib.lines import Line2D
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.quiver import Quiver
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle
from matplotlib.widgets import Button,CheckButtons,RadioButtons
from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection,Line3D


########## Nice legend handler for lines ##########

# Found at https://stackoverflow.com/questions/31544489/two-line-styles-in-legend

class HandlerTupleVertical(HandlerTuple):
    def __init__(self, **kwargs):
        HandlerTuple.__init__(self, **kwargs)

    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize, trans):
        # How many lines are there.
        numlines = len(orig_handle)
        handler_map = legend.get_legend_handler_map()

        # divide the vertical space where the lines will go
        # into equal parts based on the number of lines
        height_y = (height / numlines)

        leglines = []
        for i, handle in enumerate(orig_handle):
            handler = legend.get_legend_handler(handler_map, handle)

            legline = handler.create_artists(legend, handle,
                                             xdescent,
                                             (2*i + 1)*height_y,
                                             width,
                                             2*height,
                                             fontsize, trans)
            leglines.extend(legline)

        return leglines

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
    carrot_fmt:InitVar[str] = 'P' # Matplotlib formating string for the aircraft CARROT point
    traj_fmt:InitVar[str] = ':' # Matplotlib formating string for the aircraft INTENDED TRAJECTORY line
    past_fmt:InitVar[str] = '-' # Matplotlib formating string for the aircraft EFFECTIVE TRAJECTORY line
    
    quiver_dist:InitVar[float] = 10 # Maximal distance (in infinite norm) away from current position for plotting GVF 
    quiver_res:InitVar[int] = 10 # Number of samples (per dimension) for plotting GVF
    quiver_kwargs:InitVar[typing.Dict] = {'pivot':'mid','norm':Normalize(-90,90),'cmap':'seismic',
                                          'width':0.005, 'edgecolor':'Black','linewidth':0.2,
                                          'scale':0.01,'scale_units':'xy','angles':'xy'} # Additional Matplotlib arguments for the GVF Quiver 2D plots
    
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

        if type(color) == str:
            if color[0] == '#' and len(color) > len('#0f0f0f80'):
                # If the color uses more than 8 bits per color channel...
                # ... Transform it into a RGB float tuple (naively)
                colorstring = color[1:]
                hexlen = len(colorstring)//3
                color = tuple(int(colorstring[i*hexlen:(i+1)*hexlen],16)/(16**hexlen) for i in range(3))
                
        
        if self.name is None:
            self.name = f"AC {self.id}"

        self.ac_pos_3d = a3d.plot([],[],[],pos_fmt,color=color,label=f"{self.name} 's position")[0]
        self.carrot_pos_3d = a3d.plot([],[],[],carrot_fmt,color=color,label=f"{self.name} 's guiding point")[0]
        self.ac_traj_3d = a3d.plot([],[],[],traj_fmt,color=color,label=f"{self.name} 's path")[0]
        self.ac_past_3d = a3d.plot([],[],[],past_fmt,color=color,label=f"{self.name} 's trajectory")[0]
        
        self.ac_pos_xy = axy.plot([],[],pos_fmt,color=color,label=f"{self.name} 's position")[0]
        self.carrot_pos_xy = axy.plot([],[],carrot_fmt,color=color,label=f"{self.name} 's guiding point")[0]
        self.ac_traj_xy = axy.plot([],[],traj_fmt,color=color,label=f"{self.name} 's path")[0]
        self.ac_past_xy = axy.plot([],[],past_fmt,color=color,label=f"{self.name} 's trajectory")[0]
        
        self.ac_pos_xz = axz.plot([],[],pos_fmt,color=color,label=f"{self.name} 's position")[0]
        self.carrot_pos_xz = axz.plot([],[],carrot_fmt,color=color,label=f"{self.name} 's guiding point")[0]
        self.ac_traj_xz = axz.plot([],[],traj_fmt,color=color,label=f"{self.name} 's path")[0]
        self.ac_past_xz = axz.plot([],[],past_fmt,color=color,label=f"{self.name} 's trajectory")[0]
        
        self.ac_pos_yz = ayz.plot([],[],pos_fmt,color=color,label=f"{self.name} 's position")[0]
        self.carrot_pos_yz = ayz.plot([],[],carrot_fmt,color=color,label=f"{self.name} 's guiding point")[0]
        self.ac_traj_yz = ayz.plot([],[],traj_fmt,color=color,label=f"{self.name} 's path")[0]
        self.ac_past_yz = ayz.plot([],[],past_fmt,color=color,label=f"{self.name} 's trajectory")[0]
        
        base_axis = np.linspace(-quiver_dist, quiver_dist, quiver_res)
        null_arrows = np.zeros((quiver_res,quiver_res))
        
        # self.gvf_3d = a3d.quiver([],[],[],[],[],[],**quiver3d_kwargs)
        self.gvf_xy = axy.quiver(base_axis,base_axis,null_arrows,null_arrows,null_arrows,**quiver_kwargs)
        self.gvf_xz = axz.quiver(base_axis,base_axis,null_arrows,null_arrows,null_arrows,**quiver_kwargs)
        self.gvf_yz = ayz.quiver(base_axis,base_axis,null_arrows,null_arrows,null_arrows,**quiver_kwargs)
        
        self.base_flat_grid = self.gvf_xy.get_offsets()
        
    
    def get_3d_artists(self) -> typing.List[Line3D]:
        return [self.ac_pos_3d,self.carrot_pos_3d,self.ac_traj_3d,self.ac_past_3d]
    
    def get_xy_artists(self) -> typing.List[Line2D]:
        return [self.ac_pos_xy,self.carrot_pos_xy,self.ac_traj_xy,self.ac_past_xy]
    
    def get_xz_artists(self) -> typing.List[Line2D]:
        return [self.ac_pos_xz,self.carrot_pos_xz,self.ac_traj_xz,self.ac_past_xz]
    
    def get_yz_artists(self) -> typing.List[Line2D]:
        return [self.ac_pos_yz,self.carrot_pos_yz,self.ac_traj_yz,self.ac_past_yz]
    
    def get_all_artists(self) -> typing.List[Artist]:
        return self.get_3d_artists() + self.get_xy_artists() + self.get_xz_artists() + self.get_yz_artists()

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

#################### AC tracking ####################

@dataclass
class TrackingCode():
    code:int
    
    def to_none(self):
        self.code = -1
        
    def to_barycenter(self):
        self.code = -2
    
    @property
    def is_none(self) -> bool:
        return self.code == -1
    
    @property
    def is_barycenter(self) -> bool:
        return self.code == -2
    
    @property
    def is_ac_id(self) -> bool:
        return self.code >= 0
    
    @staticmethod
    def none() -> TrackingCode:
        return TrackingCode(-1)
    
    @staticmethod
    def barycenter() -> TrackingCode:
        return TrackingCode(-2)
    
    def __str__(self) -> str:
        if self.is_none:
            return "Nothing"
        elif self.is_barycenter:
            return "Barycenter"
        else:
            return str(self.code)
    
    
    

            
#################### Main display ####################

class Trajectory3DMap():
    def __init__(self,ac_ids:typing.List[int],pprzinterface_name:str='GVF_collector',
                 w_dist:float = 400, 
                 show_field:bool = False, gvf_dist: float = 300, 
                 resolution: int = 10,
                 history_size:int = 10,
                 fps:int = 20,
                 normalized_fun:bool = False
                 ):
        
        ## Paparazzi interface
        self.pprzinterface:AC_DataCollector
        if pprzinterface_name is None:
            self.pprzinterface = AC_DataCollector(ac_ids,'GVF_collector')
        else:
            self.pprzinterface = AC_DataCollector(ac_ids,pprzinterface_name)
            
        ## Plotting parameters
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
        
        self.normalized_fun:bool = normalized_fun
            
        ## Matplotlib figure
        self.main_fig = plt.figure()
        grid = GridSpec(1,2,self.main_fig,width_ratios=[0.2,0.8])
        
        
        # Setting drawing
        self.plot_fig = self.main_fig.add_subfigure(grid[1])
        
        self.a3d:Axes3D = self.plot_fig.add_subplot(2,2,1,projection='3d')
        self.axy:Axes = self.plot_fig.add_subplot(2,2,2)
        self.axz:Axes = self.plot_fig.add_subplot(2,2,3)
        self.ayz:Axes = self.plot_fig.add_subplot(2,2,4)
        
        self.a3d.set_title('3D Map')
        self.axy.set_title('XY Map')
        self.axz.set_title('XZ Map')
        self.ayz.set_title('YZ Map')
        
        self.a3d.set_xlabel("Easting (m)")
        self.a3d.set_ylabel("Northing (m)")
        self.a3d.set_zlabel("Altitude (m)")

        self.axy.set_xlabel("Easting (m)")
        self.axy.set_ylabel("Northing (m)")

        self.axz.set_xlabel("Easting (m)")
        self.axz.set_ylabel("Altitude (m)")

        self.ayz.set_xlabel("Northing (m)")
        self.ayz.set_ylabel("Altitude (m)")

        self.a3d.axis('equal')
        self.axy.axis('equal')
        self.axz.axis('equal')
        self.ayz.axis('equal')
        
        # Setting buttons
        self.button_fig = self.main_fig.add_subfigure(grid[0])
        
        button_col = 2 # One for legend, One for PushButtons
        button_lines = len(ac_ids) + 5 # One per AC, + 1 for 'Normalize' checkbox, + 1 for 'Tracking' checkbox, + 1 for 'Reset', + 1 for 'Barycenter', + 1 for legends
        
        matrix_to_flat_index = lambda i,j : button_col * i + j + 1
        
        self.normalize:bool = show_field
        self.normalize_checkbox_ax:Axes = self.button_fig.add_subplot(button_lines,button_col,matrix_to_flat_index(1,1),frameon=False)
        self.normalize_checkbox:CheckButtons = CheckButtons(self.normalize_checkbox_ax,
                                                           labels=["Normalize field"],
                                                           actives=[show_field], # Auto-active if field is shown
                                                           )
        
        def wrapped__normalize_toggle(e):
            self.__normalize_toggle()
            
        self.normalize_checkbox.on_clicked(wrapped__normalize_toggle)
        
        self.tracking:bool = len(ac_ids) == 1 # Auto-active if only one AC, disabled otherwise
        self.tracking_checkbox_ax:Axes = self.button_fig.add_subplot(button_lines,button_col,matrix_to_flat_index(2,1),frameon=False)
        self.tracking_checkbox:CheckButtons = CheckButtons(self.tracking_checkbox_ax,
                                                           labels=["Tracking"],
                                                           actives=[len(ac_ids) == 1], # Auto-active if only one AC, disabled otherwise
                                                           )
        
        def wrapped__tracking_toggle(e):
            self.__tracking_toggle()
        
        self.tracking_checkbox.on_clicked(wrapped__tracking_toggle)
        
        self.track_target_code:TrackingCode = TrackingCode.none()
        
        self.reset_button_ax:Axes = self.button_fig.add_subplot(button_lines,button_col,matrix_to_flat_index(3,1))
        self.bary_button_ax:Axes = self.button_fig.add_subplot(button_lines,button_col,matrix_to_flat_index(4,1))
        self.ac_buttons_axes:typing.List[Axes] = [self.button_fig.add_subplot(button_lines,button_col,matrix_to_flat_index(5+i,1)) for i in range(len(ac_ids))]
        
        self.reset_button:Button = Button(self.reset_button_ax,
                                          'Reset')
        def track_reset(e):
            self.track_target_code.to_none()
        self.reset_button.on_clicked(track_reset)        
        
        
        self.bary_button:Button = Button(self.bary_button_ax,
                                         'Barycenter')
        def track_barycenter(e):
            self.track_target_code.to_barycenter()
        self.bary_button.on_clicked(track_barycenter)
        
        
        self.ac_buttons:typing.List[Button] = [Button(self.ac_buttons_axes[i],self.get_aircraft(ac_ids[i]).name) for i in range(len(ac_ids))]
        def track_ac(e,id):
            print(id)
            self.track_target_code.code = id
        for id,b in zip(ac_ids,self.ac_buttons):
            b.on_clicked(functools.partial(track_ac,id=id))
        
        
        ## Init lines for each aircraft
        self.drawings:typing.Dict[int,AC_drawings] = dict()
        for id in self.ac_ids:
            ac_data = self.get_aircraft(id)
            self.drawings[id] = AC_drawings.from_Aircraft(ac_data,self.a3d,self.axy,self.axz,self.ayz,
                                                          self.gvf_dist,self.resolution)
        
        ac_handles = list(d.ac_pos_xy for d in self.drawings.values())
        ac_labels = list(d.name for d in self.drawings.values())
        guiding_point_handle = tuple(d.carrot_pos_xy for d in self.drawings.values())
        line_handle = tuple(d.ac_traj_xy for d in self.drawings.values())
        traj_handle = tuple(d.ac_past_xy for d in self.drawings.values())
        
        self.plot_fig.legend(ac_handles+[guiding_point_handle,line_handle,traj_handle],
                             ac_labels + ["Guiding point","Intended path","Trajectory"],
                             loc='outside left upper',
                             handler_map={tuple: HandlerTupleVertical(ndivide=None)})
        
        
        # If GVF is enabled, put a colorbar
        if self.show_field:
            # Pick any AC
            id = self.ac_ids[0]
            self.main_fig.colorbar(self.drawings[id].gvf_xy,label="Orthogonal inclination (in degrees, positive is \u2299)")
            self.main_fig.colorbar(self.drawings[id].gvf_xz,label="Orthogonal inclination (in degrees, positive is \u2299)")
            self.main_fig.colorbar(self.drawings[id].gvf_yz,label="Orthogonal inclination (in degrees, positive is \u2299)")
        
        ## Matplotlib Animator setup
        self.frame_interval = int(1000/fps)
        self.close_window:bool = False
        
        def wrapped_close(any):
            self.close(any)
        self.main_fig.canvas.mpl_connect('close_event',wrapped_close)
        
        def animation_function(frame) -> typing.Iterable:
            self.__center_axes()
            #print(self.track_target_code)
            return self.__update_all_AC_drawings(frame)
        
        def animation_init() -> typing.Iterable:
            return self.__all_updatable_AC_artists()
        
        def frame_generator() -> int:
            frame_nbr = 0
            while not self.close_window:
                yield frame_nbr
                frame_nbr += 1
        
        
        self.animator = animation.FuncAnimation(self.main_fig,
                                                animation_function,
                                                init_func=animation_init,
                                                blit=False,
                                                interval=self.frame_interval,
                                                save_count=5*fps,
                                                frames=frame_generator)
    
    def __normalize_toggle(self):
        self.normalize = not self.normalize
    
    def __tracking_toggle(self):
        self.tracking = not self.tracking
        
    def close(self,any) -> None:
        self.close_window = True
        self.pprzinterface.shutdown()
        
    def __del__(self):
        self.close(None)
        
    @property
    def ac_ids(self) -> typing.List[int]:
        return self.pprzinterface.ac_ids
    
    @property
    def ac_data_dict(self) -> typing.Dict[int,Aircraft]:
        return self.pprzinterface.ac_dict
    
    def get_aircraft(self,id:int) -> Aircraft:
        return self.pprzinterface.ac_dict[id]
    
    def get_all_aircraft_pos(self) -> np.ndarray:
        all_aircrafts = self.ac_data_dict.values()
        return np.asarray([ac.XYZ for ac in all_aircrafts])
        
    
    def __center_axes(self) -> None:
        if not self.tracking:
            return
        
        if self.track_target_code.is_none:
            track_point = np.zeros(3)
        elif self.track_target_code.is_barycenter:
            positions = self.get_all_aircraft_pos()
            track_point = np.average(positions,axis=0)
        else:
            track_point = self.get_aircraft(self.track_target_code.code).XYZ
        
        def set_limits(ax:Axes,xcenter,ycenter):
            xleft,xright = ax.get_xlim()
            xdelta = xright - xleft
            ax.set_xlim(left=xcenter - xdelta/2,right=xcenter + xdelta/2)
            
            ybot,ytop = ax.get_ylim()
            ydelta = ytop - ybot
            ax.set_ylim(bottom=ycenter - ydelta/2,top=ycenter + ydelta/2)
            
        set_limits(self.axy,track_point[0],track_point[1])
        set_limits(self.axz,track_point[0],track_point[2])
        set_limits(self.ayz,track_point[1],track_point[2])
        
        xleft,xright = self.a3d.get_xlim()
        xdelta = xright - xleft
        self.a3d.set_xlim((track_point[0] - xdelta/2, track_point[0] + xdelta/2))
        

        ybot,ytop = self.a3d.get_ylim()
        ydelta = ytop - ybot
        self.a3d.set_ylim((track_point[1] - ydelta/2, track_point[1] + ydelta/2))


        zback,zforw = self.a3d.get_zlim()
        zdelta = zforw - zback
        self.a3d.set_zlim((track_point[2] - zdelta/2, track_point[2] + zdelta/2))


        
    
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
        else:
            drawings.gvf_xy.set_visible(False)
            drawings.gvf_xz.set_visible(False)
            drawings.gvf_yz.set_visible(False)
            
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
            traj.gain = ac.gains
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
                
                if isinstance(traj,ParametricLineTrajectory):
                    traj.beta_s = ac.beta_s
                    traj.L = ac.L
                
                xy_offsets = drawings.base_flat_grid + np.asarray([XYZ[0],XYZ[1]])
                xz_offsets = drawings.base_flat_grid + np.asarray([XYZ[0],XYZ[2]])
                yz_offsets = drawings.base_flat_grid + np.asarray([XYZ[1],XYZ[2]])
                
                        
                XY_vec_list = np.stack([xy_offsets[:,0],xy_offsets[:,1],np.repeat(XYZ[2],len(xy_offsets[:,0]))])
                XZ_vec_list = np.stack([xz_offsets[:,0],np.repeat(XYZ[1],len(xz_offsets[:,0])),xz_offsets[:,1]])
                YZ_vec_list = np.stack([np.repeat(XYZ[0],len(yz_offsets[:,0])),yz_offsets[:,0],yz_offsets[:,1]])
                
                
                XY_vectorfield = traj.calc_field(XY_vec_list.transpose(), w, self.normalize,self.normalized_fun).transpose()
                XZ_vectorfield = traj.calc_field(XZ_vec_list.transpose(), w, self.normalize,self.normalized_fun).transpose()
                YZ_vectorfield = traj.calc_field(YZ_vec_list.transpose(), w, self.normalize,self.normalized_fun).transpose()
                
                
                # Global Scale Normalization
                XY_max_norm = np.sqrt(np.max(np.square(XY_vectorfield[0])+np.square(XY_vectorfield[1])+np.square(XY_vectorfield[2])))
                XY_vectorfield /= XY_max_norm

                XZ_max_norm = np.sqrt(np.max(np.square(XZ_vectorfield[0])+np.square(XZ_vectorfield[1])+np.square(XZ_vectorfield[2])))
                XZ_vectorfield /= XZ_max_norm

                YZ_max_norm = np.sqrt(np.max(np.square(YZ_vectorfield[0])+np.square(YZ_vectorfield[1])+np.square(YZ_vectorfield[2])))
                YZ_vectorfield /= YZ_max_norm
                
                # Compute angles 
                z_angles_field = np.rad2deg(np.arctan2(XY_vectorfield[2],np.sqrt(np.square(XY_vectorfield[0])+np.square(XY_vectorfield[1]))))
                y_angles_field = np.rad2deg(np.arctan2(XZ_vectorfield[1],np.sqrt(np.square(XZ_vectorfield[0])+np.square(XZ_vectorfield[2]))))
                x_angles_field = np.rad2deg(np.arctan2(YZ_vectorfield[0],np.sqrt(np.square(YZ_vectorfield[1])+np.square(YZ_vectorfield[2]))))
                
                drawings.gvf_xy.set_offsets(xy_offsets)
                drawings.gvf_xy.set_UVC(XY_vectorfield[0],XY_vectorfield[1],z_angles_field)
                
                drawings.gvf_xz.set_offsets(xz_offsets)
                drawings.gvf_xz.set_UVC(XZ_vectorfield[0],XZ_vectorfield[2],y_angles_field)
                
                drawings.gvf_yz.set_offsets(yz_offsets)
                drawings.gvf_yz.set_UVC(YZ_vectorfield[1],YZ_vectorfield[2],x_angles_field)
                
                #drawings.gvf_xy.set_norm(Normalize(-90,90))                
                #drawings.gvf_xz.set_norm(Normalize(-90,90))
                #drawings.gvf_yz.set_norm(Normalize(-90,90))

                #drawings.gvf_xy.autoscale()
                #drawings.gvf_xz.autoscale()
                #drawings.gvf_yz.autoscale()
            
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
