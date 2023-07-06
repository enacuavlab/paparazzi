from trajectories import GVF, GVF_PARAMETRIC
from trajectories.gvf_trajectories import Trajectory, LineTrajectory,SurfaceTrajectory
from pprzlink.generated import telemetry,ground
from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface
import wx
from time import sleep,time

from scipy import linalg as la
from matplotlib.path import Path
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.figure import SubFigure
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import Normalize
import numpy as np
import typing
import inspect, os
import dataclasses



import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(
    path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(
    path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from settings import PprzSettingsManager
from pprz_connect import PprzConnect,PprzConfig


#################### Meta code to import all known trajectories ####################


def __get_GVF_traj_map() -> typing.Dict[int, object]:
    classes = list(filter(lambda t: inspect.isclass(t[1]) 
                          and issubclass(t[1],(LineTrajectory,SurfaceTrajectory))
                          and not inspect.isabstract(t[1]), inspect.getmembers(GVF)))
    output = dict()
    for (n, o) in classes:
        try:
            output[o.class_id()] = o
        except Exception as e:
            print(f"Error with class {n}:\n{o}")
            raise e
    return output


GVF_traj_map = __get_GVF_traj_map()


def __get_GVF_PARAMETRIC_traj_map() -> typing.Dict[int, object]:
    classes = list(filter(lambda t: inspect.isclass(t[1]) 
                          and issubclass(t[1],(LineTrajectory,SurfaceTrajectory))
                          and not inspect.isabstract(t[1]), inspect.getmembers(GVF_PARAMETRIC)))
    output = dict()
    for (n, o) in classes:
        try:
            output[o.class_id()] = o
        except Exception as e:
            print(f"Error with class {n}:\n{o}")
            raise e
    return output


GVF_PARAMETRIC_traj_map = __get_GVF_PARAMETRIC_traj_map()

#################### Storing relevant aircraft data ####################

@dataclasses.dataclass
class Aircraft():
    id:int
    settingManager:PprzSettingsManager
    
    attitude:typing.Optional[telemetry.PprzMessage_ATTITUDE] = None
    navigation:typing.Optional[telemetry.PprzMessage_NAVIGATION] = None
    ground_data:typing.Optional[ground.PprzMessage_FLIGHT_PARAM] = None
    gps:typing.Optional[telemetry.PprzMessage_GPS] = None
    gvf:typing.Optional[telemetry.PprzMessage_GVF] = None
    gvf_parametric:typing.Optional[telemetry.PprzMessage_GVF_PARAMETRIC] = None
    config:typing.Optional[PprzConfig] = None
    last_timestamp:float = 0.
    
    @property
    def XYZ(self) -> np.ndarray:
        if self.gps is None:
            return np.zeros(3)
        
        if self.navigation is None:
            return np.zeros(3)
        
        return np.array([self.navigation.pos_x_,self.navigation.pos_y_, self.gps.alt_ / 1000])
    
    
    @property
    def trajectory(self) -> Trajectory:
        if self.gvf_parametric is None and self.gvf is None:
            return None
        
        if self.gvf_parametric is None:
            return GVF_traj_map[self.gvf.traj_].from_message(self.gvf)
        
        else:
            return GVF_PARAMETRIC_traj_map[self.gvf_parametric.traj_].from_message(self.gvf_parametric)
    
    @property
    def w(self) -> float:
        if self.gvf_parametric is None:
            return None
        else:
            return self.gvf_parametric.w_
    
#################### Drawing frame definition ####################

WIDTH = 800
HEIGHT = 800


class GVFFrame(wx.Frame):
    def __init__(self, ac_id_list:typing.List[int], w_dist:float = 400, 
                 show_field:bool = False, gvf_dist: float = 300, resolution: int = 10,):

        super().__init__(None, id=-1,
                          name=u'GVF', size=wx.Size(WIDTH, HEIGHT),
                          style=wx.DEFAULT_FRAME_STYLE, title=u'Guidance Vector Field')

        # Vehicle variables
        self.ac_ids:typing.List[int] = ac_id_list
        self.ac_dict:typing.Dict[int,Aircraft] = dict()
        
        self.course = 0
        self.pitch:float = 0
        self.yaw:float = 0
        self.XYZ:np.ndarray = np.zeros(3)

        # Desired trajectory
        self.w:float = 0.
        self.gvf_error = 0
        
        # Plotting
        self.map_gvf = Map(150000,w_dist,show_field,gvf_dist,resolution)
        self.show_field:bool = show_field

        # Frame
        self.canvas = FigureCanvas(self, -1, self.map_gvf.fig)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnRedrawTimer, self.redraw_timer)
        self.redraw_timer.Start(100)

        # Ivy
        self._ivy_interface = IvyMessagesInterface("GVF")
        self._pprz_connect = PprzConnect(ivy=self._ivy_interface)
        
        self.on_start()
        
            
    def on_start(self) -> None:
        """
        Gather aircrafts' config data and setup associated callbacks
        """
        for i in self.ac_ids:
            success = False
            while not success:
                try:
                    self._pprz_connect.get_config(i)
                    conf_msg:PprzConfig = self._pprz_connect.conf_by_id(i)
                    success = True
                except KeyError:
                    print(f"Waiting for aircraft with id {i} ...")
                    sleep(1.)
                
            settings_path = os.path.normpath(conf_msg.settings)
            self.ac_dict[i] = Aircraft(i,PprzSettingsManager(settings_path,i,self._ivy_interface))
            self.ac_dict[i].config = conf_msg
            
        
        # bind to ATTITUDE message
        def attitude_cb(ac_id, msg:telemetry.PprzMessage_ATTITUDE):
            if ac_id in self.ac_ids:# and msg.name == "ATTITUDE":
                ac = self.ac_dict[ac_id]
                ac.attitude = msg
                ac.last_timestamp = time()
        self._ivy_interface.subscribe(attitude_cb, telemetry.PprzMessage_ATTITUDE())
            
        # bind to NAVIGATION message
        def nav_cb(ac_id, msg:telemetry.PprzMessage_NAVIGATION):
            if ac_id in self.ac_ids:# and msg.name == "NAVIGATION":
                ac = self.ac_dict[ac_id]
                ac.navigation = msg
                ac.last_timestamp = time()
        self._ivy_interface.subscribe(nav_cb, telemetry.PprzMessage_NAVIGATION())

        # bind to GPS message
        def gps_cb(ac_id, msg:telemetry.PprzMessage_GPS):
            if ac_id in self.ac_ids:# and msg.name == "GPS":
                ac = self.ac_dict[ac_id]
                ac.gps = msg
                ac.last_timestamp = time()
        self._ivy_interface.subscribe(gps_cb,telemetry.PprzMessage_GPS())

        # bind to GVF message
        def gvf_cb(ac_id, msg:telemetry.PprzMessage_GVF):
            if ac_id in self.ac_ids:# and msg.name == "GVF":
                ac = self.ac_dict[ac_id]
                ac.gvf = msg
                ac.last_timestamp = time()
                ac.gvf_parametric = None
        self._ivy_interface.subscribe(gvf_cb, telemetry.PprzMessage_GVF())
        
        # bind to GVF_PARAMETRIC message
        def gvf_par_cb(ac_id, msg:telemetry.PprzMessage_GVF_PARAMETRIC):
            if ac_id in self.ac_ids:# and msg.name == "GVF_PARAMETRIC":
                ac = self.ac_dict[ac_id]
                ac.gvf_parametric = msg
                ac.last_timestamp = time()
                ac.gvf = None
        self._ivy_interface.subscribe(gvf_par_cb, telemetry.PprzMessage_GVF_PARAMETRIC())
                

    def draw_gvf(self):
        self.map_gvf.clear_axes()
        lines = []
        labels = []
        for id,ac in self.ac_dict.items():
            try:
                lines.append(self.map_gvf.draw(ac.XYZ, ac.attitude.psi_, ac.gps.course_, ac.trajectory,
                                  ac.w, ac.config.ac_name, ac.config.color))
                labels.append(ac.config.ac_name)
            except AttributeError as e:
                print(f"Attribute issue with aircraft {id}:\n{e}")
            #print(f"AC n° {id} : {ac.XYZ}")
        self.map_gvf.fig.legend(lines,labels,loc='lower center', ncols=len(labels))

    def OnClose(self, event):
        self._ivy_interface.shutdown()
        self.Destroy()
        

    def OnRedrawTimer(self, event):
        self.draw_gvf()
        self.canvas.draw()


########## Map classes (for drawings) ##########

class Map():
    def __init__(self, area, w_dist:float = 400, show_field:bool = False, gvf_dist: float = 300, resolution: int = 10):
        self.area = area
        self.show_field:bool = show_field
        self.gvf_dist: float = gvf_dist # L1 range from the aircraft when computing GVF
        self.w_dist:float = w_dist # Abstract range from the virtual parameter when computing trajectory (does not affect closed trajectories)
        self.resolution: int = resolution # Number of points used per dimension when computing GVF (10 times more are used for trajectory)
        
        # Setup the figure
        self.fig = plt.figure()
        self.fig.clear()
        self.a3d:Axes3D = self.fig.add_subplot(2,2,1,projection='3d')
        self.axy:Axes3D = self.fig.add_subplot(2,2,2)
        self.axz:Axes3D = self.fig.add_subplot(2,2,3)
        self.ayz:Axes3D = self.fig.add_subplot(2,2,4)
        
        self.a3d.set_title('3D Map')
        self.axy.set_title('XY Map')
        self.axz.set_title('XZ Map')
        self.ayz.set_title('YZ Map')
            
    def vehicle_patch(self, XY, yaw, color:str='red'):
        Rot = np.array([[np.cos(yaw), np.sin(yaw)],
                       [-np.sin(yaw), np.cos(yaw)]])

        apex = 45*np.pi/180  # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        a = b*np.sin(apex/2)
        h = b*np.cos(apex/2)

        z1 = np.array([a/2, -h*0.3])
        z2 = np.array([-a/2, -h*0.3])
        z3 = np.array([0, h*0.6])

        z1 = Rot.dot(z1)
        z2 = Rot.dot(z2)
        z3 = Rot.dot(z3)

        verts = [(XY[0]+z1[0], XY[1]+z1[1]),
                 (XY[0]+z2[0], XY[1]+z2[1]),
                 (XY[0]+z3[0], XY[1]+z3[1]),
                 (0, 0)]

        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
        path = Path(verts, codes)

        return patches.PathPatch(path, facecolor=color, lw=2)
        
    def draw(self, XYZ, yaw, course, traj: LineTrajectory, w:float, name:str, color:str='green'):
        a3d:Axes3D = self.a3d
        axy:Axes3D = self.axy
        axz:Axes3D = self.axz
        ayz:Axes3D = self.ayz
        
        if self.show_field and traj is not None:
            XYZ_meshgrid = np.asarray(np.meshgrid(np.linspace(XYZ[0]-self.gvf_dist, XYZ[0]+self.gvf_dist, self.resolution),
                                      np.linspace(XYZ[1]-self.gvf_dist, XYZ[1]+self.gvf_dist, self.resolution),
                                      np.linspace(XYZ[2]-self.gvf_dist, XYZ[2]+self.gvf_dist, self.resolution),indexing='ij'))
            
            #print(traj.rot.as_matrix())

            vector_field = traj.calc_field(XYZ_meshgrid.transpose(), w).transpose()
        # 3D
        
        if traj is not None:
            if w is None:
                traj_points = traj.calc_traj(np.linspace(-self.w_dist,self.w_dist,self.resolution*10))
            else:
                traj_points = traj.calc_traj(np.linspace(w-self.w_dist,w+self.w_dist,self.resolution*10))
            
            a3d.plot(
                traj_points[:,0], traj_points[:,1], traj_points[:,2], color=color,
                label=f'{name} Trajectory')
            
            if w is not None:
                virtual_point = traj.param_point(w)
            
                a3d.plot(virtual_point[0], virtual_point[1], virtual_point[2], marker='P',
                        color=color,label=f'{name} Virtual carrot')
        else:
            print(f"No GVF trajectory for aircraft {name}")

        a3d.plot(XYZ[0], XYZ[1], XYZ[2], marker='o',
                    color=color, label='{name}')
        
        if self.show_field and traj is not None:
            a3d.quiver3D(XYZ_meshgrid[0], XYZ_meshgrid[1], XYZ_meshgrid[2],
                        vector_field[0], vector_field[1],vector_field[2],
                        color='Teal', pivot='middle', alpha=1, normalize=True)

        a3d.axis('equal')

        # XY
        if traj is not None:
            axy.plot(traj_points[:,0], traj_points[:,1],color=color,
                label=f'{name} Trajectory')
            
            if w is not None:
                axy.plot(virtual_point[0], virtual_point[1], marker='P',
                        color=color,label=f'{name} Virtual carrot')
        
        #axy.add_patch(self.vehicle_patch(XYZ, yaw,color))  # In radians
        axy.plot(XYZ[0], XYZ[1], 'o',color=color, label=f'{name}')
        #apex = 45*np.pi/180  # 30 degrees apex angle
        #b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        #h = b*np.cos(apex/2)
        #axy.arrow(XYZ[0], XYZ[1],
        #          h*np.sin(course), h*np.cos(course),
        #          head_width=5, head_length=10, fc='k', ec='k')
        #axy.annotate('HOME', xy=(0, 0))
        #axy.annotate(traj.name, xy=(traj.x_offset, traj.y_offset))
        # axy.plot(0, 0, 'kx', ms=10, mew=2)
        # axy.plot(traj.x_offset, traj.y_offset, 'kx', ms=10, mew=2)
        
        if self.show_field and traj is not None:
            z_angles_field = np.rad2deg(np.arctan2(vector_field[2][:,:,self.resolution//2],np.sqrt(np.square(vector_field[0][:,:,self.resolution//2])+np.square(vector_field[1][:,:,self.resolution//2]))))
            axy_quiver = axy.quiver(XYZ_meshgrid[0][:,:,self.resolution//2], XYZ_meshgrid[1][:,:,self.resolution//2],
                    vector_field[0][:,:,self.resolution//2], vector_field[1][:,:,self.resolution//2], z_angles_field,
                    color='Teal', pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                    linewidth=0.2)
            self.fig.colorbar(axy_quiver,label="Orthogonal inclination (in degrees, positive is \u2299)")
        
        axy.axis('equal')
        
        # XZ
        if traj is not None:
            axz.plot(traj_points[:, 0], traj_points[:, 2], color=color,label='Trajectory')
            if w is not None:
                axz.plot(virtual_point[0], virtual_point[2], marker='P',
                        color=color,label=f'{name} Virtual carrot')
        axz.plot(XYZ[0], XYZ[2], 'o',color=color, label=f'{name}')
        
        if self.show_field and traj is not None:
            y_angles_field = np.rad2deg(np.arctan2(vector_field[1][:,self.resolution//2,:],np.sqrt(np.square(vector_field[0][:,self.resolution//2,:])+np.square(vector_field[2][:,self.resolution//2,:]))))
            axz_quiver = axz.quiver(XYZ_meshgrid[0][:,self.resolution//2,:], XYZ_meshgrid[2][:,self.resolution//2,:],
                        vector_field[0][:,self.resolution//2,:], vector_field[2][:,self.resolution//2,:], y_angles_field,
                        color='Teal', pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                        linewidth=0.2)
            self.fig.colorbar(axz_quiver)
            
        # YZ
        if traj is not None:
            ayz.plot(traj_points[:, 1], traj_points[:, 2], color=color,label='Trajectory')
            if w is not None:
                ayz.plot(virtual_point[1], virtual_point[2], marker='P',
                        color=color,label=f'{name} Virtual carrot')
            
        if self.show_field and traj is not None:
            x_angles_field = np.rad2deg(np.arctan2(vector_field[0][self.resolution//2,:,:],np.sqrt(np.square(vector_field[1][self.resolution//2,:,:])+np.square(vector_field[2][self.resolution//2,:,:]))))
            ayz_quiver = ayz.quiver(XYZ_meshgrid[1][self.resolution//2,:,:], XYZ_meshgrid[2][self.resolution//2,:,:],
                        vector_field[1][self.resolution//2,:,:], vector_field[2][self.resolution//2,:,:], x_angles_field,
                        color='Teal', pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                        linewidth=0.2)
            self.fig.colorbar(ayz_quiver)
        
        last_line, = ayz.plot(XYZ[1], XYZ[2], 'o',color=color, label=f'{name}')
        return last_line
        
