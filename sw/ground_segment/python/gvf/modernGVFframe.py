from trajectories import GVF, GVF_PARAMETRIC
from trajectories.gvf_trajectories import Trajectory, LineTrajectory,SurfaceTrajectory
from pprzlink.generated.telemetry import PprzMessage_GVF, PprzMessage_GVF_PARAMETRIC
from pprzlink.message import PprzMessage
from pprzlink.ivy import IvyMessagesInterface
import wx
import time

from scipy import linalg as la
from matplotlib.path import Path
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import Normalize
import numpy as np
import typing
import inspect
from abc import ABC,abstractmethod

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(
    path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(
    path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_SRC + "/sw/lib/python")


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


#################### Drawing frame definition ####################

WIDTH = 800
HEIGHT = 800


class GVFFrame(wx.Frame):
    def __init__(self, ac_id, dim:int = 3, w_dist:float = 400, gvf_dist: float = 300, resolution: int = 10):

        wx.Frame.__init__(self, id=-1, parent=None,
                          name=u'GVF', size=wx.Size(WIDTH, HEIGHT),
                          style=wx.DEFAULT_FRAME_STYLE, title=u'Guidance Vector Field')

        # Vehicle variables
        self.ac_id:int = ac_id
        self.course = 0
        self.pitch:float = 0
        self.yaw:float = 0
        self.XYZ:np.ndarray = np.zeros(3)
        self.ground_altitude = -1

        # Desired trajectory
        self.s = 0
        self.ke = 0
        self.w:float = 0.
        self.gvf_error = 0
        
        self.traj: typing.Optional[Trajectory] = None
        
        # Plotting
        self.dim = dim
        self.map_gvf = Map(150000,dim,w_dist,gvf_dist,resolution)

        # Frame
        self.canvas = FigureCanvas(self, -1, self.map_gvf.fig)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnRedrawTimer, self.redraw_timer)
        self.redraw_timer.Start(100)

        # Ivy
        self.interface = IvyMessagesInterface("GVF")
        self.interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg: PprzMessage):
        try:
            ac_id = int(ac_id)
        except ValueError:
            return
        
        if ac_id == self.ac_id:
            if msg.name == 'GPS':
                self.course = int(msg.get_field(3))*np.pi/1800
                self.XYZ[2] = float(msg.get_field(4))/1000
            if msg.name == 'NAVIGATION':
                self.XYZ[0] = float(msg.get_field(2))
                self.XYZ[1] = float(msg.get_field(3))
            if msg.name == 'NAVIGATION_REF':
                self.ground_altitude = float(msg.get_field(3))
            if msg.name == 'ATTITUDE':
                self.pitch = float(msg.get_field(0))
                self.yaw = float(msg.get_field(1))
            if msg.name == 'GVF':
                self.traj = GVF_traj_map[int(msg.get_field(1))].from_message(msg)
                self.w = 0

            if msg.name == 'GVF_PARAMETRIC':
                self.traj = GVF_PARAMETRIC_traj_map[int(msg.get_field(0))].from_message(msg)
                self.w = float(msg.get_field(2))
                

    def draw_gvf(self, XYZ, yaw, course):
        if self.traj is not None:
            self.map_gvf.draw(XYZ, yaw, course, self.traj, self.w)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

    def OnRedrawTimer(self, event):
        self.draw_gvf(self.XYZ, self.yaw, self.course)
        self.canvas.draw()


########## Map classes (for drawings) ##########
class Map():
    def __init__(self, area, dim:int, w_dist:float = 400, gvf_dist: float = 300, resolution: int = 10):
        self.area = area
        self.dim = dim # Prefered dimension for plotting (if 2, project 3D trajectories on XY plane; if 3, nothing special for 2D)
        self.gvf_dist: float = gvf_dist # L1 range from the aircraft when computing GVF
        self.w_dist:float = w_dist # Abstract range from the virtual parameter when computing trajectory (does not affect closed trajectories)
        self.resolution: int = resolution # Number of points used per dimension when computing GVF (10 times more are used for trajectory)
        self.fig = plt.figure()
        
    def vehicle_patch(self, XY, yaw):
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

        return patches.PathPatch(path, facecolor='red', lw=2)
        
    def draw(self, XYZ, yaw, course, traj: Trajectory, w:float):
        if self.dim == 3:
            if traj.dim == 3:
                self.draw_3D(XYZ,yaw,course,traj,w)
            else:
                self.draw_2D(XYZ,yaw,course,traj,w)
        else:
            self.draw_2D(XYZ,yaw,course,traj,w)
    
    def draw_2D(self, XYZ, yaw, course, traj: LineTrajectory, w: float):
        self.fig.clf()

        ax = self.fig.add_subplot(111)

        traj_points = traj.calc_traj(np.linspace(w-self.w_dist,w+self.w_dist,self.resolution*10))
        ax.plot(traj_points[:,0],traj_points[:,1], color='Green')

        XYZ_meshgrid = np.asarray(np.meshgrid(np.linspace(XYZ[0]-self.gvf_dist, XYZ[0]+self.gvf_dist, self.resolution),
                                  np.linspace(XYZ[1]-self.gvf_dist, XYZ[1]+self.gvf_dist, self.resolution),
                                  np.zeros(self.resolution),indexing='ij'))

        vector_field = traj.calc_field(XYZ_meshgrid.transpose(), w).transpose()

        if traj.dim == 2:
            ax.quiver(XYZ_meshgrid[0][:,:,self.resolution//2], XYZ_meshgrid[1][:,:,self.resolution//2],
                    vector_field[0][:,:,self.resolution//2], vector_field[1][:,:,self.resolution//2], color='Teal',
                    pivot='mid', width=0.002, edgecolor='black')
        else: #traj.dim == 3
            angles_field = np.rad2deg(np.arctan2(vector_field[2][:,:,self.resolution//2],np.sqrt(np.square(vector_field[0][:,:,self.resolution//2])+np.square(vector_field[1][:,:,self.resolution//2]))))
            
            quiver_plot = ax.quiver(XYZ_meshgrid[0][:,:,self.resolution//2], XYZ_meshgrid[1][:,:,self.resolution//2],
                    vector_field[0][:,:,self.resolution//2], vector_field[1][:,:,self.resolution//2], angles_field,
                    pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                    linewidth=0.1)
            self.fig.colorbar(quiver_plot)
            
        ax.add_patch(self.vehicle_patch(XYZ, yaw))  # In radians
        apex = 45*np.pi/180  # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        h = b*np.cos(apex/2)
        ax.arrow(XYZ[0], XYZ[1],
                 h*np.sin(course), h*np.cos(course),
                 head_width=5, head_length=10, fc='k', ec='k')
        ax.annotate('HOME', xy=(0, 0))
        ax.annotate(traj.name, xy=(traj.x_offset, traj.y_offset))
        ax.plot(0, 0, 'kx', ms=10, mew=2)
        ax.plot(traj.x_offset, traj.y_offset, 'kx', ms=10, mew=2)

        # elif isinstance(traj, traj_param_trefoil_2D):
        #     ax.annotate('TREFOIL', xy = (traj.XYoff[0], traj.XYoff[1]))
        #     ax.plot(0, 0, 'kx', ms=10, mew=2)
        #     ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
        #     ax.plot(traj.wpoint[0], traj.wpoint[1], 'rx', ms=10, mew=2)

        ax.set_xlabel('South [m]')
        ax.set_ylabel('West [m]')
        ax.set_title('2D Map')
        ax.set_xlim(XYZ[0]-self.gvf_dist,
                    XYZ[0]+self.gvf_dist)
        ax.set_ylim(XYZ[1]-self.gvf_dist,
                    XYZ[1]+self.gvf_dist)
        ax.axis('equal')
        ax.grid()

    def draw_3D(self, XYZ, yaw, course, traj: Trajectory, w:float):
        self.fig.clf()
        assert traj.dim == 3
        """
        if isinstance(traj, traj_param_surf_torus_3D):
            a3d = self.fig.add_subplot(1, 1, 1, projection='3d')
            angletor = np.linspace(0, 2 * np.pi, 32)
            thetator, phitor = np.meshgrid(angletor, angletor)
            xtor = (traj.rh + traj.rv * np.cos(phitor)) * \
                np.cos(thetator) + traj.XYoff[0]
            ytor = (traj.rh + traj.rv * np.cos(phitor)) * \
                np.sin(thetator) + traj.XYoff[1]
            ztor = traj.rv * np.sin(phitor) + traj.zo
            a3d.plot_surface(xtor, ytor, ztor, color='w',
                             rstride=1, cstride=1, alpha=0.5)

            a3d.plot([XY[0]], [XY[1]], [altitude], marker='o',
                     markerfacecolor='r', markeredgecolor='r')
            a3d.plot([traj.wpoint[0]], [traj.wpoint[1]], [traj.wpoint[2]],
                     marker='x', markerfacecolor='r', markeredgecolor='r')

            a3d.axis('equal')
            if traj.deltaz < 0:
                a3d.set_zlim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                a3d.set_zlim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)
        """
        a3d:Axes3D = self.fig.add_subplot(2, 2, 1, projection='3d')
        axy:Axes3D = self.fig.add_subplot(2, 2, 2)
        axz:Axes3D = self.fig.add_subplot(2, 2, 3)
        ayz:Axes3D = self.fig.add_subplot(2, 2, 4)

        a3d.set_title('3D Map')
        axy.set_title('XY Map')
        axz.set_title('XZ Map')
        ayz.set_title('YZ Map')

        XYZ_meshgrid = np.asarray(np.meshgrid(np.linspace(XYZ[0]-self.gvf_dist, XYZ[0]+self.gvf_dist, self.resolution),
                                  np.linspace(XYZ[1]-self.gvf_dist, XYZ[1]+self.gvf_dist, self.resolution),
                                  np.linspace(XYZ[2]-self.gvf_dist, XYZ[2]+self.gvf_dist, self.resolution),indexing='ij'))

        vector_field = traj.calc_field(XYZ_meshgrid.transpose(), w).transpose()
        # 3D
        
        traj_points = traj.calc_traj(np.linspace(w-self.w_dist,w+self.w_dist,self.resolution*10))
        
        virtual_point = traj.param_point(w)
        
        a3d.plot(
            traj_points[:,0], traj_points[:,1], traj_points[:,2], color='green',
            label='Trajectory')
        
        a3d.plot(virtual_point[0], virtual_point[1], virtual_point[2], marker='P',
                 color='r',label='Virtual carrot')

        a3d.plot(XYZ[0], XYZ[1], XYZ[2], marker='o',
                    color='r', label='Aircraft')
        
        a3d.quiver3D(XYZ_meshgrid[0], XYZ_meshgrid[1], XYZ_meshgrid[2],
                    vector_field[0], vector_field[1],vector_field[2],
                    color='Teal', pivot='middle', alpha=1, normalize=True)

        a3d.axis('equal')

        # XY
        z_angles_field = np.rad2deg(np.arctan2(vector_field[2][:,:,self.resolution//2],np.sqrt(np.square(vector_field[0][:,:,self.resolution//2])+np.square(vector_field[1][:,:,self.resolution//2]))))

        axy.plot(traj_points[:,0], traj_points[:,1],color='green',
            label='Trajectory')
        axy.add_patch(self.vehicle_patch(XYZ, yaw))  # In radians
        apex = 45*np.pi/180  # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        h = b*np.cos(apex/2)
        axy.arrow(XYZ[0], XYZ[1],
                  h*np.sin(course), h*np.cos(course),
                  head_width=5, head_length=10, fc='k', ec='k')
        axy.annotate('HOME', xy=(0, 0))
        axy.annotate(traj.name, xy=(traj.x_offset, traj.y_offset))
        axy.plot(0, 0, 'kx', ms=10, mew=2)
        axy.plot(traj.x_offset, traj.y_offset, 'kx', ms=10, mew=2)
        axy.plot(virtual_point[0], virtual_point[1], marker='P',
                 color='r',label='Virtual carrot')
        
        axy_quiver = axy.quiver(XYZ_meshgrid[0][:,:,self.resolution//2], XYZ_meshgrid[1][:,:,self.resolution//2],
                    vector_field[0][:,:,self.resolution//2], vector_field[1][:,:,self.resolution//2], z_angles_field,
                    color='Teal', pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                    linewidth=0.2)
        self.fig.colorbar(axy_quiver)
        axy.axis('equal')
        
        # XZ
        y_angles_field = np.rad2deg(np.arctan2(vector_field[1][:,self.resolution//2,:],np.sqrt(np.square(vector_field[0][:,self.resolution//2,:])+np.square(vector_field[2][:,self.resolution//2,:]))))
        
        axz.plot(traj_points[:, 0], traj_points[:, 2], color='green',label='Trajectory')
        axz.plot(XYZ[0], XYZ[2], 'ro', label='Aircraft')
        axz.plot(virtual_point[0], virtual_point[2], marker='P',
                 color='r',label='Virtual carrot')
        axz_quiver = axz.quiver(XYZ_meshgrid[0][:,self.resolution//2,:], XYZ_meshgrid[2][:,self.resolution//2,:],
                    vector_field[0][:,self.resolution//2,:], vector_field[2][:,self.resolution//2,:], y_angles_field,
                    color='Teal', pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                    linewidth=0.2)
        self.fig.colorbar(axz_quiver)
            
        # YZ
        x_angles_field = np.rad2deg(np.arctan2(vector_field[0][self.resolution//2,:,:],np.sqrt(np.square(vector_field[1][self.resolution//2,:,:])+np.square(vector_field[2][self.resolution//2,:,:]))))
        
        ayz.plot(traj_points[:, 1], traj_points[:, 2], color='green',label='Trajectory')
        ayz.plot(XYZ[1], XYZ[2], 'ro', label='Aircraft')
        ayz.plot(virtual_point[1], virtual_point[2], marker='P',
                 color='r',label='Virtual carrot')
        ayz_quiver = ayz.quiver(XYZ_meshgrid[1][self.resolution//2,:,:], XYZ_meshgrid[2][self.resolution//2,:,:],
                    vector_field[1][self.resolution//2,:,:], vector_field[2][self.resolution//2,:,:], x_angles_field,
                    color='Teal', pivot='mid', width=0.005, edgecolor='Black', cmap='seismic', norm=Normalize(-90,90),
                    linewidth=0.2)
        self.fig.colorbar(ayz_quiver)
