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
import numpy as np
import typing
import inspect

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
                          and t[1] not in (LineTrajectory,SurfaceTrajectory), inspect.getmembers(GVF)))
    output = dict()
    for (_, o) in classes:
        output[o.class_id()] = o
    return output
    return output


GVF_traj_map = __get_GVF_traj_map()


def __get_GVF_PARAMETRIC_traj_map() -> typing.Dict[int, object]:
    classes = list(filter(lambda t: inspect.isclass(t[1]) 
                          and issubclass(t[1],(LineTrajectory,SurfaceTrajectory))
                          and t[1] not in (LineTrajectory,SurfaceTrajectory), inspect.getmembers(GVF_PARAMETRIC)))
    output = dict()
    for (_, o) in classes:
        output[o.class_id()] = o
    return output


GVF_PARAMETRIC_traj_map = __get_GVF_PARAMETRIC_traj_map()


#################### Drawing frame definition ####################

WIDTH = 800
HEIGHT = 800


class GVFFrame(wx.Frame):
    def __init__(self, ac_id):

        wx.Frame.__init__(self, id=-1, parent=None,
                          name=u'GVF', size=wx.Size(WIDTH, HEIGHT),
                          style=wx.DEFAULT_FRAME_STYLE, title=u'Guidance Vector Field')

        # Vehicle variables
        self.ac_id = ac_id
        self.course = 0
        self.yaw = 0
        self.XY = np.array([0, 0])
        self.altitude = 0
        self.ground_altitude = -1

        # Desired trajectory
        self.s = 0
        self.ke = 0
        self.w:float = 0.
        self.gvf_error = 0
        self.map_gvf = Map2D(np.array([0, 0]), 150000)
        self.traj: typing.Optional[Trajectory] = None

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
        if int(ac_id) == self.ac_id:
            if msg.name == 'GPS':
                self.course = int(msg.get_field(3))*np.pi/1800
                self.altitude = float(msg.get_field(4))/1000
            if msg.name == 'NAVIGATION':
                self.XY[0] = float(msg.get_field(2))
                self.XY[1] = float(msg.get_field(3))
            if msg.name == 'NAVIGATION_REF':
                self.ground_altitude = float(msg.get_field(3))
            if msg.name == 'ATTITUDE':
                self.yaw = float(msg.get_field(1))
            if msg.name == 'GVF':
                self.traj = GVF_traj_map[int(msg.get_field(1))].from_message(msg)
                self.w = 0

            if msg.name == 'GVF_PARAMETRIC':
                self.traj = GVF_PARAMETRIC_traj_map[int(msg.get_field(1))].from_message(msg)
                self.w = msg.get_field(3)

    def draw_gvf(self, XY, yaw, course, altitude):
        if self.traj is not None:
            self.map_gvf.draw(XY, yaw, course, altitude, self.traj, self.w)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

    def OnRedrawTimer(self, event):
        self.draw_gvf(self.XY, self.yaw, self.course,
                      self.altitude-self.ground_altitude)
        self.canvas.draw()


class Map2D:
    def __init__(self, XYoff, area, ac_dist: float = 300, resolution: int = 10):
        self.XYoff = XYoff
        self.area = area
        self.ac_dist: float = ac_dist
        self.resolution: int = resolution
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

    def draw(self, XY, yaw, course, altitude, traj: LineTrajectory, w: float):
        self.fig.clf()
        assert traj.dim == 2

        ax = self.fig.add_subplot(111)

        traj_points = traj.calc_traj(np.linspace(w-10,w+10,self.resolution*10))
        ax.plot(traj_points[:,0],traj_points[:,1], color='Green')

        XYZ_meshgrid = np.asarray(np.meshgrid(np.linspace(XY[0]-self.ac_dist, XY[0]+self.ac_dist, self.resolution),
                                  np.linspace(XY[1]-self.ac_dist, XY[1]+self.ac_dist, self.resolution),
                                  np.zeros(self.resolution),indexing='ij'))

        vector_field = traj.calc_field(XYZ_meshgrid.transpose(), w).transpose()

        ax.quiver(XYZ_meshgrid[0][:,:,0], XYZ_meshgrid[1][:,:,0],
                  vector_field[0][:,:,0], vector_field[1][:,:,0], color='Teal',
                  pivot='mid', width=0.002)
        ax.add_patch(self.vehicle_patch(XY, yaw))  # In radians
        apex = 45*np.pi/180  # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        h = b*np.cos(apex/2)
        ax.arrow(XY[0], XY[1],
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
        ax.set_xlim(XY[0]-self.ac_dist,
                    XY[0]+self.ac_dist)
        ax.set_ylim(XY[1]-self.ac_dist,
                    XY[1]+self.ac_dist)
        ax.axis('equal')
        ax.grid()

"""
class Map3D:
    def __init__(self, XYoff, area):
        self.XYoff = XYoff
        self.area = area
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

    def draw(self, XY, yaw, course, altitude, traj: Trajectory):
        self.fig.clf()
        assert traj.dim == 3
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

        else:
            a3d = self.fig.add_subplot(2, 2, 1, projection='3d')
            axy = self.fig.add_subplot(2, 2, 2)
            axz = self.fig.add_subplot(2, 2, 3)
            ayz = self.fig.add_subplot(2, 2, 4)

            a3d.set_title('3D Map')
            axy.set_title('XY Map')
            axz.set_title('XZ Map')
            ayz.set_title('YZ Map')

            # 3D
            a3d.plot(
                traj.traj_points[0, :], traj.traj_points[1, :], traj.traj_points[2, :])

            if altitude != -1:
                a3d.plot([XY[0]], [XY[1]], [altitude], marker='o',
                         markerfacecolor='r', markeredgecolor='r')
                a3d.plot([traj.wpoint[0]], [traj.wpoint[1]], [
                         traj.wpoint[2]], marker='x', markerfacecolor='r', markeredgecolor='r')

                a3d.axis('equal')
            if traj.deltaz < 0:
                a3d.set_zlim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                a3d.set_zlim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)

            # XY
            axy.plot(traj.traj_points[0, :], traj.traj_points[1, :])
            axy.add_patch(self.vehicle_patch(XY, yaw))  # In radians
            apex = 45*np.pi/180  # 30 degrees apex angle
            b = np.sqrt(2*(self.area/2000) / np.sin(apex))
            h = b*np.cos(apex/2)
            axy.arrow(XY[0], XY[1],
                      h*np.sin(course), h*np.cos(course),
                      head_width=5, head_length=10, fc='k', ec='k')
            axy.annotate('HOME', xy=(0, 0))
            axy.plot(traj.wpoint[0], traj.wpoint[1], 'rx', ms=10, mew=2)
            axy.annotate(traj.name, xy=(traj.x_offset, traj.y_offset))
            axy.plot(0, 0, 'kx', ms=10, mew=2)
            axy.plot(traj.x_offset, traj.y_offset, 'kx', ms=10, mew=2)
            axy.axis('equal')

            # XZ
            axz.plot(traj.traj_points[0, :], traj.traj_points[2, :])
            if altitude != -1:
                axz.plot([XY[0]], [altitude], 'ro')
                axz.plot(traj.wpoint[0], traj.wpoint[2], 'rx', ms=10, mew=2)
            if traj.deltaz < 0:
                axz.set_ylim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                axz.set_ylim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)
            # YZ
            ayz.plot(traj.traj_points[1, :], traj.traj_points[2, :])
            if altitude != -1:
                ayz.plot([XY[1]], [altitude], 'ro')
                ayz.plot(traj.wpoint[1], traj.wpoint[2], 'rx', ms=10, mew=2)
            if traj.deltaz < 0:
                axz.set_ylim(traj.zo+1.5*traj.deltaz, traj.zo-1.5*traj.deltaz)
            else:
                ayz.set_ylim(traj.zo-1.5*traj.deltaz, traj.zo+1.5*traj.deltaz)
"""