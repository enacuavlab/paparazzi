import wx
import time

from scipy import linalg as la
from matplotlib.path import Path
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
import matplotlib.pyplot as pl
import matplotlib.patches as patches
import numpy as np

import sys
from os import path, getenv
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 
from settings_xml_parse import PaparazziACSettings

WIDTH = 800
HEIGHT = 800

class GVFFrame(wx.Frame):
    def __init__(self, ac_id=3):

        wx.Frame.__init__(self, id=-1, parent=None, \
                name=u'MessagesFrame', size=wx.Size(WIDTH, HEIGHT), \
                style=wx.DEFAULT_FRAME_STYLE, title=u'Messages')

        # Vehicle variables
        self.ac_id = ac_id
        self.course = 0
        self.yaw = 0
        self.XY = np.array([0, 0])

        # Desired trajectory
        self.timer_traj = 0 # We do not update the traj every time we receive a msg
        self.timer_traj_lim = 7 # (7+1) * 0.25secs
        self.kn = 0
        self.ke = 0
        self.map_gvf = map2d(np.array([0, 0]), 150000)
        self.traj = None

        # Frame
        self.canvas = FigureCanvas(self, -1, self.map_gvf.fig)
        self.Bind(wx.EVT_CLOSE, self.OnClose)
       
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnRedrawTimer, self.redraw_timer)        
        self.redraw_timer.Start(100)
        
        # Ivy
        self.interface = IvyMessagesInterface("GVF")
        self.interface.subscribe(self.message_recv)
        settings = PaparazziACSettings(ac_id)
        self.ke_index = None
        self.kn_index = None
        self.indexes_are_good = 0
        self.list_of_indexes = ['gvf_ke', 'gvf_kn']

        for setting_ in self.list_of_indexes:
            try:
                index = settings.name_lookup[setting_].index
                if setting_ == 'gvf_ke':
                    self.ke_index = index
                if setting_ == 'gvf_kn':
                    self.kn_index = index
                self.indexes_are_good = self.indexes_are_good + 1
            except Exception as e:
                print(e)
                print(setting_ + " setting not found, \
                        have you forgotten gvf.xml in your settings?")

    def message_recv(self, ac_id, msg):
        if msg.name == 'GPS':
            self.course = int(msg.get_field(3))*np.pi/1800
        
        if msg.name == 'NAVIGATION':
            self.XY[0] = float(msg.get_field(2))
            self.XY[1] = float(msg.get_field(3))
        
        if msg.name == 'ATTITUDE':
            self.yaw = float(msg.get_field(1))
        
        if msg.name == 'DL_VALUE' and \
                self.indexes_are_good == len(self.list_of_indexes):
            if int(msg.get_field(0)) == int(self.ke_index):
                self.ke = float(msg.get_field(1))
                if self.traj is not None:
                    self.traj.vector_field(self.traj.XYoff, \
                            self.map_gvf.area, self.kn, self.ke)
            if int(msg.get_field(0)) == int(self.kn_index):
                self.kn = float(msg.get_field(1))
                if self.traj is not None:
                    self.traj.vector_field(self.traj.XYoff, \
                            self.map_gvf.area, self.kn, self.ke)

        if msg.name == 'GVF':
            self.gvf_error = float(msg.get_field(0))
            # Ellipse
            if int(msg.get_field(1)) == 1 \
                    and self.timer_traj == self.timer_traj_lim:
                ex = float(msg.get_field(2))
                ey = float(msg.get_field(3))
                ea = float(msg.get_field(4))
                eb = float(msg.get_field(5))
                ealpha = float(msg.get_field(6))
                self.traj = traj_ellipse(np.array([ex, ey]), ealpha, ea, eb)
                self.traj.vector_field(self.traj.XYoff, \
                        self.map_gvf.area, self.kn, self.ke)

            self.timer_traj = self.timer_traj + 1
            if self.timer_traj > self.timer_traj_lim:
                self.timer_traj = 0

    def draw_gvf(self, XY, yaw, course):
        if self.traj is not None:
            self.map_gvf.draw(XY, yaw, course, self.traj)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

    def OnRedrawTimer(self, event):
        self.draw_gvf(self.XY, self.yaw, self.course)
        self.canvas.draw()

class map2d:
    def __init__(self, XYoff, area):
        self.XYoff = XYoff
        self.area = area
        self.fig, self.ax = pl.subplots()
        self.ax.set_xlabel('South [m]')
        self.ax.set_ylabel('West [m]')
        self.ax.set_title('2D Map')
        self.ax.annotate('HOME', xy = (0, 0))
        self.ax.set_xlim(XYoff[0]-0.5*np.sqrt(area), XYoff[0]+0.5*np.sqrt(area))
        self.ax.set_ylim(XYoff[1]-0.5*np.sqrt(area), XYoff[1]+0.5*np.sqrt(area))
        self.ax.axis('equal')

    def vehicle_patch(self, XY, yaw):
        Rot = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])
    
        apex = 45*np.pi/180 # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        a = b*np.sin(apex/2)
        h = b*np.cos(apex/2)

        z1 = np.array([a/2, -h*0.3])
        z2 = np.array([-a/2, -h*0.3])
        z3 = np.array([0, h*0.6])

        z1 = Rot.dot(z1)
        z2 = Rot.dot(z2)
        z3 = Rot.dot(z3)

        verts = [(XY[0]+z1[0], XY[1]+z1[1]), \
                 (XY[0]+z2[0], XY[1]+z2[1]), \
                 (XY[0]+z3[0], XY[1]+z3[1]), \
                 (0, 0)]
        
        codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
        path = Path(verts, codes)

        return patches.PathPatch(path, facecolor='red', lw=2)

    def draw(self, XY, yaw, course, traj):
        self.ax.clear()
        self.ax.plot(traj.traj_points[0, :], traj.traj_points[1, :])
        self.ax.quiver(traj.mapgrad_X, traj.mapgrad_Y, \
                traj.mapgrad_U, traj.mapgrad_V, color='Teal', \
                pivot='tip', width=0.002)
        self.ax.add_patch(self.vehicle_patch(XY, yaw)) # In radians
        apex = 45*np.pi/180 # 30 degrees apex angle
        b = np.sqrt(2*(self.area/2000) / np.sin(apex))
        h = b*np.cos(apex/2)
        self.ax.arrow(XY[0], XY[1], \
                h*np.sin(course), h*np.cos(course),\
                head_width=5, head_length=10, fc='k', ec='k')
        self.ax.annotate('HOME', xy = (0, 0))
        self.ax.annotate('ELLIPSE', xy = (traj.XYoff[0], traj.XYoff[1]))
        self.ax.plot(0, 0, 'kx', ms=10, mew=2)
        self.ax.plot(traj.XYoff[0], traj.XYoff[1], 'kx', ms=10, mew=2)
        self.ax.set_xlabel('South [m]')
        self.ax.set_ylabel('West [m]')
        self.ax.set_title('2D Map')
        self.ax.set_xlim(self.XYoff[0]-0.5*np.sqrt(self.area), \
                self.XYoff[0]+0.5*np.sqrt(self.area))
        self.ax.set_ylim(traj.XYoff[1]-0.5*np.sqrt(self.area), \
                self.XYoff[1]+0.5*np.sqrt(self.area))
        self.ax.axis('equal')
        self.ax.grid()

class traj_ellipse:
    def float_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def __init__(self, XYoff, rot, a, b):
        self.XYoff = XYoff
        self.a, self.b = a, b
        self.rot = rot
        self.traj_points = np.zeros((2, 200))
        self.mapgrad_X = []
        self.mapgrad_Y = []
        self.mapgrad_U = []
        self.mapgrad_V = []
        
        i = 0
        for t in self.float_range(0, 1, 0.005):
            self.traj_points[:, i] = self.param_point(t)
            i = i + 1

    def param_point(self, t):
        angle = 2*np.pi*t
        return self.XYoff \
                + np.array([self.a*np.cos(angle)*np.cos(-self.rot) - \
                self.b*np.sin(angle)*np.sin(-self.rot), \
                self.a*np.cos(angle)*np.sin(-self.rot) + \
                self.b*np.sin(angle)*np.cos(-self.rot)])

    def vector_field(self, XYoff, area, kn, ke):
        self.mapgrad_X, self.mapgrad_Y = np.mgrid[XYoff[0]-0.5*np.sqrt(area):\
                XYoff[0]+0.5*np.sqrt(area):30j, \
                XYoff[1]-0.5*np.sqrt(area):\
                XYoff[1]+0.5*np.sqrt(area):30j]

        Xel = (self.mapgrad_X-self.XYoff[0])*np.cos(self.rot) \
                - (self.mapgrad_Y-self.XYoff[1])*np.sin(self.rot)

        Yel = (self.mapgrad_X-self.XYoff[0])*np.sin(self.rot) \
                + (self.mapgrad_Y-self.XYoff[1])*np.cos(self.rot)

        nx = 2*Xel*np.cos(self.rot)/self.a**2 \
                + 2*Yel*np.sin(self.rot)/self.b**2
        ny = -2*Xel*np.sin(self.rot)/self.a**2 \
                + 2*Yel*np.cos(self.rot)/self.b**2

        tx = ny
        ty = -nx

        e = (Xel/self.a)**2 + (Yel/self.b)**2 - 1
        
        self.mapgrad_U = tx -ke*e*nx
        self.mapgrad_V = ty -ke*e*ny
        
        norm = np.sqrt(self.mapgrad_U**2 + self.mapgrad_V**2)

        self.mapgrad_U = self.mapgrad_U/norm
        self.mapgrad_V = self.mapgrad_V/norm
