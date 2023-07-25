from __future__ import annotations

from dataclasses import dataclass, field
from abc import ABC, abstractmethod

from .gvf_trajectories import LineTrajectory, SurfaceTrajectory
from pprzlink.message import PprzMessage

import typing
import numpy as np
from scipy.spatial.transform import Rotation

#################### Abstract class specific to parametric trajectories ####################


@dataclass
class ParametricLineTrajectory(LineTrajectory, ABC):
    beta_s:float = 1.
    L:float = 1.
    
    def parse_gvf_config(self, msg:PprzMessage) -> None:
        config_field = msg.get_field(1)
        self.beta_s = float(config_field[2])
        self.L = float(config_field[6])
        
        self._parse_affine_transform(msg)
    
    def _parse_affine_transform(self, msg: PprzMessage) -> None:
        xyzw_quaternion = np.array(msg.get_field(4), dtype=float)
        self.rot = Rotation.from_quat(xyzw_quaternion)
        self.XYZoffset = np.array(msg.get_field(5), dtype=float)

    def gvf(self, pos: np.ndarray, t: float) -> np.ndarray:
        """
        The GVF equation for paramertic line trajectories
        (cf https://doi.org/10.48550/arXiv.2012.01826 ; proof of Theorem 2)
        """
        ft = self.param_point(t)
        phi = self.L*(pos - ft) * self.gain

        dir = 1-2*(self.dim % 2)  # = (-1)^dim
        return self.L*(self.beta_s* self.L* self.L*dir * self.grad_param_point(t) - phi)
    
    def normalized_gvf(self,pos:np.ndarray, t:float) -> np.ndarray:
        ft = self.param_point(t)
        phi = self.L*(pos - ft) * self.gain

        dir = 1-2*(self.dim % 2)  # = (-1)^dim
        fdt = self.grad_param_point(t)
        return self.L*(self.beta_s* self.L* self.L* dir * fdt/np.sqrt(np.sum(np.square(fdt))) - phi)
        

#################### Actually defined parametric trajectories ####################


@dataclass
class Trefoil_2D(ParametricLineTrajectory):
    name:str = "Trefoil 2D"
    w1:float = 1.
    w2:float = 1.
    ratio:float = 0.
    r:float = 1.

    @staticmethod
    def class_id() -> int:
        return 0

    @staticmethod
    def class_dim() -> int:
        return 2
    
    @staticmethod
    def from_message(msg: PprzMessage) -> Trefoil_2D:
        assert msg.name == "GVF_PARAMETRIC" and int(
            msg.get_field(0)) == Trefoil_2D.class_id()
        
        param = [float(x) for x in msg.get_field(2)]
        xo = param[0]
        yo = param[1]
        w1 = param[2]
        w2 = param[3]
        ratio = param[4]
        r = param[5]
        alpha = param[6]
        
        output = Trefoil_2D(w1=w1,w2=w2,ratio=ratio,r=r)
        output.parse_gvf_config(msg)
        output.rot *= Rotation.from_euler('z',alpha,degrees=True)
        output.XYZoffset += np.array([xo,yo,0])
        
        return output
    
    def _param_point(self, t: float) -> np.ndarray:
        xs = np.cos(t*self.w1)*(self.r*np.cos(t*self.w2) + self.ratio)
        ys = np.sin(t*self.w1)*(self.r*np.cos(t*self.w2) + self.ratio)
        zs = 0

        return np.stack([xs, ys, zs])

    def _grad_param_point(self, t: float, step: float = 0.005) -> np.ndarray:
        xs = -self.w1*np.sin(t*self.w1)*(self.r*np.cos(t*self.w2) + self.ratio) - np.cos(t*self.w1)*self.r*self.w2*np.sin(t*self.w2)
        ys =  self.w1*np.cos(t*self.w1)*(self.r*np.cos(t*self.w2) + self.ratio) - np.sin(t*self.w1)*self.r*self.w2*np.sin(t*self.w2)
        zs = 0

        return np.stack([xs, ys, zs])



@dataclass
class Ellipse_3D(ParametricLineTrajectory):
    name: str = "Ellipse 3D"
    r: float = 0  # Radius on the XY projection
    zl: float = 0  # Highest point
    zh: float = 0  # Lowest point
    alpha: float = 0  # Low-high orientation
    # Closed curve: memorize trajectory
    _traj_points: np.ndarray = field(default=None, init=False)

    @staticmethod
    def class_id() -> int:
        return 1

    @staticmethod
    def class_dim() -> int:
        return 3

    @staticmethod
    def from_message(msg: PprzMessage) -> Ellipse_3D:
        assert msg.name == "GVF_PARAMETRIC" and int(
            msg.get_field(0)) == Ellipse_3D.class_id()

        param = [float(x) for x in msg.get_field(2)]
        xo = param[0]
        yo = param[1]
        r = param[2]
        zl = param[3]
        zh = param[4]
        alpha = param[5]
        zm = (zl+zh)/2

        output = Ellipse_3D(r=r, zl=zl-zm, zh=zh-zm, alpha=alpha)
        output.parse_gvf_config(msg)
        output.XYZoffset += np.array([xo, yo, zm])

        return output

    def _param_point(self, t: float) -> np.ndarray:
        xs = np.cos(t)*self.r
        ys = - np.sin(t)*self.r
        zs = (self.zl-self.zh)*np.sin(self.alpha + t)/2

        return np.stack([xs, ys, zs])

    def _grad_param_point(self, t: float, step: float = 0.005) -> np.ndarray:
        xs = -np.sin(t)*self.r
        ys = - np.cos(t)*self.r
        zs = (self.zl-self.zh)*np.cos(self.alpha + t)/2

        return np.stack([xs, ys, zs])

    def calc_traj(self, range: np.ndarray) -> np.ndarray:
        if self._traj_points is None:
            self._traj_points = super().calc_traj(np.linspace(0, 2*np.pi, len(range)))
        elif len(self._traj_points) < len(range):
            self._traj_points = super().calc_traj(np.linspace(0, 2*np.pi, len(range)))
        return self._traj_points


@dataclass
class Lissajous_3D(ParametricLineTrajectory):
    name:str = "Lissajous 3D"
    cx:float = 1.
    cy:float = 1.
    cz:float = 1.
    wx:float = 1.
    wy:float = 1.
    wz:float = 1.
    deltax:float = 0.
    deltay:float = 0.
    deltaz:float = 0.
    
    @staticmethod
    def class_id() -> int:
        return 2

    @staticmethod
    def class_dim() -> int:
        return 3
    
    @staticmethod
    def from_message(msg: PprzMessage) -> Lissajous_3D:
        assert msg.name == "GVF_PARAMETRIC" and int(
            msg.get_field(0)) == Lissajous_3D.class_id()

        param = [float(x) for x in msg.get_field(2)]
        xo = param[0]
        yo = param[1]
        zo = param[2]
        cx = param[3]
        cy = param[4]
        cz = param[5]
        wx = param[6]
        wy = param[7]
        wz = param[8]
        deltax = np.deg2rad(param[9])
        deltay = np.deg2rad(param[10])
        deltaz = np.deg2rad(param[11])
        alpha = param[12]
        
        output = Lissajous_3D(cx=cx,cy=cy,cz=cz,wx=wx,wy=wy,wz=wz,deltax=deltax,deltay=deltay,deltaz=deltaz)
        output.parse_gvf_config(msg)
        output.rot *= Rotation.from_euler('z',alpha,degrees=True)
        output.XYZoffset += np.array([xo,yo,zo])
        
        return output

    def _param_point(self, t: float) -> np.ndarray:
        xs = self.cx*np.cos(self.wx*t + self.deltax)
        ys = self.cy*np.cos(self.wy*t + self.deltay)
        zs = self.cz*np.cos(self.wz*t + self.deltaz)

        return np.stack([xs, ys, zs])

    def _grad_param_point(self, t: float, step: float = 0.005) -> np.ndarray:
        xs = -self.wx*self.cx*np.sin(self.wx*t + self.deltax)
        ys = -self.wy*self.cy*np.sin(self.wy*t + self.deltay)
        zs = -self.wz*self.cz*np.sin(self.wz*t + self.deltaz)

        return np.stack([xs, ys, zs])

@dataclass
class Sinusoid_3D(ParametricLineTrajectory):
    name:str = "Sinusoid 3D"
    ay: float = 1.
    freq_y: float = 1.
    phase_y: float = 0.
    az: float = 1.
    freq_z: float = 1.
    phase_z: float = 0.

    @staticmethod
    def class_id() -> int:
        return 4

    @staticmethod
    def class_dim() -> int:
        return 3

    @staticmethod
    def from_message(msg: PprzMessage) -> Sinusoid_3D:
        assert msg.name == "GVF_PARAMETRIC" and int(
            msg.get_field(0)) == Sinusoid_3D.class_id()

        param = [float(x) for x in msg.get_field(2)]
        ay = param[0]
        freq_y = param[1]
        phase_y = param[2]
        az = param[3]
        freq_z = param[4]
        phase_z = param[5]

        output = Sinusoid_3D(ay=ay, freq_y=freq_y, phase_y=phase_y, az=az,
                             freq_z=freq_z, phase_z=phase_z)
        output.parse_gvf_config(msg)
        return output

    def _param_point(self, t: float) -> np.ndarray:
        xs = t
        ys = np.sin(2*np.pi*self.freq_y*t + self.phase_y)*self.ay
        zs = np.sin(2*np.pi*self.freq_z*t + self.phase_z)*self.az

        return np.stack([xs, ys, zs])

    def _grad_param_point(self, t: float, step: float = 0.005) -> np.ndarray:
        xs = 1
        ys = np.cos(2*np.pi*self.freq_y*t+ self.phase_y)*self.ay * 2*np.pi*self.freq_y
        zs = np.cos(2*np.pi*self.freq_z*t+ self.phase_z)*self.az * 2*np.pi*self.freq_z

        return np.stack([xs, ys, zs])
    
@dataclass
class Growing_Lissajou_3D(ParametricLineTrajectory):
    name:str = "Growing Lissajou 3D"
    ax: float = 1.
    ay: float = 1.
    az: float = 1.
    freq_y: float = 1.
    phase_y: float = 0.
    freq_z: float = 1.
    phase_z: float = 0.

    @staticmethod
    def class_id() -> int:
        return 5

    @staticmethod
    def class_dim() -> int:
        return 3

    @staticmethod
    def from_message(msg: PprzMessage) -> Growing_Lissajou_3D:
        assert msg.name == "GVF_PARAMETRIC" and int(
            msg.get_field(0)) == Growing_Lissajou_3D.class_id()

        param = [float(x) for x in msg.get_field(2)]
        ax = param[0]
        ay = param[1]
        az = param[2]
        freq_y = param[3]
        phase_y = param[4]
        freq_z = param[5]
        phase_z = param[6]

        output = Growing_Lissajou_3D(ax=ax,ay=ay, freq_y=freq_y, phase_y=phase_y, az=az,
                             freq_z=freq_z, phase_z=phase_z)
        output.parse_gvf_config(msg)
        return output

    def _param_point(self, t: float) -> np.ndarray:
        xs = self.ax * t
        ys = np.cos(2*np.pi*self.freq_y*t + self.phase_y)*self.ay*t
        zs = np.sin(2*np.pi*self.freq_z*t + self.phase_z)*self.az*t

        return np.stack([xs, ys, zs])

    def _grad_param_point(self, t: float, step: float = 0.005) -> np.ndarray:
        xs = self.ax
        ys = self.ay * np.cos(2*np.pi*self.freq_y*t + self.phase_y) - self.ay * 2*np.pi*self.freq_y * t * np.sin(2*np.pi*self.freq_y*t + self.phase_y);
        zs = self.az * np.sin(2*np.pi*self.freq_z*t + self.phase_z) + self.az * 2*np.pi*self.freq_z * t * np.cos(2*np.pi*self.freq_z*t + self.phase_z);

        return np.stack([xs, ys, zs])


@dataclass
class Drifting_Ellipse_3D(ParametricLineTrajectory):
    name:str = "Drifting Ellipse 3D"
    vx: float = 1.
    ax: float = 1.
    ay: float = 1.
    freq: float = 1.
    phase: float = 0.

    @staticmethod
    def class_id() -> int:
        return 6

    @staticmethod
    def class_dim() -> int:
        return 3

    @staticmethod
    def from_message(msg: PprzMessage) -> Drifting_Ellipse_3D:
        assert msg.name == "GVF_PARAMETRIC" and int(
            msg.get_field(0)) == Drifting_Ellipse_3D.class_id()

        param = [float(x) for x in msg.get_field(2)]
        vx = param[0]
        ax = param[1]
        ay = param[2]
        freq = param[3]
        phase = param[4]

        output = Drifting_Ellipse_3D(vx=vx,ax=ax,ay=ay, freq=freq, phase=phase)
        output.parse_gvf_config(msg)
        return output

    def _param_point(self, t: float) -> np.ndarray:
        xs = self.vx * t + self.ax * t * np.cos(2*np.pi*self.freq * t + self.phase)
        ys = self.ay * t * np.sin(2*np.pi*self.freq * t + self.phase)
        zs = np.zeros(np.shape(t))

        return np.stack([xs, ys, zs])

    def _grad_param_point(self, t: float, step: float = 0.005) -> np.ndarray:
        u = 2 * np.pi * self.freq
        xs = self.vx + self.ax * (np.cos(u*t + self.phase) - u*t*np.sin(u*t+self.phase))
        ys = self.ay * (np.sin(u*t + self.phase) + u*t*np.cos(u*t+self.phase))
        zs = np.zeros(np.shape(t))
        
        return np.stack([xs, ys, zs])

@dataclass
class Torus_3D(SurfaceTrajectory):

    @staticmethod
    def class_id() -> int:
        return 3

    @staticmethod
    def class_dim() -> int:
        return 3
