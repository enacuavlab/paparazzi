from __future__ import annotations

from dataclasses import dataclass,field
from abc import ABC,abstractmethod

from .gvf_trajectories import LineTrajectory,SurfaceTrajectory
from pprzlink.message import PprzMessage

import typing
import numpy as np
from scipy.spatial.transform import Rotation

#################### Abstract class specific to parametric trajectories ####################

class ParametricLineTrajectory(LineTrajectory,ABC):
    def parse_affine_transform(self,msg:PprzMessage) -> None:
        self.rot = Rotation.from_quat(np.array(msg.get_field(5),dtype=float))
        self.XYZoffset = np.array(msg.get_field(6),dtype=float)
        
    def gvf(self, pos: np.ndarray, t: float) -> np.ndarray:
        """
        The GVF equation for paramertic line trajectories
        (cf https://doi.org/10.48550/arXiv.2012.01826 ; proof of Theorem 2)
        """
        ft = self.param_point(t)
        phi = (pos - ft) * self.gain
        
        dir = 1-2*(self.dim % 2) # = (-1)^dim
        return  dir * self.grad_param_point(t) - phi

#################### Actually defined parametric trajectories ####################

@dataclass
class Trefoil_2D(ParametricLineTrajectory):
    
    @staticmethod
    def class_id() -> int:
        return 0
    
    @staticmethod
    def class_dim() -> int:
        return 2

@dataclass
class Ellipse_3D(ParametricLineTrajectory):
    name:str = "3D Ellipse"
    r:float = 0 # Radius on the XY projection
    zl:float = 0 # Highest point
    zh:float = 0 # Lowest point
    alpha:float = 0 # Low-high orientation
    _traj_points:np.ndarray = field(default=None,init=False) # Closed curve: memorize trajectory
    
    @staticmethod
    def class_id() -> int:
        return 1
    
    @staticmethod
    def class_dim() -> int:
        return 3
    
    @staticmethod
    def from_message(msg:PprzMessage) -> Ellipse_3D:
        assert msg.name == "GVF_PARAMETRIC" and int(msg.get_field(1)) == Ellipse_3D.class_id()
        
        param = [float(x) for x in msg.get_field(3)]
        xo = param[0]
        yo = param[1]
        r = param[2]
        zl = param[3]
        zh = param[4]
        alpha = param[5]
        zm = (zl+zh)/2
        
        output = Ellipse_3D(r=r,zl=zl-zm,zh=zh-zm,alpha=alpha)
        output.parse_affine_transform(msg)
        output.XYZoffset += np.array([xo,yo,zm])
        
        return output
    
    def _param_point(self,t:float) -> np.ndarray:
        xs = np.cos(t)*self.r
        ys = np.sin(t)*self.r
        zs = (self.zl-self.zh)*np.sin(self.alpha - t)/2
        
        return np.stack([xs,ys,zs])
        
    def _grad_param_point(self, t:float, step:float=0.005) -> np.ndarray:
        xs = -np.sin(t)*self.r
        ys = np.cos(t)*self.r
        zs = (self.zl+self.zh)*np.cos(self.alpha - t)/2
        
        return  np.stack([xs,ys,zs])
    
    def calc_traj(self, range: np.ndarray) -> np.ndarray:
        if self._traj_points is None:
            self._traj_points = super().calc_traj(np.linspace(0,2*np.pi,len(range)))
        elif len(self._traj_points) < len(range):
            self._traj_points = super().calc_traj(np.linspace(0,2*np.pi,len(range)))
        return self._traj_points

@dataclass
class Lissajous_3D(ParametricLineTrajectory):
    
    @staticmethod
    def class_id() -> int:
        return 2
    
    @staticmethod
    def class_dim() -> int:
        return 3

@dataclass
class Sinusoid_3D(ParametricLineTrajectory):
    
    @staticmethod
    def class_id() -> int:
        return 4
    
    @staticmethod
    def class_dim() -> int:
        return 3

@dataclass
class Torus_3D(SurfaceTrajectory):
    
    @staticmethod
    def class_id() -> int:
        return 3
    
    @staticmethod
    def class_dim() -> int:
        return 3

