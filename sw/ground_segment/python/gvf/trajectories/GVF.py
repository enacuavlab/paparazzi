from __future__ import annotations

from dataclasses import dataclass,field
from abc import ABC,abstractmethod

from .gvf_trajectories import LineTrajectory
from pprzlink.message import PprzMessage

import typing
import numpy as np
from scipy.spatial.transform import Rotation

#################### Abstract class specific to implicit trajectories ####################

def __base_vector(size:int,num:int) -> np.ndarray:
    o = np.zeros(size)
    o[num] = 1.0
    return o

@dataclass
class ImplicitLineTrajectory(LineTrajectory,ABC):
    
    @abstractmethod
    def _implicit(self,pos:np.ndarray) -> float:
        ...
        
    def implicit(self,pos:np.ndarray) -> float:
        return self._implicit(self.rot.apply(pos - self.XYZoffset,True))
    
    def _implicit_grad(self,pos:np.ndarray,step:float=0.005) -> np.ndarray:
        output = np.zeros(len(pos))
        
        for i in range(len(pos)):
            delta_v = __base_vector(len(pos),i) * step
            p_plus = pos + delta_v
            p_minus = pos - delta_v  
            output[i] = self._implicit(p_plus) - self._implicit(p_minus)
        
        return output/(2*step)
    
    def implicit_grad(self,pos:np.ndarray, step:float=0.005) -> np.ndarray:
        c_pos = self.rot.apply(pos - self.XYZoffset,True)
        return self.rot.apply(self._implicit_grad(c_pos,step),True)
    
    def gvf(self, pos: np.ndarray, t: float) -> np.ndarray:
        E = np.array([[0,1,0],[-1,0,0],[0,0,0]],dtype=float)
        m = E - np.identity(3) * self.gain * self.implicit(pos)
        return m @ self.implicit_grad(pos)
        
#################### Actually defined implicit trajectories ####################

@dataclass
class Line(ImplicitLineTrajectory):
    alpha:float = 0. # Orientation, in radiants
    name:str = "2D line"
    
    @staticmethod
    def class_id() -> int:
        return 0
    
    @staticmethod
    def class_dim() -> int:
        return 2
    
    @staticmethod
    def from_message(msg: PprzMessage) -> Line:
        assert msg.name == "GVF" and int(msg.get_field(1)) == Line.class_id()
        
        params = [float(x) for x in msg.get_field(4)]
        
        return Line(alpha=np.radians(params[2]),XYZoffset=np.array([params[0],params[1],0]))
    
    def _implicit(self,pos: np.ndarray) -> float:
        return np.dot(pos,np.array([-np.sin(self.alpha),np.cos(self.alpha),0]))
    
    def _implicit_grad(self, pos: np.ndarray, step: float = 0.005) -> np.ndarray:
        return np.array([-np.sin(self.alpha),np.cos(self.alpha),0])
    
    def _param_point(self, t:float) -> np.ndarray:
        return np.array([t*np.cos(self.alpha),
                         t*np.sin(self.alpha),
                         0])
        
    def _grad_param_point(self, t:float, step:float=0.005) -> np.ndarray:
        return np.array([np.cos(self.alpha),np.sin(self.alpha),0])
    
        
    
@dataclass
class Ellipse(ImplicitLineTrajectory):
    ea:float = 1. # Major axis
    eb:float = 1. # Minor axis
    name:str = "2D ellipse"
    _traj_points:np.ndarray = field(default=None,init=False) # Closed curve: memorize trajectory
    
    @staticmethod
    def class_id() -> int:
        return 1
    
    @staticmethod
    def class_dim() -> int:
        return 2
    
    @staticmethod
    def from_message(msg: PprzMessage) -> Ellipse:
        assert msg.name == "GVF" and int(msg.get_field(1)) == Ellipse.class_id()
        
        
        params = [float(x) for x in msg.get_field(4)]
        rot = Rotation.from_euler('z',params[4],True)
        
        return Ellipse(ea=params[2],eb=params[3],
                           XYZoffset=np.array([params[0],params[1],0]),
                           rot=rot)
    
    def _implicit(self, pos: np.ndarray) -> float:
        rescaled = pos/np.array([self.ea,self.eb,1])
        return np.dot(rescaled,rescaled)-2
    
    def _implicit_grad(self, pos: np.ndarray, step: float = 0.005) -> np.ndarray:
        return np.array([2/(self.ea*self.ea),2/(self.eb*self.eb),0]) * pos
    
    def _param_point(self, t:float) -> np.ndarray:
        angle = 2*np.pi * t
        
        xs = np.cos(angle) * self.ea
        ys = np.sin(angle) * self.eb
        return np.stack([xs,ys,np.zeros(np.shape(xs))])

        
    def _grad_param_point(self, t:float, step:float=0.005) -> np.ndarray:
        angle = 2*np.pi * t
        
        xs = - 2*np.pi * self.ea * np.sin(angle)
        ys = 2*np.pi * self.eb * np.cos(angle)
        
        return  np.stack([xs,ys,np.zeros(np.shape(xs))])
    
    def calc_traj(self, range: np.ndarray) -> np.ndarray:
        if self._traj_points is None:
            self._traj_points = super().calc_traj(np.linspace(0,1,len(range)))
        elif len(self._traj_points) < len(range):
            self._traj_points = super().calc_traj(np.linspace(0,1,len(range)))
        return self._traj_points
    
@dataclass
class Sinusoid(ImplicitLineTrajectory):
    pass