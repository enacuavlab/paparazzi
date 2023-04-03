from __future__ import annotations

import typing
import enum
from dataclasses import dataclass
from abc import ABC,abstractmethod

import numpy as np
from scipy.spatial.transform import Rotation

from pprzlink.message import PprzMessage
from pprzlink.generated.telemetry import PprzMessage_GVF,PprzMessage_GVF_PARAMETRIC

#################### Generic classes ####################

@dataclass
class Trajectory(ABC):
    gain:np.ndarray = np.ones(1) # Gain vector
    name:str = "Trajectory" # Name for the trajectory
    XYZoffset:np.ndarray = np.zeros(3)
    rot:Rotation = Rotation.identity() # Rotation applied to the trajectory
    
    ##### Abstract methods #####
    
    @staticmethod
    @abstractmethod
    def from_message(msg:PprzMessage) -> Trajectory:
        """
        Build the object from the corresponding PprzMessage
        (may assert message correctness)
        """
        raise NotImplementedError()
    
    @abstractmethod
    def _param_point(self,t:typing.Union[float,np.ndarray]) -> np.ndarray:
        """
        Given a parameter (a float most of the time, but may be 2D
        for parametric surfaces), compute its associated point
        """
        raise NotImplementedError()
        
    def param_point(self,t:typing.Union[float,np.ndarray]) -> np.ndarray:
        """
        Compute the actual trajectory point (applies rotation and translation to the 'raw' point)
        @see _param_point
        """
        raw_point = self._param_point(t).transpose()
        return self.rot.apply(raw_point) + self.XYZoffset
        
    @staticmethod
    @abstractmethod
    def class_id() -> int:
        """
        The class' ID, as defined in its module's source
        """
        raise NotImplementedError()
    
    @staticmethod
    @abstractmethod
    def class_dim() -> int:
        """
        The *output* dimension of the trajectory (2 or 3).
        Internally, only vectors of dimension 3 are handled (streamline management)
        For a 2D curve, the 3rd coordinate is always neglected/set to 0
        """
        raise NotImplementedError()
        
    @abstractmethod
    def gvf(self, pos:np.ndarray, t:typing.Union[float,np.ndarray]) -> np.ndarray:
        """
        Given a position and the trajectory's parameter, compute the corresponding vector
        of the guiding field.
        """
        raise NotImplementedError()
        

    ##### Shorthands #####
    
    @property
    def id(self) -> int:
        """
        ID used to identify the trajectory type inside its class
        (that is, either a 'GVF' trajectory, or a 'GVF_PARAMETRIC' one)
        """
        return self.class_id()    
    
    @property
    def dim(self) -> int:
        """
        The *output* dimension of the trajectory (2 or 3)
        @see self.class_dim
        """
        return self.class_dim()
        
    @property
    def XYoffset(self) -> np.ndarray:
        """
        Shortand to get the 2D offset vector
        """
        return self.XYZoffset[:2]
    
    @XYoffset.setter
    def XYoffset(self, a:np.ndarray) -> None:
        self.XYZoffset[0] = a[0]
        self.XYZoffset[1] = a[1]
        
    @property
    def x_offset(self) -> float:
        return self.XYZoffset[0]
    
    @x_offset.setter
    def x_offset(self,a:float) -> None:
        self.XYZoffset[0] = a
        
    @property
    def y_offset(self) -> float:
        return self.XYZoffset[1]
    
    @y_offset.setter
    def y_offset(self,a:float) -> None:
        self.XYZoffset[1] = a
        
    @property
    def z_offset(self) -> float:
        return self.XYZoffset[2]
    
    @z_offset.setter
    def z_offset(self,a:float) -> None:
        self.XYZoffset[2] = a    

@dataclass
class LineTrajectory(Trajectory,ABC):
    gain:np.ndarray = np.ones(3)
    def _grad_param_point(self, t:float, step:float=0.005) -> np.ndarray:
        """
        Estimated derivative of the curve's equation
        (Finite difference, middle point approximation)
        
        Note: you may (and should) reimplement this function in subclasses, 
        using the analytical expression instead
        """
        t_minus = t - step
        t_plus = t + step
        f_minus = self._param_point(t_minus)
        f_plus = self._param_point(t_plus)
        return (f_plus - f_minus) / 2*step
    
    def grad_param_point(self, t:float, step:float=0.005) -> np.ndarray:
        """
        Compute the actual gradient (apply the rotation to the 'raw' gradient)
        @see _grad_param_point
        """
        raw_grad = self._grad_param_point(t,step).transpose()
        return self.rot.apply(raw_grad)
         
    def calc_traj(self,range:np.ndarray) -> np.ndarray:
        """
        Compute and return the trajectory given the current state (curve's parameters)
        """
        return self.param_point(range)
        
    def calc_field(self,meshgrid:np.ndarray,t:float) -> np.ndarray:
        """
        Compute and return the vector field given the current state (curve's parameters)
        and the 'virtual coordinate'
        """
             
        l_i,l_j,l_k,_ = meshgrid.shape
        output = meshgrid.copy()
        for i in range(l_i):
            for j in range(l_j):
                for k in range(l_k):
                    output[i,j,k] = self.gvf(meshgrid[i,j,k],t)
        return output
        

@dataclass
class SurfaceTrajectory(Trajectory,ABC):
    pass

    