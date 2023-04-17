from __future__ import annotations

import typing

from dataclasses import dataclass
from abc import ABC,abstractmethod

import numpy as np
from numpy import linalg
from scipy.spatial.transform import Rotation

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

################## Global definition of a trajectory ##################

class Trajectory(ABC):
    @abstractmethod
    def at(self,t:float) -> np.ndarray:
        raise NotImplementedError()
    
    def __call__(self, t:float) -> np.ndarray:
        return self.at(t)
    
    @abstractmethod
    def grad(self,t:float,step:float=0.001) -> np.ndarray:
        return (self.at(t-step) + self.at(t+step))/(2*step)
    
    @abstractmethod
    def hess(self,t:float,step:float=0.001) -> np.ndarray:
        return (self.grad(t-step) + self.grad(t+step))/(2*step)

    def mobile_axes(self,t:float) -> np.ndarray:
        tangential = self.grad(t)
        normal = self.hess(t)
        binormal = np.cross(tangential,normal)
        return np.stack([tangential,normal,binormal])
    
    def mobile_unit_axes(self,t:float) -> np.ndarray:
        tangential = self.grad(t)
        normal = self.hess(t)
        binormal = np.cross(tangential,normal)
        return np.stack([tangential/linalg.norm(tangential),normal/linalg.norm(normal),binormal/linalg.norm(binormal)])
    
    def curvature(self,t:float,step:float=0.001) -> float:
        return linalg.norm(self.hess(t,step))
    
    def radius_of_curvature(self,t:float,step:float=0.001) -> float:
        return 1/self.curvature(t,step)
    
    def __add__(self,other:typing.Union[Trajectory,np.ndarray]) -> Trajectory:
        if isinstance(other,Trajectory):
            return Sum(self,other)
        elif isinstance(other,np.ndarray):
            return Translate(self,other)
        else:
            raise NotImplementedError()
    
    def __mul__(self,other:typing.Union[Trajectory,Rotation,np.ndarray,float]) -> Trajectory:
        if isinstance(other,Trajectory):
            return Composed(other,self)
        elif isinstance(other,Rotation):
            return Rotate(self,other)
        elif isinstance(other,np.ndarray):
            return LinearTransform(self,other)
        elif isinstance(other,float):
            return RescaleOut(self,other)
        else:
            NotImplementedError()

################## Classes to define operations between trajectories ##################

@dataclass
class Composed(Trajectory):
    base:Trajectory
    wrap:Trajectory
    
    def at(self,t:float) -> np.ndarray:
        axes = self.base.mobile_axes(t)
        return self.base.at(t) + axes.transpose().dot(self.wrap.at(t))

    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return super().grad(t, step)
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return super().hess(t, step)

@dataclass 
class Sum(Trajectory):
    traj1:Trajectory
    traj2:Trajectory
    
    def at(self,t:float) -> np.ndarray:
        return self.traj1(t) + self.traj2(t)
    
    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.traj1.grad(t,step) + self.traj2.grad(t,step)
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.traj1.hess(t,step) + self.traj2.hess(t,step)

@dataclass
class Translate(Trajectory):
    traj:Trajectory
    translation:np.ndarray
    
    def at(self, t: float) -> np.ndarray:
        return self.translation + self.traj(t)
    
    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.traj.grad(t,step)
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.traj.hess(t,step)
    
@dataclass 
class Rotate(Trajectory):
    traj:Trajectory
    rot:Rotation

    def at(self, t: float) -> np.ndarray:
        return self.rot.apply(self.traj(t))
    
    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.rot.apply(self.traj.grad(t,step))
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.rot.apply(self.traj.hess(t,step))
    
@dataclass
class LinearTransform(Trajectory):
    traj:Trajectory
    transform:np.ndarray
    
    def at(self, t: float) -> np.ndarray:
        return self.transform.dot(self.traj(t))
    
    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.transform.dot(self.traj.grad(t,step))
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.transform.dot(self.traj.hess(t,step))
    
@dataclass
class RescaleOut(Trajectory):
    traj:Trajectory
    s:float
    
    def at(self, t: float) -> np.ndarray:
        return self.s*self.traj(t)
    
    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.s*self.traj.grad(t,step)
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.s*self.traj.hess(t,step)
    
@dataclass
class RescaleIn(Trajectory):
    traj:Trajectory
    s:float
    
    def at(self, t: float) -> np.ndarray:
        return self.traj(self.s*t)
    
    def grad(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.traj.grad(self.s*t,step)
    
    def hess(self, t: float, step: float = 0.001) -> np.ndarray:
        return self.traj.hess(self.s*t,step)
    

################## Actual trajectories ##################

@dataclass
class Rosace(Trajectory):
    k:float = 2.
    
    def at(self,t:float) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = np.cos(self.k*t)*np.sin(t)
        zs = np.cos(self.k*t)*np.cos(t)

        return np.stack([xs, ys, zs])
    
    def grad(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = - self.k * np.sin(self.k * t) * np.sin(t) + np.cos(self.k * t) * np.cos(t)
        zs = - self.k * np.sin(self.k * t) * np.cos(t) - np.cos(self.k * t) * np.sin(t)

        return np.stack([xs, ys, zs])
    
    def hess(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = -(1 + self.k * self.k) * np.cos(self.k * t) * np.sin(t) - 2 * self.k * np.cos(t) * np.sin(self.k * t)
        zs = -(1 + self.k * self.k) * np.cos(t) * np.cos(self.k * t) + 2 * self.k * np.sin(t) * np.sin(self.k * t)
        
        return np.stack([xs, ys, zs])

@dataclass
class Sinusoid3D(Trajectory):
    a_x:float = 1.
    a_y:float = 1.
    a_z:float = 1.
    w_y:float = 1.
    w_z:float = 1.
    phi:float = 0.
    
    def at(self,t:float) -> np.ndarray:
        xs = t*self.a_x
        ys = np.sin(self.w_y*t)*self.a_y
        zs = np.sin(self.w_z*t + self.phi)*self.a_z

        return np.stack([xs, ys, zs])
    
    def grad(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.ones(np.shape(t))*self.a_x
        ys = np.cos(self.w_y*t)*self.a_y * self.w_y
        zs = np.cos(self.w_z*t + self.phi)*self.a_z *self.w_z

        return np.stack([xs, ys, zs])
    
    def hess(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = - np.sin(self.w_y * t)*self.a_y * self.w_y * self.w_y
        zs = - np.sin(self.w_z * t)*self.a_z * self.w_z * self.w_z
        
        return np.stack([xs, ys, zs])
    
@dataclass
class Circle(Trajectory):
    r:float = 1.
    w:float = 1.
    
    def at(self,t:float) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = np.cos(self.w*t)*self.r
        zs = np.sin(self.w*t)*self.r

        return np.stack([xs, ys, zs])
    
    def grad(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = - np.sin(self.w*t)*self.r * self.w
        zs = np.cos(self.w*t)*self.r * self.w

        return np.stack([xs, ys, zs])
    
    def hess(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = - np.cos(self.w*t)*self.r * self.w * self.w
        zs = - np.sin(self.w*t)*self.r * self.w * self.w
        
        return np.stack([xs, ys, zs])
    
@dataclass
class Ellipse(Trajectory):
    a:float = 1.
    b:float = 2.
    w:float = 1.
    
    def at(self,t:float) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = np.cos(self.w*t)*self.a
        zs = np.sin(self.w*t)*self.b

        return np.stack([xs, ys, zs])
    
    def grad(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = - np.sin(self.w*t)*self.r * self.a
        zs = np.cos(self.w*t)*self.r * self.b

        return np.stack([xs, ys, zs])
    
    def hess(self,t:float,step:float=0.001) -> np.ndarray:
        xs = np.zeros(np.shape(t))
        ys = - np.cos(self.w*t)*self.r * self.a * self.a
        zs = - np.sin(self.w*t)*self.r * self.b * self.b
        
        return np.stack([xs, ys, zs])
    
@dataclass
class Line3D(Trajectory):
    offset:np.ndarray
    direction:np.ndarray
    
    def at(self,t:float) -> np.ndarray:
        return self.offset + t*self.direction
    
    def grad(self,t:float,step:float=0.001) -> np.ndarray:
        return np.repeat(self.direction,np.shape(t))
    
    def hess(self,t:float,step:float=0.001) -> np.ndarray:
        return np.repeat(np.zeros(3),np.shape(t))
    

################## Program entry-point ##################
    
def main():
    
    spiral = Sinusoid3D(phi=np.pi/2,w_y=4,w_z=4)
    slow_spiral = Sinusoid3D(phi=np.pi/2,w_y=0.5,w_z=0.5,a_y=5,a_z=5)
    fast_small_spiral = Sinusoid3D(phi=np.pi/2,w_y=6,w_z=6,a_y=0.8,a_z=0.8)
    vertical_sin = Sinusoid3D(a_x=4,a_y=0,a_z=4,w_z=2)
    circle = Circle(r=1,w=10)
    vert_circle = Rotate(circle,Rotation.from_euler('y',90,True))
    vert_spir = Sum(vert_circle,vertical_sin)
    rosace = Rosace(np.pi)
    
    circ_around_spir = Sum(slow_spiral,vert_circle)
    
    fig = plt.figure()
    a3d:Axes3D = fig.add_subplot(projection='3d')
    
    range_ = np.linspace(-10,10,1000)
    #base_traj = np.array([slow_spiral.at(t) for t in range_])
    #axes = np.array([sinsin.base.mobile_axes(t) for t in range_])
    traj = np.array([circ_around_spir.at(t) for t in range_])
    #print(traj.shape)
    a3d.plot(traj[:,0],traj[:,1],traj[:,2],label="Composed trajectory",color='blue')
    #a3d.plot(base_traj[:,0],base_traj[:,1],base_traj[:,2],label="Base trajectory",color='black')
    #a3d.quiver3D(base_traj[:,0],base_traj[:,1],base_traj[:,2],
    #             axes[:,0,0],axes[:,0,1],axes[:,0,2],label="Tangents",color='yellow')
    #a3d.quiver3D(base_traj[:,0],base_traj[:,1],base_traj[:,2],
    #             axes[:,1,0],axes[:,1,1],axes[:,1,2],label="Normals",color='green')
    #a3d.quiver3D(base_traj[:,0],base_traj[:,1],base_traj[:,2],
    #             axes[:,2,0],axes[:,2,1],axes[:,2,2],label="Binormals",color='red')
    a3d.axis('equal')
    
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
    