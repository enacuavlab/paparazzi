
import typing

import numpy as np
import scipy.optimize as opt
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

"""
def _gaussian(x: float, sigma: float) -> float:
    return np.exp(-np.power(x/sigma, 2)/2)/(np.sqrt(2*np.pi)*sigma)
"""
    
#################### Common functions ####################

def gaussian_transform(popt:np.ndarray,pos:np.ndarray) -> np.ndarray:
    rot = Rotation.from_rotvec(
            popt[0] * np.array([0, 0, 1], dtype=float))
    
    xo = popt[1]
    yo = popt[2]
    h = popt[3]
    tr = np.asarray([xo,yo,h],dtype=float)
    
    return rot.apply(pos,inverse=True) - tr

def gaussian_inv_transform(popt:np.ndarray,pos:np.ndarray) -> np.ndarray:
    rot = Rotation.from_rotvec(
            popt[0] * np.array([0, 0, 1], dtype=float))
    
    xo = popt[1]
    yo = popt[2]
    h = popt[3]
    tr = np.asarray([xo,yo,h],dtype=float)
    
    return rot.apply(pos + tr)


#################### Gaussian tube ####################

def gaussian_tube_params():
    return ["Angle", "x0", "y0", "Height", "q", "S_y", "S_z"]

def gaussian_tube_units():
    return ["rad", "as x", "as y", "as z", "a.u.", "as y", "as z"]

def gaussian_tube_description(popt:np.ndarray,pcov:np.ndarray) -> None:
    params = gaussian_tube_params()
    units = gaussian_tube_units()
    
    print("Gaussian tube model:")
    for i,(name, val, cov, unit) in enumerate(zip(params, popt, pcov, units)):
        print(f"\t- {name} : {val} ± {cov[i]}  ({unit})")

def gaussian_tube_sample_filter(samples:np.ndarray) -> np.ndarray:
    barycenter = np.average(samples, axis=0)
    significant = samples[samples[:, 3] > barycenter[3]/1000]
    return significant

def gaussian_tube_sy(popt:np.ndarray) -> typing.Callable[[float],float]:
    return lambda x : popt[-2]

def gaussian_tube_sz(popt:np.ndarray) -> typing.Callable[[float],float]:
    return lambda x : popt[-1]

def fit_gaussian_tube(samples: np.array) -> typing.Tuple[np.array, np.array]:
    # Normalize
    minima = np.amin(samples,axis=0)
    maxima = np.amax(samples,axis=0)
    rescale_vect = maxima-minima
    
    normalized = (samples - minima)/rescale_vect
    
    # Functions definition

    def gaussian_tube(pos: np.array, angle: float, xo: float, yo: float, h: float, q: float, sy: float, sz: float) -> float:
        """
        Ground othogonal 2D Gaussian distribution extended to the space according to a direction 
        With X,Y,Z the coordinates along the axis, we have:
        C(X,Y,Z) = q * gaussian(Y,sy) * gaussian(Z,sz)

        [With gaussian(x,s) = e^(-x²/2s²)/sqrt(2pi)s]

        @param pos  : [x,y,z] position of the point we want to know the concentration of
        @param angle,xo,yo: Define the ground center axis of the Gaussian distribution (angle in radiants)
        @param h    : elevation of the 2D Gaussian above z=0
        @param q    : Rescale coefficient
        @param sy   : Horizontal variance
        @param sz   : Vertical variance
        """
        # Linear convertion to local plume
        # (A in direction in the XY plane, with (1,0) at 0, and the axe's heigh)

        rot = np.transpose(Rotation.from_rotvec(
            angle * np.array([0, 0, 1], dtype=float)).as_matrix())

        gpos = rot.dot(pos) - \
            np.reshape(np.array([xo, yo, h], dtype=float), (3, 1))

        
        exponent_Y = gpos[1]/sy
        exponent_Y = - exponent_Y * exponent_Y / 2
        exponent_Z = gpos[2]/sz
        exponent_Z = - exponent_Z * exponent_Z / 2

        output = q * np.exp(exponent_Y + exponent_Z)/(2 * np.pi * sy * sz)
        return output

    def gaussian_tube_jac(pos: np.array, angle: float, xo: float, yo: float, h: float, q: float, sy: float, sz: float) -> np.array:
        cos_t = np.cos(angle)
        sin_t = np.sin(angle)

        angle_derivative = (pos[0]*cos_t - pos[1]*sin_t) * \
            (pos[0]*sin_t + pos[1]*cos_t - yo)/(sy*sy)
        xo_derivative = np.zeros(pos.shape[1])
        yo_derivative = (pos[0]*sin_t + pos[1]*cos_t - yo)/(sy*sy)
        h_derivative = (pos[2] - h)/(sz*sz)
        q_derivative = np.ones(pos.shape[1]) / q
        sy_derivative = -(sy - pos[0]*sin_t - pos[1]*cos_t + yo) * \
            (sy + pos[0]*sin_t + pos[1] * cos_t - yo)/(sy*sy*sy)
        sz_derivative = ((pos[2]-h)*(pos[2]-h) - sz*sz)/(sz*sz*sz)

        return np.transpose(np.asarray([angle_derivative,
                         xo_derivative,
                         yo_derivative,
                         h_derivative,
                         q_derivative,
                         sy_derivative,
                         sz_derivative], dtype=float) * gaussian_tube(pos, angle, xo, yo, h, q, sy, sz))

    # Initial variable guesses
    barycenter = np.average(normalized, axis=0)
    angle = np.arctan2(
        barycenter[0], barycenter[1])

    # Fit
    # (Provide adversarial angle to motivate correcting it)
    popt,pcov = opt.curve_fit(gaussian_tube, np.transpose(normalized[:, :3]), normalized[:, 3],
                         [angle+np.pi/2, barycenter[0],
                             barycenter[1], barycenter[2], 1., 0.1,0.1],
                         method='lm',
                         jac=gaussian_tube_jac)
    
    angle,xo,yo,h,q,sy,sz = popt
    
    # De-normalize
    
    def to_axial_referential(pos):
        rot = Rotation.from_rotvec(
                angle * np.array([0, 0, 1], dtype=float))

        return rot.apply(pos) - np.asarray([xo, yo, h], dtype=float)
    
    axial_minima = to_axial_referential(minima[:3])
    axial_rescale = to_axial_referential(rescale_vect[:3])
    
    param_offset = np.asarray([0,minima[0],minima[1],minima[2],0,axial_minima[1],axial_minima[2]],dtype=float)
    param_rescale = np.asarray([1,rescale_vect[0],rescale_vect[1],rescale_vect[2],rescale_vect[3],axial_rescale[1],axial_rescale[2]],dtype=float)
    
    
    
    return np.multiply(popt,param_rescale)+ param_offset,np.multiply(pcov,param_rescale.transpose()*param_rescale)

#################### 3D simple model ####################
# i.e. no gas elevation considered, no source on the ground

def simple_gaussian_params():
    return ["Angle", "x0", "y0", "Height", "q", "a_y", "a_z"]

def simple_gaussian_units():
    return ["rad", "as x", "as y", "as z", "a.u.", "scalar", "scalar"]

def simple_gaussian_description(popt:np.ndarray,pcov:np.ndarray) -> None:
    params = simple_gaussian_params()
    units = simple_gaussian_units()
    
    print("Simple gaussian model:")
    for i,(name, val, cov, unit) in enumerate(zip(params, popt, pcov, units)):
        print(f"\t- {name} : {val} ± {cov[i]}  ({unit})")

def simple_gaussian_sample_filter(samples:np.ndarray) -> np.ndarray:
    return samples

def simple_gaussian_sy(popt:np.ndarray) -> typing.Callable[[float],float]:
    return lambda x : popt[-2] * x

def simple_gaussian_sz(popt:np.ndarray) -> typing.Callable[[float],float]:
    return lambda x : popt[-1] * x

def fit_simple_gaussian(samples:np.ndarray):
    # Normalize
    minima = np.amin(samples,axis=0)
    maxima = np.amax(samples,axis=0)
    rescale_vect = maxima-minima
    
    normalized = (samples - minima)/rescale_vect
    
    # Functions definition

    def simple_gaussian(pos: np.array, angle: float, xo: float, yo: float, h: float, q: float, ay: float, az: float) -> float:
        """
        Ground othogonal Gaussian distribution extended to the space according to a direction, with linear growth of variance
        With X,Y,Z the coordinates along the axis, we have:
        C(X,Y,Z) = q * gaussian(Y,sy(x)) * gaussian(Z,sz(x))

        [With gaussian(x,s) = e^(-x²/2s²)/sqrt(2pi)s and sy(x) = a_y * x and sz(x) = a_z * x]

        @param pos  : [x,y,z] position of the point we want to know the concentration of
        @param angle,xo,yo: Define the ground center axis of the Gaussian distribution (angle in radiants)
        @param h    : elevation of the 2D Gaussian above z=0
        @param q    : Rescale coefficient
        @param ay   : Horizontal variance growth
        @param az   : Vertical variance growth
        """
        # Linear convertion to local plume
        # (A in direction in the XY plane, with (1,0) at 0, and the axe's heigh)

        rot = np.transpose(Rotation.from_rotvec(
            angle * np.array([0, 0, 1], dtype=float)).as_matrix())

        gpos = rot.dot(pos) - \
            np.reshape(np.array([xo, yo, h], dtype=float), (3, 1))

        
        exponent_Y = gpos[1]/(ay * gpos[0])
        exponent_Y = - exponent_Y * exponent_Y / 2
        exponent_Z = gpos[2]/(az * gpos[0])
        exponent_Z = - exponent_Z * exponent_Z / 2

        output = q * np.exp(exponent_Y + exponent_Z)/(2 * np.pi * ay * az * np.square(gpos[0]))
        return output
    
    """
    def gaussian_tube_jac(pos: np.array, angle: float, xo: float, yo: float, h: float, q: float, ay: float, az: float) -> np.array:
        cos_t = np.cos(angle)
        sin_t = np.sin(angle)

        gx = cos_t * x - sin_t * y - xo
        gy = sin_t * x + cos_t * y - yo

        angle_derivative = 
        xo_derivative = np.zeros(pos.shape[1])
        yo_derivative = (pos[0]*sin_t + pos[1]*cos_t - yo)/(sy*sy)
        h_derivative = (pos[2] - h)/(sz*sz)
        q_derivative = np.ones(pos.shape[1]) / q
        sy_derivative = -(sy - pos[0]*sin_t - pos[1]*cos_t + yo) * \
            (sy + pos[0]*sin_t + pos[1] * cos_t - yo)/(sy*sy*sy)
        sz_derivative = ((pos[2]-h)*(pos[2]-h) - sz*sz)/(sz*sz*sz)

        return np.transpose(np.asarray([angle_derivative,
                         xo_derivative,
                         yo_derivative,
                         h_derivative,
                         q_derivative,
                         sy_derivative,
                         sz_derivative], dtype=float) * gaussian_tube(pos, angle, xo, yo, h, q, sy, sz))
    """
    
    # Initial variable guesses
    barycenter = np.average(normalized, axis=0)
    angle = np.arctan2(
        barycenter[0], barycenter[1])

    # Fit
    # (Provide adversarial angle to motivate correcting it)
    popt,pcov = opt.curve_fit(simple_gaussian, np.transpose(normalized[:, :3]), normalized[:, 3],
                         [angle+np.pi/2, barycenter[0],
                             barycenter[1], barycenter[2], 1., 0.1,0.1],
                         method='lm')
    
    angle,xo,yo,h,q,ay,az = popt
    
    # De-normalize
    
    def to_axial_referential(pos):
        rot = Rotation.from_rotvec(
                angle * np.array([0, 0, 1], dtype=float))

        return rot.apply(pos) - np.asarray([xo, yo, h], dtype=float)
    
    axial_minima = to_axial_referential(minima[:3])
    axial_rescale = to_axial_referential(rescale_vect[:3])
    
    param_offset = np.asarray([0,minima[0],minima[1],minima[2],0,axial_minima[1],axial_minima[2]],dtype=float)
    param_rescale = np.asarray([1,rescale_vect[0],rescale_vect[1],rescale_vect[2],rescale_vect[3],axial_rescale[1],axial_rescale[2]],dtype=float)
    
    
    
    return np.multiply(popt,param_rescale)+ param_offset,np.multiply(pcov,param_rescale.transpose()*param_rescale)

