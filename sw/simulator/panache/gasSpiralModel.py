
import numpy as np
import scipy.optimize as opt
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

import typing


def _gaussian(x: float, sigma: float) -> float:
    return np.exp(-np.power(x/sigma, 2)/2)/(np.sqrt(2*np.pi)*sigma)


def find_main_axis(samples: np.array) -> typing.Tuple[np.array, np.array]:
    # Normalize
    minima = np.amin(samples,axis=0)
    maxima = np.amax(samples,axis=0)
    rescale_vect = maxima-minima
    
    normalized = (samples - minima)/rescale_vect
    
    # Functions definition

    def gaussian_tube(pos: np.array, angle: float, xo: float, yo: float, h: float, q: float, sy: float, sz: float) -> float:
        """
        Vertical 2D Gaussian distribution extended to the space according to a direction 
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


def show_samples(samples: np.array) -> None:
    
    # Provides only the most significant points
    barycenter = np.average(samples, axis=0)
    significant = samples[samples[:, 3] > barycenter[3]/1000]
    significant_barycenter = np.average(significant, axis=0)
    
    # Model fitting
    popt, pcov = find_main_axis(significant)

    # Recover and display model params
    params = ["Angle", "x0", "y0", "Height", "q", "S_y", "S_z"]
    units = ["rad", "as x", "as y", "as z", "a.u.", "as y", "as z"]

    print("2D Gaussian model (vertical cut, orthogonal to wind axis):")
    for i,(name, val, cov, unit) in enumerate(zip(params, popt, pcov, units)):
        print(f"\t- {name} : {val} ± {cov[i]}  ({unit})")

    
    # Plot acquired data
    xs = significant[:, 0]
    ys = significant[:, 1]
    zs = significant[:, 2]
    vals = significant[:, 3]+1e-9

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Distribution's axis
    cos_t = np.cos(popt[0])
    sin_t = np.cos(popt[0])
    x0 = popt[1]
    y0 = popt[2]
    h = popt[3]
    sy = popt[-2]
    sz = popt[-1]
    c = sin_t * x0 - cos_t * y0

    if cos_t != 0:
        def line_eq(x): return (sin_t * x - c) / cos_t
        line_xs = np.asarray([min(xs), max(xs)],dtype=float)
        line_ys = line_eq(line_xs)
        line_zs = np.asarray([h,h],dtype=float)
    else:
        line_xs = np.asarray([x0,x0],dtype=float)
        line_ys = np.asarray([min(ys), max(ys)],dtype=float)
        line_zs = np.asarray([h,h],dtype=float)
    
    ax.plot([barycenter[0],significant_barycenter[0]],[barycenter[1],significant_barycenter[1]],[significant_barycenter[2],significant_barycenter[2]],label="Initial guess")
    ax.plot(line_xs,line_ys,line_zs, label="Distribution's main axis")
    
    horizontal_spread = max(np.amax(samples[:,0]) - np.amin(samples[:,0]),np.amax(samples[:,1]) - np.amin(samples[:,1]))
    if abs(sy) < horizontal_spread*5:
        ax.plot(line_xs -sin_t*sy,line_ys+cos_t*sy,line_zs,color='red')
        ax.plot(line_xs +sin_t*sy,line_ys-cos_t*sy,line_zs,color='red')
    else:
        print(f"Guessed horizontal spread is more than 10 time the horizontal data spread\n{abs(sy)} VS {horizontal_spread}")
    
    vertical_spread = (max(significant[:,2]) - min(significant[:,2]))
    if abs(sz) < vertical_spread*5:
        ax.plot(line_xs,line_ys,line_zs+sz,color='red')
        ax.plot(line_xs,line_ys,line_zs-sz,color='red')
    else:
        print("Guessed vertical spread is more than 10 time the vertical data spread\n{abs(sz)} VS {vertical_spread}")


    # Raw datapoints
    im = ax.scatter(xs, ys, zs, c=vals, marker='8', cmap='magma_r',
                    norm='log', plotnonfinite=True, alpha=0.7)
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Gas concentration (a.u., log scale)",
                       rotation=-90, va="bottom")

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.legend()
    plt.show()

