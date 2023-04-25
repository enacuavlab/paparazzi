#!/usr/bin/env python3
import argparse
import typing
import os
import functools
import itertools


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

import sampleParser
import gasGaussianModel



def show_raw_samples(samples: np.array) -> None:

    # Plot acquired data
    xs = samples[:, 0]
    ys = samples[:, 1]
    zs = samples[:, 2]
    vals = samples[:, 3]+1e-9

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Raw datapoints
    im = ax.scatter(xs, ys, zs, c=vals, marker='8', cmap='magma_r',
                    norm='log', plotnonfinite=True, alpha=0.7)
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Gas concentration (a.u., log scale)",
                       rotation=-90, va="bottom")

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_aspect('equal')

    plt.legend()
    plt.show()
    
def show_fitted_samples(samples: np.array, model:typing.Union[typing.Literal['gaussian_tube'],typing.Literal['gaussian_simple']]) -> None:
    
    filter_fun:typing.Callable
    fit_fun:typing.Callable
    descr_fun:typing.Callable
    sy_fun:typing.Callable
    sz_fun:typing.Callable
    
    if model == 'gaussian_tube':
        filter_fun = gasGaussianModel.gaussian_tube_sample_filter
        fit_fun = gasGaussianModel.fit_gaussian_tube
        descr_fun = gasGaussianModel.gaussian_tube_description
    elif model == 'gaussian_simple':
        filter_fun = gasGaussianModel.simple_gaussian_sample_filter
        fit_fun = gasGaussianModel.fit_simple_gaussian
        descr_fun = gasGaussianModel.simple_gaussian_description
    
    
    # Provides only the most significant points
    significant = filter_fun(samples)
    significant_barycenter = np.average(significant, axis=0)
    
    # Model fitting
    popt, pcov = fit_fun(significant)

    # Recover and display model params
    descr_fun(popt,pcov)

    if model == 'gaussian_tube':
        sy_fun = gasGaussianModel.gaussian_tube_sy(popt)
        sz_fun = gasGaussianModel.gaussian_tube_sz(popt)
        
    elif model == 'gaussian_simple':
        sy_fun = gasGaussianModel.simple_gaussian_sy(popt)
        sz_fun = gasGaussianModel.simple_gaussian_sz(popt)
        
    src_pt = gasGaussianModel.gaussian_src(popt)

    
    # Plot acquired data
    xs = significant[:, 0]
    ys = significant[:, 1]
    zs = significant[:, 2]
    vals = significant[:, 3]+1e-9

    fig = plt.figure()
    ax:Axes3D = fig.add_subplot(projection='3d')
    ax.plot([src_pt[0]],[src_pt[1]],[src_pt[2]],'gp',label="Estimated source point")

    # Test planar fitter
    if model == 'gaussian_tube':
        p,res = gasGaussianModel.fit_plan(significant)
        direction = p[0]
        offset = p[1]
        range_x = np.amax(xs) - np.amin(xs)
        range_y = np.amax(ys) - np.amin(ys)
        range_z = np.amax(zs) - np.amin(zs)
        superrange = (range_x + range_y + range_z)
        pt_forward = offset + superrange * direction
        pt_backward = offset - superrange * direction
        ax.plot([pt_backward[0],pt_forward[0]],[pt_backward[1],pt_forward[1]],[pt_backward[2],pt_forward[2]],'b-.',label="Distribution's main axis (using planar fitting)")
        print("Planar model:", superrange,pt_backward,pt_forward)

    # Distribution's axis
    affine_transform = functools.partial(gasGaussianModel.gaussian_transform,popt)
    reverse_affine_transform = functools.partial(gasGaussianModel.gaussian_inv_transform,popt)
    angle = popt[0]
    x0 = popt[1]
    y0 = popt[2]
    h = popt[3]


    if angle != 0:
        def line_eq(x): return (x0-x)*np.tan(-angle) + y0
        line_xs = np.asarray([min(xs), max(xs)],dtype=float)
        line_ys = line_eq(line_xs)
        line_zs = np.asarray([h,h],dtype=float)
    else:
        line_xs = np.asarray([x0,x0],dtype=float)
        line_ys = np.asarray([min(ys), max(ys)],dtype=float)
        line_zs = np.asarray([h,h],dtype=float)
    
    ax.plot([significant_barycenter[0]],[significant_barycenter[1]],[significant_barycenter[2]],'y*',label="Barycenter of significant data",)
    ax.plot(line_xs,line_ys,line_zs, 'b-',label="Distribution's main axis")
    
    # Plot Gaussian envelope
    
    tr_pts = affine_transform(np.stack([line_xs,line_ys,line_zs]).transpose())
    
    sy = sy_fun(tr_pts[:,0])
    sz = sz_fun(tr_pts[:,0])
    print("Middle line:\n",np.stack([line_xs,line_ys,line_zs]).transpose())
    
    v_sy = np.stack([np.zeros(sy.shape),sy,np.zeros(sy.shape)]).transpose()
    v_sz = np.stack([np.zeros(sz.shape),np.zeros(sz.shape),sz]).transpose()
    
    sy_p_pt = tr_pts + v_sy
    sy_m_pt = tr_pts - v_sy
    sz_p_pt = tr_pts + v_sz
    sz_m_pt = tr_pts - v_sz
    print("Tr Line:\n",tr_pts)
    
    print("sy:\n",sy)
    print("Tr Line+sy:\n",sy_p_pt)
    print("Tr Line-sy:\n",sy_m_pt)

    sy_p_pt = reverse_affine_transform(sy_p_pt)
    sy_m_pt = reverse_affine_transform(sy_m_pt)
    sz_p_pt = reverse_affine_transform(sz_p_pt)
    sz_m_pt = reverse_affine_transform(sz_m_pt)
    
    ax.plot(sy_p_pt[:,0],sy_p_pt[:,1],sy_p_pt[:,2],color='orange',linestyle='--')
    ax.plot(sy_m_pt[:,0],sy_m_pt[:,1],sy_m_pt[:,2],color='orange',linestyle='--',label='Gaussian horizontal envelope')
    ax.plot(sz_p_pt[:,0],sz_p_pt[:,1],sz_p_pt[:,2],color='red',linestyle='--')
    ax.plot(sz_m_pt[:,0],sz_m_pt[:,1],sz_m_pt[:,2],color='red',linestyle='--',label='Gaussian vertical envelope')
    
    print("+sy line:\n",sy_p_pt)
    print("-sy line:\n",sy_m_pt)
    print("Base director: ", (line_ys[1] - line_ys[0])/(line_xs[1] - line_xs[0]))
    print("+sy director: ",(sy_p_pt[1,1] - sy_p_pt[0,1])/(sy_p_pt[1,0] - sy_p_pt[0,0]))
    print("-sy director: ",(sy_m_pt[1,1] - sy_m_pt[0,1])/(sy_m_pt[1,0] - sy_m_pt[0,0]))

    # Raw datapoints
    im = ax.scatter(xs, ys, zs, c=vals, marker='8', cmap='magma_r',
                    norm='log', plotnonfinite=True, alpha=0.7)
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Gas concentration (a.u., log scale)",
                       rotation=-90, va="bottom")

    ax.set_xlim(min(xs),max(xs))
    ax.set_ylim(min(ys),max(ys))
    ax.set_zlim(min(zs),max(zs))
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_aspect('equal')

    plt.legend()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        prog="Gas Samples displayer"
    )

    parser.add_argument("file", help="File to read gas samples from.\
        If it has extension '.pkl', use the pickle reader. Otherwise, assume TopoJSON.", nargs='+')
    parser.add_argument('-m','--model',dest='model',default=None,choices=['gaussian_tube','gt','gaussian_simple','gs'],
                        help='Model to fit to the samples')

    args = parser.parse_args()

    samples = iter([])
    for f in args.file:
        _, ext = os.path.splitext(f)
        if ext == ".pkl":
            reader = sampleParser.SampleReader_pickle(f)
        else:
            reader = sampleParser.SampleReader_TopoJSON(f)

        reader._parse_dict
        samples = itertools.chain(samples,(np.array([s.x, s.y, s.z, s.val], dtype=float) for s in reader))
        
    samples = np.stack(samples)

    if args.model is None:
        show_raw_samples(samples)
    elif args.model == 'gaussian_tube' or args.model == 'gt':
        show_fitted_samples(samples,'gaussian_tube')
    elif args.model == 'gaussian_simple' or args.model == 'gs':
        show_fitted_samples(samples,'gaussian_simple')
    else:
        raise ValueError('Unknown model: ' + str(args.model))
        
        


if __name__ == "__main__":
    main()
