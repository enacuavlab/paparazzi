#!/usr/bin/env python3

import sampleParser
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import argparse
import typing
import os
import functools

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

    
    # Plot acquired data
    xs = significant[:, 0]
    ys = significant[:, 1]
    zs = significant[:, 2]
    vals = significant[:, 3]+1e-9

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Distribution's axis
    affine_transform = functools.partial(gasGaussianModel.gaussian_transform,popt)
    reverse_affine_transform = functools.partial(gasGaussianModel.gaussian_inv_transform,popt)
    cos_t = np.cos(popt[0])
    sin_t = np.cos(popt[0])
    x0 = popt[1]
    y0 = popt[2]
    h = popt[3]
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
    
    #ax.plot([barycenter[0],significant_barycenter[0]],[barycenter[1],significant_barycenter[1]],[significant_barycenter[2],significant_barycenter[2]],label="Initial guess")
    ax.plot(line_xs,line_ys,line_zs, label="Distribution's main axis")
    
    # Plot Gaussian envelope
    
    sy_p_collection = []
    sy_m_collection = []
    sz_p_collection = []
    sz_m_collection = []
    
    tr_pts = affine_transform(np.stack([line_xs,line_zs,line_zs]).transpose())
    
    sy = sy_fun(tr_pts[:,0])
    sz = sz_fun(tr_pts[:,0])
    
    sy_p_pt = tr_pts
    sy_m_pt = tr_pts
    sz_p_pt = tr_pts
    sz_m_pt = tr_pts
    
    sy_p_pt[:,1] += 0
    sy_m_pt[:,1] -= 0
    sz_p_pt[:,1] += 0
    sz_m_pt[:,1] -= 0
    
    sy_p_pt = reverse_affine_transform(sy_p_pt)
    sy_m_pt = reverse_affine_transform(sy_m_pt)
    sz_p_pt = reverse_affine_transform(sz_p_pt)
    sz_m_pt = reverse_affine_transform(sz_m_pt)

    
    ax.plot(sy_p_pt[:,0],sy_p_pt[:,1],sy_p_pt[:,2],color='red')
    ax.plot(sy_m_pt[:,0],sy_m_pt[:,1],sy_m_pt[:,2],color='red')
    ax.plot(sz_p_pt[:,0],sz_p_pt[:,1],sz_p_pt[:,2],color='red')
    ax.plot(sz_m_pt[:,0],sz_m_pt[:,1],sz_m_pt[:,2],color='red',label='Gaussian envelope')
    
    print(sy_p_pt.transpose() - np.asarray([line_xs,line_ys,line_zs]))

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
        If it has extension '.pkl', use the pickle reader. Otherwise, assume TopoJSON.")
    parser.add_argument('-m','--model',dest='model',default=None,choices=['gaussian_tube','gt','gaussian_simple','gs'],
                        help='Model to fit to the samples')

    args = parser.parse_args()

    _, ext = os.path.splitext(args.file)
    if ext == ".pkl":
        reader = sampleParser.SampleReader_pickle(args.file)
    else:
        reader = sampleParser.SampleReader_TopoJSON(args.file)

    samples = np.array(
        [np.array([s.x, s.y, s.z, s.val], dtype=float) for s in reader])

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
