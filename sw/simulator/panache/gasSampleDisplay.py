#!/usr/bin/env python3
import argparse
import typing
import os
import functools
import itertools


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import CenteredNorm
from mpl_toolkits.mplot3d.axes3d import Axes3D

import sampleParser
import gasModel.gasModel as gasModel
import gasModel.wlineModel as wlineModel

import gasModel.optimize as gasOpt



def show_raw_samples(samples: np.array) -> None:

    # Plot acquired data
    xs = samples[:, 0]
    ys = samples[:, 1]
    zs = samples[:, 2]
    vals = samples[:, 3]+1e-9

    fig = plt.figure()
    ax:Axes3D = fig.add_subplot(projection='3d')

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

gaussian_models = ['gaussian','continued_gaussian','log_gaussian']

def show_fitted_samples(samples: np.array,
                        model:typing.Union[typing.Literal['gaussian'],typing.Literal['continued_gaussian'],typing.Literal['log_gaussian'],
                                           typing.Literal['hline']],
                        optimizer:typing.Union[typing.Literal['curve_fit'],typing.Literal['shgo'],typing.Literal['basinhopping']]) -> None:

    # Call approriate model:
    popt = gasOpt.fit_gas_model(model+'_model',optimizer,np.copy(samples),True)
    
    xo=popt[0]
    yo=popt[1]
    zo=popt[2]
    angle=popt[3]
    if model in gaussian_models:
        intensity=popt[4]
        ay=popt[5]
        az=popt[6]
    
    if model == 'gaussian':
        model_fun = functools.partial(gasModel.gaussian_model,
                                      xo=popt[0],
                                      yo=popt[1],
                                      zo=popt[2],
                                      angle=popt[3],
                                      intensity=popt[4],
                                      ay=popt[5],
                                      az=popt[6])
        variables = ['xo','yo','zo','angle','intensity','ay','az']
        units = ['m','m','m','rad','units/m³','m','m']
    elif model == 'continued_gaussian':
        model_fun = functools.partial(gasModel.continued_gaussian_model,
                                      xo=popt[0],
                                      yo=popt[1],
                                      zo=popt[2],
                                      angle=popt[3],
                                      intensity=popt[4],
                                      ay=popt[5],
                                      az=popt[6])
        variables = ['xo','yo','zo','angle','intensity','ay','az']
        units = ['m','m','m','rad','units/m³','m','m']
    elif model == 'log_gaussian':
        model_fun = functools.partial(gasModel.log_gaussian_model,
                                      xo=popt[0],
                                      yo=popt[1],
                                      zo=popt[2],
                                      angle=popt[3],
                                      intensity=popt[4],
                                      ay=popt[5],
                                      az=popt[6])
        variables = ['xo','yo','zo','angle','intensity','ay','az']
        units = ['m','m','m','rad','units/m³','m','m']
    elif model == 'hline':
        model_fun = functools.partial(wlineModel.weighted_dist2_from_hline,
                                      x0=popt[0],
                                      y0=popt[1],
                                      z0=popt[2],
                                      z_angle=popt[3])
        variables = ['xo','yo','zo','angle']
        units = ['m','m','m','rad']
    else:
        raise ValueError(f"Unknown model: {model}")
    
    
    # Print results
    print("Model parameters:")
    for i,t in enumerate(zip(variables,units)):
        v,u = t
        print(f"\t{v} : {popt[i]:.2f} ({u})")
        
    print("----------------------------------------")
    
    # Define relevant affine transforms
    
    to_model_ref = functools.partial(gasModel.referential_transform,
                                        xo=xo,
                                        yo=yo,
                                        zo=zo,
                                        angle=angle)
        
    from_model_ref = functools.partial(gasModel.inverse_referential_transform,
                                        xo=xo,
                                        yo=yo,
                                        zo=zo,
                                        angle=angle)
    
    ### Plot acquired data
    xs = samples[:, 0]
    ys = samples[:, 1]
    zs = samples[:, 2]
    
    # Shift everything by a negligible amount to avoid issues with log scaling
    vals = samples[:, 3] + np.min(samples[np.nonzero(samples[:, 3])])/1000

    fig = plt.figure()
    ax:Axes3D = fig.add_subplot(1,2,1,projection='3d')
    err_ax:Axes3D = fig.add_subplot(1,2,2,projection='3d')

    ## Raw datapoints
    im = ax.scatter(xs, ys, zs, c=vals, marker='8', cmap='magma_r',
                    norm='log', plotnonfinite=True, alpha=0.7)
    cbar = ax.figure.colorbar(im, ax=ax,location='bottom')
    cbar.ax.set_xlabel("Gas concentration (a.u., log scale)",
                       rotation=0, va="top")

    ax.set_xlim(np.min(xs),np.max(xs))
    ax.set_ylim(np.min(ys),np.max(ys))
    ax.set_zlim(np.min(zs),np.max(zs))
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_aspect('equal')

    ## Error datapoints
    if model == 'hline':
        norm_samples = samples.transpose()
        err_vals = model_fun(norm_samples)
        
    elif model == 'log_gaussian':
        err_vals = vals - np.exp(model_fun(np.stack([xs,ys,zs])))
    else:
        err_vals = vals - model_fun(np.stack([xs,ys,zs]))
        
    half_range = np.amax(np.abs(err_vals))
    
    err_im = err_ax.scatter(xs, ys, zs, c=err_vals, marker='8', cmap='bwr',
                    norm=CenteredNorm(0.,half_range),
                    plotnonfinite=True, alpha=0.7)
    cbar = err_ax.figure.colorbar(err_im, ax=err_ax,location='bottom')
    cbar.ax.set_xlabel("Relative error values",
                       rotation=0, va="top")

    err_ax.set_xlim(np.min(xs),np.max(xs))
    err_ax.set_ylim(np.min(ys),np.max(ys))
    err_ax.set_zlim(np.min(zs),np.max(zs))

    err_ax.set_xlabel('X Label')
    err_ax.set_ylabel('Y Label')
    err_ax.set_zlabel('Z Label')
    err_ax.set_aspect('equal')

    ### Model values
    
    ## Model src
    if model in gaussian_models:
        ax.plot([xo],[yo],[zo],'g+',label='Estimated source location')
        err_ax.plot([xo],[yo],[zo],'g+',label='Estimated source location')
    
    ## Model axis
    
    # Gather extremal plotting points
    xmin,xmax = ax.get_xlim()
    ymin,ymax = ax.get_ylim()
    zmin,zmax = ax.get_zlim()
    
    # Put them in the plume referential
    
    line = np.stack([np.array([xmin,ymin,zmin]),np.array([xmax,ymax,zmax])]).transpose()
    line_in_ref = to_model_ref(line)
    
    projected_line_in_ref = np.copy(line_in_ref)
    projected_line_in_ref[1] *= 0.
    projected_line_in_ref[2] *= 0.
    plume_line = from_model_ref(projected_line_in_ref)
    
    ax.plot(plume_line[0],plume_line[1],plume_line[2],'b-',label='Estimated plume axis')
    err_ax.plot(plume_line[0],plume_line[1],plume_line[2],'b-',label='Estimated plume axis')
    
    # Standard deviation plot
    if model in gaussian_models:
        sy = ay * projected_line_in_ref[0] + 1
        sz = az * projected_line_in_ref[0] + 1
            
        projected_line_in_ref_p_sy = np.copy(projected_line_in_ref)
        projected_line_in_ref_m_sy = np.copy(projected_line_in_ref)
            
        projected_line_in_ref_p_sy[1] = projected_line_in_ref_p_sy[1] + sy
        projected_line_in_ref_m_sy[1] = projected_line_in_ref_m_sy[1] - sy 
        
        projected_line_in_ref_p_sz = np.copy(projected_line_in_ref)
        projected_line_in_ref_m_sz = np.copy(projected_line_in_ref)
        
        projected_line_in_ref_p_sz[2] = projected_line_in_ref_p_sz[2] + sz
        projected_line_in_ref_m_sz[2] = projected_line_in_ref_m_sz[2] - sz 
            
        
        p_sy_line = from_model_ref(projected_line_in_ref_p_sy)
        m_sy_line = from_model_ref(projected_line_in_ref_m_sy)
        p_sz_line = from_model_ref(projected_line_in_ref_p_sz)
        m_sz_line = from_model_ref(projected_line_in_ref_m_sz)
        
        # Plot
        ax.plot(p_sy_line[0],p_sy_line[1],p_sy_line[2],color='orange',linestyle='-.')
        err_ax.plot(p_sy_line[0],p_sy_line[1],p_sy_line[2],color='orange',linestyle='-.')
        
        ax.plot(m_sy_line[0],m_sy_line[1],m_sy_line[2],color='orange',linestyle='-.',label='Estimated plume horizontal envelope')
        err_ax.plot(m_sy_line[0],m_sy_line[1],m_sy_line[2],color='orange',linestyle='-.',label='Estimated plume horizontal envelope')
        
        ax.plot(p_sz_line[0],p_sz_line[1],p_sz_line[2],color='red',linestyle='-.')
        err_ax.plot(p_sz_line[0],p_sz_line[1],p_sz_line[2],color='red',linestyle='-.')
        
        ax.plot(m_sz_line[0],m_sz_line[1],m_sz_line[2],color='red',linestyle='-.',label='Estimated plume vertical envelope')
        err_ax.plot(m_sz_line[0],m_sz_line[1],m_sz_line[2],color='red',linestyle='-.',label='Estimated plume vertical envelope')

    ### Finally, show

    ax.legend()
    err_ax.legend()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        prog="Gas Samples displayer"
    )

    parser.add_argument("file", help="File to read gas samples from.\
        If it has extension '.pkl', use the pickle reader. Otherwise, assume TopoJSON.", nargs='+')
    parser.add_argument('-m','--model',dest='model',default=None,choices=['gaussian','g','continued_gaussian','cg','log_gaussian','lg',
                                                                          'hline','hl'],
                        help='Model to fit to the samples')
    parser.add_argument('-o','--optimizer',dest='optimizer',default='curve_fit',choices=['curve_fit','shgo','basinhopping'],
                        help='Optimizer to use for model fitting (the first three are from scipy.optimize). Default to curve_fit.')

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
        
    samples = np.stack(list(samples))

    if args.model is None:
        show_raw_samples(samples)
    elif args.model == 'gaussian' or args.model == 'g':
        show_fitted_samples(samples,'gaussian',args.optimizer)
    elif args.model == 'continued_gaussian' or args.model == 'cg':
        show_fitted_samples(samples,'continued_gaussian',args.optimizer)
    elif args.model == 'log_gaussian' or args.model == 'lg':
        show_fitted_samples(samples,'log_gaussian',args.optimizer)
    elif args.model == 'hline' or args.model == 'hl':
        show_fitted_samples(samples,'hline',args.optimizer)
    else:
        raise ValueError('Unknown model: ' + str(args.model))
        
        


if __name__ == "__main__":
    main()
