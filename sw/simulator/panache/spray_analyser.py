#!/usr/bin/env python3

import typing
import argparse
import shutil

import numpy as np
import matplotlib.pyplot as plt

from scipy.linalg import lstsq
from scipy.optimize import least_squares

from netCDF4 import Dataset

def show_data(xs:np.ndarray,ys:np.ndarray,vals:np.ndarray,estimated_src:np.ndarray,plume_coefs:np.ndarray,delta_direction:float):
    fig,ax = plt.subplots()
    
    plot = ax.scatter(xs,ys,
                      c=vals,norm='log',
                      cmap='inferno_r',
                      s=1.6)
    fig.colorbar(plot)
    
    plume_direction = plume_coefs[0]
    plume_ord_at_0 = plume_coefs[1]
    
    ax.scatter(estimated_src[0],estimated_src[1],marker='x',c='lightgreen',label='Estimated source location')
    ax.plot(xs[0],(xs[0]-estimated_src[0])*plume_direction + estimated_src[1]+plume_ord_at_0,'-g',label='Estimated plume direction')
    
    r_plus = (plume_direction + delta_direction)/(1-plume_direction*delta_direction)
    r_minus = (plume_direction - delta_direction)/(1+plume_direction*delta_direction)
    
    ax.plot(xs[0],(xs[0]-estimated_src[0])*r_plus + estimated_src[1]+plume_ord_at_0,color='g',linestyle='dashed',label='Plume border')
    ax.plot(xs[0],(xs[0]-estimated_src[0])*r_minus + estimated_src[1]+plume_ord_at_0,color='g',linestyle='dashed')
    
    ax.set_xlim(xmin=np.min(xs),xmax=np.max(xs))
    ax.set_ylim(ymin=np.min(ys),ymax=np.max(ys))
    ax.set_aspect('equal')
    ax.set_title('Airborne gas mass per ground surface, log scale (g/m²)')
    ax.legend()
    plt.show()
    
def show_histogram(vals:np.ndarray,bins:int,tol:float):
    flat_vals = vals.flatten()
    plt.hist(flat_vals[flat_vals > tol],bins)
    plt.title(f'Airborne gas concentration distribution (g/m², values higher than {tol})')
    plt.show()
    
def find_middleline_coefs(xs:np.ndarray,ys:np.ndarray,ws:np.ndarray) -> np.ndarray:
    flat_ws = ws.flatten()
    flat_xs = xs.flatten()
    affine_xs = np.stack([flat_xs*flat_ws,np.ones(len(flat_xs))*flat_ws]).T
    weighted_ys = (ws*ys).flatten()
    
    r,_,_,_ = lstsq(affine_xs,weighted_ys)
    
    return r
    
    

def main():
    parser = argparse.ArgumentParser("Spray Analyser",
                                     description="Try to find good parameters for the exploring the gass cone provided")
    
    parser.add_argument("file_in",help="Input .nc file")
    
    args = parser.parse_args()
    
    d = Dataset(args.file_in,'r')
    xs = np.ma.getdata(d.variables['x'][:])
    ys = np.ma.getdata(d.variables['y'][:])
    max_h_pm10 = np.ma.getdata(d.variables['MaxFlatPM10'][:])
    d.close()
    
    meshgrid = np.meshgrid(xs,ys)
    
    
    show_histogram(max_h_pm10,100,0.1)
    
    
    
    high_concentration_indexes = max_h_pm10 > 1
    estimated_src = np.asarray([np.average(meshgrid[0][high_concentration_indexes]),np.average(meshgrid[1][high_concentration_indexes])])
    
    coefs = find_middleline_coefs(meshgrid[0]-estimated_src[0],meshgrid[1]-estimated_src[1],max_h_pm10)
    
    
    
    show_data(meshgrid[0],meshgrid[1],max_h_pm10,estimated_src,coefs,np.arctan(np.pi/8))
    
    return

if __name__ == "__main__":
    main()
