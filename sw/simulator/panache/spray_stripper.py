#!/usr/bin/env python3

import typing
import argparse
import shutil

import numpy as np
import matplotlib.pyplot as plt

from netCDF4 import Dataset

def add_to_file(d_file:str,max_pm10:np.ndarray):
    d = Dataset(d_file,'a')
    
    var_maxPM10 = d.createVariable('MaxFlatPM10','f4',('y','x',))
    var_maxLogPM10 = d.createVariable('MaxFlatLogPM10','f4',('y','x',))
    
    var_maxPM10[:] = max_pm10
    var_maxLogPM10[:] = np.clip(np.log10(max_pm10),a_min=-1000,a_max=None)
    
    d.close()
    
def show_file(max_pm10:np.ndarray,scale:str='log'):

    fig,ax = plt.subplots()
        
    cs = ax.imshow(max_pm10,
                   norm=scale,
                   aspect='equal',
                   origin='lower',)
    fig.colorbar(cs)
        
    ax.set_title(f'Maximal instantaneous PM10 {scale} ground concentration recorded after the incident, g/m²')
        
    plt.show()
    return

def plot_3d(d_file:str,cutoff:float=1e-3):
    d = Dataset(d_file,'r')
    xs = d.variables['x'][:]
    ys = d.variables['y'][:]
    zs = d.variables['Z'][:]
    
    _,m_ys,m_xs = np.meshgrid(np.linspace(0,4000,np.shape(zs)[0]),ys,xs)
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    fl_xs = m_xs.flatten()
    fl_ys = m_ys.flatten()
    fl_zs = zs.flatten()
    fl_ws = d.variables['PM10'][6].flatten()
    
    wanted = fl_ws > cutoff
    
    cs = ax.scatter(fl_xs[wanted],fl_ys[wanted],fl_zs[wanted],c=fl_ws[wanted],
               norm='log',label='')
    
    
    ax.set_aspect('equal')
    ax.set_xlabel
    
    fig.colorbar(cs)
    plt.show()

def main():
    parser = argparse.ArgumentParser("Spray Stripper",
                                     description="Strip a PANACHE Spray simulation .nc file from its data, keeping only one "\
                                         "snapshot (the one with the largest total gas concentration), and concentration summed over the vertical coordinate 'Z'."\
                                        "The program can output the resulting file or display it using Matplotlib (no background provided).")
    
    parser.add_argument("file_in",help="Input .nc file")
    parser.add_argument("-l","--log",dest='log',action='store_true',help="Save the log of the concentration instead of flat concentration")
    parser.add_argument("-p","--print",dest='print',action='store_true',help="Display the stripped data using Matplotlib")
    parser.add_argument("-o","--out",dest='out',help="Name of the file where to store the stripped data")
    parser.add_argument("-3d",dest='d3',action='store_true',help="If this flag is set, show a 3D scatter plot then close.")
    
    args = parser.parse_args()
    
    if args.d3:
        plot_3d(args.file_in,1e-3)
    
    d = Dataset(args.file_in,'r')
    
    all_h_pm10 = np.sum(d.variables['PM10'],axis=1)
    max_hour = np.argmax(np.sum(all_h_pm10,axis=(1,2)))
    max_h_pm10 = all_h_pm10[max_hour]
    d.close()
    if args.print:
        show_file(max_h_pm10,'log' if args.log else 'linear')
    else:
        shutil.copyfile(args.file_in,args.out)
        add_to_file(args.out,max_h_pm10)
    
    return

if __name__ == "__main__":
    main()