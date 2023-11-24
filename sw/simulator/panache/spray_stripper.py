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

def main():
    parser = argparse.ArgumentParser("Spray Stripper",
                                     description="Strip a PANACHE Spray simulation .nc file from its data, keeping only one "\
                                         "snapshot (the one with the largest total gas concentration), and concentration summed over the vertical coordinate 'Z'."\
                                        "The program can output the resulting file or display it using Matplotlib (no background provided).")
    
    parser.add_argument("file_in",help="Input .nc file")
    parser.add_argument("-l","--log",dest='log',action='store_true',help="Save the log of the concentration instead of flat concentration")
    parser.add_argument("-p","--print",dest='print',action='store_true',help="Display the stripped data using Matplotlib")
    parser.add_argument("-o","--out",dest='out',help="Name of the file where to store the stripped data")
    
    args = parser.parse_args()
    
    d = Dataset(args.file_in,'r')
    
    all_h_pm10 = np.sum(d.variables['PM10'],axis=1)
    max_hour = np.argmax(np.sum(all_h_pm10,axis=(1,2)))
    print(f"Time index with maximal concentration: {max_hour}")
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