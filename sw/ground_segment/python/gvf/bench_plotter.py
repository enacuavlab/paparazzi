import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

import argparse
import typing
import os
import itertools

# Hardcoded colors recovered from the stdout of the errorLogger.py execution 
color_dict:typing.Dict[str,typing.Tuple[float]] = dict()
color_dict['11'] = (0.9609375, 0.82421875, 0.17578125)
color_dict['12'] = (0.99609375, 0.46875, 0.0)
color_dict['13'] = (0.875, 0.10546875, 0.140625)
color_dict['21'] = (0.20703125, 0.515625, 0.890625)
color_dict['22'] = (0.19921875, 0.81640625, 0.4765625)
color_dict['23'] = (0.59375, 0.4140625, 0.265625)

# Given a dictionnary 'd' indexed by str and a str 's', return all entries 'd[k]' such
# that 'k' is a substring of 's'
def find_matching_colors(color_dict:typing.Dict[str,typing.Tuple[float]],s:str) -> typing.List[typing.Tuple[float]]:
    matching_keys = filter(lambda k : k in s,color_dict.keys())
    return [color_dict[k] for k in matching_keys]
    

def error_plot(exp_file:os.PathLike,name:str):
    data:typing.Dict[str,np.ndarray] = np.load(exp_file)
    
    fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
    ax1:Axes
    ax2:Axes
    
    title = name
    
    fig.suptitle(title)
    
    ax1.set_xlabel("Time ellapsed (s)")
    ax1.set_ylabel("Distance (m)")
    ax1.grid(True,'major','x')
    
    ax2.set_xlabel("Time ellapsed (s)")
    ax2.set_ylabel("Virtual coordinate (a.u.)")
    ax2.grid(True,'major','x')
    
    for k,v in data.items():
        colors = find_matching_colors(color_dict,k) 
        color = None if len(colors) == 0 else colors[0]
        
        ac_id = k[3:5]
        
        errors,ws,timestamps = v[0],v[1],v[2]
        ax1.plot(timestamps*1e-3,errors,label=f"AC {ac_id} euclidean distance to guiding point",
                 color=color)
        ax2.plot(timestamps*1e-3,ws,label=f"AC {ac_id} virtual coordinate",
                 color=color)
        print(f"AC {ac_id} color: {color}")
        
    ax1.legend()
    ax2.legend()
    plt.show()

def scatter_plot(exp_files:typing.List[os.PathLike],names=typing.List[str]):
    for i,f in enumerate(exp_files):
        data = np.load(f)
    return

def main():
    parser = argparse.ArgumentParser("Bench results plotter")
    parser.add_argument('files',nargs='+',
                        help="List of .npz files to plot")
    
    parser.add_argument('--names',dest='names',nargs='*',default=None,
                        help="List as long as 'bench files'. Precise the names for each plot. By default, it\
                        is left undefined, and the files names are used.")
    
    args = parser.parse_args()
    
    
    files = [os.path.normpath(f) for f in args.files]
    names = [os.path.basename(f) for f in args.files] if args.names is None else args.names
    
    for f,n in zip(files,names):
        error_plot(f,n)
    
    


if __name__ == "__main__":
    main()