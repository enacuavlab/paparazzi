import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from scipy import stats

import argparse
import typing
import os,re
import itertools
from dataclasses import dataclass
######################################## Misc ########################################

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
    

######################################## Simple plotting ########################################

def error_plot(exp_file:os.PathLike,name:str):
    data:typing.Dict[str,np.ndarray] = np.load(exp_file)
    
    fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
    ax1:Axes
    ax2:Axes
    
    title = name
    
    fig.suptitle(title)
    
    ax1.set_xlabel("Time ellapsed (s)")
    ax1.set_ylabel("Euclidean distance to guiding point (m)")
    ax1.grid(True,'major','x')
    
    ax2.set_xlabel("Time ellapsed (s)")
    ax2.set_ylabel("Virtual coordinate (a.u.)")
    ax2.grid(True,'major','x')
    
    min_time = np.inf
    min_of_max_time = np.inf
    
    for k,v in data.items():
        colors = find_matching_colors(color_dict,k) 
        color = None if len(colors) == 0 else colors[0]
        
        ac_id = re.search(r"[0-9]+",k).group(0)
        
        errors,ws,timestamps = v[0],v[1],v[2]
        ax1.plot(timestamps*1e-3,errors,label=f"AC {ac_id}",
                 color=color)
        ax2.plot(timestamps*1e-3,ws,label=f"AC {ac_id}",
                 color=color)
        print(f"AC {ac_id} color: {color}")
        
        min_time = min(min_time,np.min(timestamps*(1e-3)))
        min_of_max_time = min(min_of_max_time,np.max(timestamps*(1e-3)))
        
    ax1.set_xlim(left=min_time-5,right=min_of_max_time+5)
    ax1.set_ylim(bottom=-1,top=750)
    
    ax2.set_xlim(left=min_time-5)
    ax2.set_ylim(bottom=0)
    
    ax1.legend()
    ax2.legend()
    plt.tight_layout()
    plt.show()

def coord_plot(exp_files:typing.List[os.PathLike],coord_exp_files:typing.List[os.PathLike],name:str):
    data_list:typing.Dict[str,np.ndarray] = [np.load(e) for e in exp_files]
    coord_data_list:typing.Dict[str,np.ndarray] = [np.load(e) for e in coord_exp_files]
    
    fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
    ax1:Axes
    ax2:Axes
    
    title = name
    
    fig.suptitle(title)
    
    ax1.set_xlabel("Time ellapsed (s)")
    ax1.set_ylabel("Euclidean distance to guiding point (m)")
    ax1.grid(True,'major','x')
    
    ax2.set_xlabel("Time ellapsed (s)")
    ax2.set_ylabel("L1 distance to desired coordination (a.u.)")
    ax2.grid(True,'major','x')
    
    min_time = np.inf
    min_of_max_time = np.inf
    
    for exp_data,coord_data in zip(data_list,coord_data_list):
        first_color = None
        ac_ids = []
        for k,v in exp_data.items():
            colors = find_matching_colors(color_dict,k) 
            color = None if len(colors) == 0 else colors[0]
            if first_color is None:
                first_color = color
            
            ac_id = re.search(r"[0-9]+",k).group(0)
            ac_ids.append(ac_id)
            
            errors,timestamps = v[0],v[2]
            ax1.plot(timestamps*1e-3,errors,label=f"AC {ac_id}",
                    color=color)
            
            min_time = min(min_time,np.min(timestamps*(1e-3)))
            min_of_max_time = min(min_of_max_time,np.max(timestamps*(1e-3)))
        for k,v in coord_data.items():
            if "total" in k.lower():
                print(k)
                coord_error,timestamps = v[0],v[1]
                ax2.plot(timestamps*1e-3,coord_error,label=f"AC: {ac_ids}",
                         color=first_color)
            else:
                continue
                
    ax1.set_xlim(left=min_time-5,right=min_of_max_time+5)
    ax1.set_ylim(bottom=-1,top=750)
    
    ax2.set_xlim(left=min_time-5)
    ax2.set_ylim(bottom=0)
    
    ax1.legend()
    ax2.legend()
    plt.tight_layout()
    plt.show()

######################################## Time series comparison ########################################

## Raw value computations

@dataclass
class ErrorAnalysis():
    asympt_min_err:float # Minimal value for Euclidean distance in the asymptotic range (m)
    asympt_max_err:float # Maximum value for Euclidean distance in the asymptotic range (m)
    asympt_mean_err:float # Mean value for Euclidean distance in the asymptotic range (integral approximation) (m)
    asympt_rmse:float # Root Mean Square Error, as the square root of the average integral approximation of the squared error in the asymptotic range (m)
    asympt_min_index:int # Start index of the asymptotic range (included)
    asympt_max_index:int # Stop index of the asymptotic range (excluded, or -1)
    convergence_index:int # Index at which convergence is declared (first time reaching a value below the asymptotic mean error)
    convergence_timestamp:float # Timestamp at the convergence declaration (ms)
    convergence_time:float # Duration before reaching convergence (ms)
    convergence_speed:float # (Delta Err)/(Delta timestamp) from start to convergence (m/s)
    
    @property
    def asympt_minmax_err(self) -> typing.Tuple[float,float]:
        # Shorthand to get the Min and Max for Euclidean distance in the asymptotic range (m)
        return (self.asympt_min_err,self.asympt_max_err)
    
    @property
    def asympt_index_range(self) -> typing.Tuple[int,int]:
        # Shorthand to get the Min and Max indexes for the asymptotic range
        return (self.asympt_min_index,self.asympt_max_index)

def __error_analysis(exp_ac_results:np.ndarray,name:str,asympt_range:typing.Optional[typing.Tuple[int,int]]=None,printing:bool=True) -> ErrorAnalysis:
    
    if printing:
        print(f"/----- Analysis of {name} error -----/\n")
    
    err = exp_ac_results[0]
    timestamps = exp_ac_results[2]
    
    ### Converged section ("asymptotic behavior") ###
    if asympt_range is None:
        samples = np.shape(exp_ac_results)[1]
        asympt_range = (samples//3,-1)
    
    asympt_err = err[asympt_range[0]:asympt_range[1]]
    asympt_ts = timestamps[asympt_range[0]:asympt_range[1]]
    
    # Serie analysis
    asympt_err = np.nan_to_num(asympt_err,nan=np.inf) # Remove NaN, are they are known to be +inf
    asympt_err = np.nan_to_num(asympt_err,posinf=1e20,neginf=-1e20) # Normalize, by replacing +inf by some large value
    
    asympt_int_err = np.trapz(asympt_err,asympt_ts)
    asympt_mean_err = asympt_int_err/(asympt_ts[-1] - asympt_ts[0])
    asympt_stats = stats.describe(asympt_err)
    asympt_rmse = np.sqrt(np.trapz(np.square(asympt_err),asympt_ts)/(asympt_ts[-1] - asympt_ts[0]))
    
    try:
        convergence_declaration = np.where(err < asympt_mean_err)[0][0] #Return a 1D-matrix, which is not an array...
    except:
        convergence_declaration = np.where(err < asympt_mean_err)[0] # Perhaps it is an array...
    if convergence_declaration == 0:
        convergence_declaration += 1

    if printing:
        print(f"Asymptotic error time serie (second half of the dataset):\n\
            - Error range           (m) : {asympt_stats.minmax}\n\
            - Mean integrated error (m) : {asympt_mean_err}\n\
            - Mean error            (m) : {asympt_stats.mean}\n\
            - RMSE                  (m) : {asympt_rmse}\n\n")
        
        
        print(f"Convergence declared at:\n\
            - Convergence Index       : {convergence_declaration}\n\
            - Timestamp           (s) : {(1e-3)*timestamps[convergence_declaration]}\n\
            - Convergence time    (s) : {(1e-3)*(timestamps[convergence_declaration] - timestamps[0])}\n\
            - Convergence speed (m/s) : {1e3*(err[convergence_declaration] - err[0])/(timestamps[convergence_declaration] - timestamps[0])}")
        
        print("\n\n")
    
    output = ErrorAnalysis(
        asympt_min_err=asympt_stats.minmax[0],
        asympt_max_err=asympt_stats.minmax[1],
        asympt_mean_err=asympt_mean_err,
        asympt_rmse=asympt_rmse,
        asympt_min_index=asympt_range[0],
        asympt_max_index=asympt_range[1],
        convergence_index=convergence_declaration,
        convergence_timestamp=timestamps[convergence_declaration],
        convergence_time=timestamps[convergence_declaration] - timestamps[0],
        convergence_speed=1e3*(err[convergence_declaration] - err[0])/(timestamps[convergence_declaration] - timestamps[0])
    )
    
    return output

def print_all_error_analysis(exp_file:os.PathLike,name:str):
    data:typing.Dict[str,np.ndarray] = np.load(exp_file)
    
    for k,v in data.items():
        ac_id = re.search(r"[0-9]+",k).group(0)
        __error_analysis(v,f"Exp {name} : AC {ac_id}")

## Plotting 
def one_summary_error_plot(exp_file:os.PathLike,name:str):
    data:typing.Dict[str,np.ndarray] = np.load(exp_file)
    i = 0
    
    fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
    ax1:Axes
    ax2:Axes
    fig.suptitle(f"Summarized anamysis of {name}")
    
    ax1.set_xlabel("Aircraft ID")
    ax1.set_ylabel("Asymptotic error (m)")
    ax2.set_xlabel("Aircraft ID")
    ax2.set_ylabel("Convergence time (s)")
    
    for k,v in data.items():
        colors = find_matching_colors(color_dict,k) 
        color = None if len(colors) == 0 else colors[0]
        ac_id = re.search(r"[0-9]+",k).group(0)
        err_results = __error_analysis(v,f"Exp {name} : AC {ac_id}")
        ax1.errorbar([i],[err_results.asympt_mean_err],
                     yerr=[[err_results.asympt_mean_err-err_results.asympt_min_err],
                           [err_results.asympt_max_err-err_results.asympt_mean_err]],
                     markersize=15,
                     fmt='_',label=f"AC ID {ac_id}",color=color)
        
        ax1.scatter([i],[err_results.asympt_min_err],marker='v',color=color)
        ax1.scatter([i],[err_results.asympt_max_err],marker='^',color=color)
        
        ax2.errorbar([i],[(1e-3)*err_results.convergence_time],
                     yerr=[[(1e-3)*err_results.convergence_time],[0]],
                     fmt='o',
                     label=f"AC ID {ac_id}",color=color)
        i += 1
        
    ax1.set_xticks(np.asarray(range(0,i)),[k[3:5] for k in data.keys()])
    ax2.set_xticks(np.asarray(range(0,i)),[k[3:5] for k in data.keys()])
    
    ax1.set_ylim(bottom=-1,top=500)
    
    ax2.set_ylim(bottom=0)
    
    ax1.legend()
    ax2.legend()
    plt.tight_layout()
    plt.show()


def all_summary_error_plot(exp_files:typing.List[os.PathLike],names=typing.List[str]):
    fig, (ax1,ax2) = plt.subplots(2,1,sharex=True)
    ax1:Axes
    ax2:Axes
    fig.suptitle(f"Summarized analysis of individual path following benchmarks")
    
    # ax1.set_xlabel("Experiment")
    ax1.set_ylabel("(min,mean,max) asymptotic error (m)")
    ax2.set_xlabel("Experiment")
    ax2.set_ylabel("Convergence time (s)")
    
    max_errors = []
    
    i_offset = 0
    
    first = True
    
    for f,n in zip(exp_files,names):
        data:typing.Dict[str,np.ndarray] = np.load(f)
        
        i=0
        
        for k,v in data.items():
            colors = find_matching_colors(color_dict,k) 
            color = None if len(colors) == 0 else colors[0]
            ac_id = re.search(r"[0-9]+",k).group(0)
            err_results = __error_analysis(v,f"Exp {n} : AC {ac_id}")
            ax1.errorbar([i_offset*10+i],[err_results.asympt_mean_err],
                        yerr=[[err_results.asympt_mean_err-err_results.asympt_min_err],
                            [err_results.asympt_max_err-err_results.asympt_mean_err]],
                        markersize=15,
                        fmt='_',label=f"AC ID {ac_id}" if first else None,color=color)
            
            ax1.scatter([i_offset*10+i],[err_results.asympt_min_err],marker='v',color=color)
            ax1.scatter([i_offset*10+i],[err_results.asympt_max_err],marker='^',color=color)
            
            ax2.errorbar([i_offset*10+i],[(1e-3)*err_results.convergence_time],
                        yerr=[[(1e-3)*err_results.convergence_time],[0]],
                        fmt='o',
                        label=f"AC ID {ac_id}" if first else None,color=color)
            i += 1
            max_errors.append(err_results.asympt_max_err)
            
        i_offset += 1
        first=False
            
    max_errors = np.sort(max_errors)
    
    ax1.set_ylim(bottom=-1,top=500)
    
    ax2.set_ylim(bottom=0)
    
    ax1.set_xticks(np.asarray(range(0,i_offset))*10+(i-1)/2,names)
    ax2.set_xticks(np.asarray(range(0,i_offset))*10+(i-1)/2,names)
    
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')
    plt.tight_layout()
    plt.show()


######################################## Program entry point ########################################

def main():
    
    plt.rc('font', size=13)          # controls default text sizes
    plt.rc('axes', titlesize=13)     # fontsize of the axes title
    plt.rc('axes', labelsize=10)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=10)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=10)    # fontsize of the tick labels
    plt.rc('legend', fontsize=13)    # legend fontsize
    plt.rc('figure', titlesize=25)  # fontsize of the figure title
    
    parser = argparse.ArgumentParser("Bench results plotter")
    parser.add_argument('files',nargs='+',
                        help="List of .npz files to plot")
    
    parser.add_argument('--names',dest='names',nargs='*',default=None,
                        help="List as long as 'bench files'. Precise the names for each plot. By default, it\
                        is left undefined, and the files names are used. If the 'coord' argument is set, only one name is expected")
        
    parser.add_argument("-c","--coord",dest='coord',nargs='*',default=None,
                        help="Associated coordination files to the error files given as main argument. If this list is provided,\
                            it must be as long as the main argument. All values are summarized in one graph.")
    
    args = parser.parse_args()
    
    
    files = [os.path.normpath(f) for f in args.files]
    names = [os.path.basename(f) for f in args.files] if args.names is None else args.names
    
    
    if args.coord is not None:
        assert len(args.coord) == len(files)
        coord_files = [os.path.normpath(f) for f in args.coord]
        coord_plot(files,coord_files,names[0])
    else:
        coord_files = None
        for f,n in zip(files,names):
            error_plot(f,n)
    
        all_summary_error_plot(files,names)

        
    

if __name__ == "__main__":
    main()
