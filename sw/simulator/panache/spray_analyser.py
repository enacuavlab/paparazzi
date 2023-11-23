#!/usr/bin/env python3

import typing
import argparse

import numpy as np
import matplotlib.pyplot as plt

from scipy.linalg import lstsq
from scipy.optimize import least_squares,curve_fit

from netCDF4 import Dataset

def show_data(xs:np.ndarray,ys:np.ndarray,vals:np.ndarray,estimated_src:np.ndarray,plume_coefs:np.ndarray,delta_direction:float):
    fig,ax = plt.subplots()
    
    min_xs = np.min(xs)
    max_xs = np.max(xs)
    # min_ys = np.min(ys)
    # max_ys = np.max(ys)
    
    minmax_xs = np.asarray([min_xs,max_xs])
    
    plot = ax.scatter(xs,ys,
                      c=vals,norm='log',
                      cmap='inferno_r',
                      s=1.6)
    fig.colorbar(plot)
    
    plume_direction = plume_coefs[0]
    plume_ord_at_0 = plume_coefs[1]
    
    ax.scatter(estimated_src[0],estimated_src[1],marker='x',c='lightgreen',label='Estimated source location')
    ax.plot(minmax_xs,(minmax_xs-estimated_src[0])*plume_direction + estimated_src[1]+plume_ord_at_0,'-g',label='Estimated plume direction')
    
    r_plus = (plume_direction + delta_direction)/(1-plume_direction*delta_direction)
    r_minus = (plume_direction - delta_direction)/(1+plume_direction*delta_direction)
    
    ax.plot(minmax_xs,(minmax_xs-estimated_src[0])*r_plus + estimated_src[1]+plume_ord_at_0,color='g',linestyle='dashed',label='Plume border')
    ax.plot(minmax_xs,(minmax_xs-estimated_src[0])*r_minus + estimated_src[1]+plume_ord_at_0,color='g',linestyle='dashed')
    
    ax.set_xlim(xmin=np.min(xs),xmax=np.max(xs))
    ax.set_ylim(ymin=np.min(ys),ymax=np.max(ys))
    ax.set_aspect('equal')
    ax.set_title('Airborne gas mass per ground surface, log scale (g/m²)')
    ax.legend()
    plt.show()
    
def show_histogram(vals:np.ndarray,bins:int,tol:float,title:str):
    """Plot an histogram of the gas samples

    Args:
        vals (np.ndarray): N-D array of floats: gas concentration
        bins (int): Number of bins for the histogram
        tol (float): Truncation, remove all datapoints smaller than tol
        title (str): Name for the plot
    """
    flat_vals = vals.flatten()
    plt.hist(flat_vals[flat_vals > tol],bins)
    plt.title(title)
    plt.show()
    
def find_middleline_coefs(xs:np.ndarray,ys:np.ndarray,ws:np.ndarray) -> np.ndarray:
    """Given positions and weights, find (r,b) such that y = r*x + b 

    This is formulated as the following least squares problem:
        min{sum_i{(w_i*(y_i - r*x_i - b))²}}

    Args:
        xs (np.ndarray): Points abscissa
        ys (np.ndarray): Points ordinates
        ws (np.ndarray): Points weights

    Returns:
        np.ndarray: (r,b) the growth coefficient and ordinate to origin of the linear regression
    """
    affine_xs = np.stack([xs*ws,np.ones(len(xs))*ws]).T
    weighted_ys = (ws*ys)
    
    r,_,_,_ = lstsq(affine_xs,weighted_ys)
    
    return r

def find_closest(xs:np.ndarray,ys:np.ndarray, line_coefs:np.ndarray,tol:float) -> np.ndarray:
    r = line_coefs[0]
    b = line_coefs[1]
    
    #normal_vec = np.asarray([-r,1])
    distances = np.abs(-r*xs + (ys-b))*np.sqrt(r*r+1)
    
    return distances > tol 

def intensity_loss_analysis(datapoints:np.ndarray) -> float:
    return 1

def gaussian_interpolation(xs:np.ndarray,ys:np.ndarray,
                           default_values:typing.Tuple[float,float]=(0,1),
                           plotting:bool=False) -> np.ndarray:
    
    max_ys = np.max(ys)
    w_barycenter = np.average(xs,weights=ys)
    
    def my_scaled_gauss(x,c,s):
        return max_ys*np.exp(-np.square((x-c)/s)/2)
    
    def my_scaled_gauss_jac(x,c,s):
        return np.asarray([
                max_ys*(x-c)/(np.square(s))*np.exp(-np.square((x-c)/s)/2),
                max_ys/(4*s*s*s)*np.exp(-np.square((x-c)/s)/2)]).T
    
    val,_ = curve_fit(my_scaled_gauss,xs,ys,
                      p0=(w_barycenter,1),
                      jac=my_scaled_gauss_jac,
                      method='trf')
    
    if plotting:
        print(f"Center, dispersion: {val}")
        plt.plot(xs,ys,'x k',label=f"Datapoints")
        plt.plot(np.sort(xs),my_scaled_gauss(np.sort(xs),val[0],val[1]),'-b',label="Interpolated gaussian")
        plt.plot([w_barycenter,w_barycenter],[0,max_ys],'-r',label=f'Weighted X barycenter (avg-gaussian_mean = {w_barycenter-val[0]:.3f})')
        plt.plot([val[0]-val[1],val[0]+val[1]],my_scaled_gauss([val[0]-val[1],val[0]+val[1]],val[0],val[1]),'|-g',label="One standard deviation away from mean")
        plt.legend()
        plt.show()
        
    return val[0],val[1]
    

def main():
    parser = argparse.ArgumentParser("Spray Analyser",
                                     description="Try to find good parameters for the exploring the gass cone provided")
    
    parser.add_argument("file_in",help="Input .nc file")
    
    args = parser.parse_args()
    
    ### Load dataset from .nc file
    
    d = Dataset(args.file_in,'r')
    xs = np.ma.getdata(d.variables['x'][:])
    ys = np.ma.getdata(d.variables['y'][:])
    max_h_pm10 = np.ma.getdata(d.variables['MaxFlatPM10'][:])
    d.close()
    
    ### Flatten the data into a point list
    
    meshgrid = np.meshgrid(xs,ys)
    
    xs_flat = meshgrid[0].flatten()
    ys_flat = meshgrid[1].flatten()
    ws_list = max_h_pm10.flatten()
    
    ### Plot the gas concentration repartition
    
    show_histogram(ws_list,100,0.1,f'Airborne gas concentration distribution (g/m², values higher than 0.1)')
    
    ### Estimate the source location (highest concentration point)
    
    high_concentration_indexes = ws_list > 1
    estimated_src = np.asarray([np.average(xs_flat[high_concentration_indexes]),np.average(ys_flat[high_concentration_indexes])])
    print(f"Estimated source location:\n\t{estimated_src}")
    
    ### Estimate main axis ('middle line' of the points)  
    
    coefs = find_middleline_coefs(xs_flat-estimated_src[0],ys_flat-estimated_src[1],ws_list)
    print(f"Center line (m,b) coefficients (y = m*x+b):\n\t{coefs}")
    
    delta_line_coeff = np.arctan(np.pi/8)
    show_data(xs_flat,ys_flat,ws_list,estimated_src,coefs,delta_line_coeff)
    print(f"Delta line coef for cone: {delta_line_coeff}")
    
    ### Change of referential: use the middle line as x-axis, y-axis becomes distance to line
    
    line_d_vector = np.asarray([1,coefs[0]])
    line_d_vector = line_d_vector/np.linalg.norm(line_d_vector)
    line_n_vector = np.asarray([-line_d_vector[1],line_d_vector[0]])
    
    to_line_ref_matrix = np.stack([line_d_vector,line_n_vector])
    
    ds_ts_list = to_line_ref_matrix @ np.stack([xs_flat-estimated_src[0],ys_flat-estimated_src[1] - coefs[1]])
    
    ### Focus on points close to the line to study gas linear spread coefficient (how much the concentration drops along the line)
    
    # #show_histogram(np.abs(ts_ds_list[0]),100,0,"Distance to centerline")
    # close_indices = np.logical_and(np.abs(ds_ts_list[1]) < 50,ds_ts_list[0] > 100)
    # close_xs = xs_flat[close_indices]
    # close_ys = ys_flat[close_indices]
    # close_ds = ds_ts_list[0][close_indices]
    # close_ws = ws_list[close_indices]
    # close_log_ws = np.log10(close_ws[close_ws>0])
    # close_ds = close_ds[close_ws>0]
    
    # close_log_ws = close_log_ws[close_ds>2000]
    # close_ds = close_ds[close_ds>2000]
    
    # # show_data(close_xs,close_ys,close_ws,estimated_src,coefs,np.arctan(np.pi/8))
    
    # minmax_ds = np.asarray([np.min(close_ds),np.max(close_ds)])
    # line_gas_dispersion_coef,_,_,_ = lstsq(np.stack([close_ds,np.ones(len(close_ds))]).T,close_log_ws)
    
    # plt.plot(close_ds,close_log_ws,'x k',label="Log10 Gas concentration of measurements close to the middle line")
    # plt.plot(minmax_ds,minmax_ds*line_gas_dispersion_coef[0]+line_gas_dispersion_coef[1],'-b',label="Interpolated linear coefficient")
    # plt.xlabel("Linear abscissa (m)")
    # plt.ylabel("Log10 Gas concentration (g/m²)")
    # plt.legend()
    # plt.show()
    
    ### Plot all points at a given linear abscissa
    deviation_from_middle = []
    ts_sampling_space = np.linspace(1000,7000,100)
    for target in ts_sampling_space:
        
        target_tol = 10
        target_indices = np.abs(ds_ts_list[0] - target) < target_tol
        target_xs = xs_flat[target_indices]
        target_ys = ys_flat[target_indices]
        target_ws = ws_list[target_indices]
        target_ts = ds_ts_list[1][target_indices]
        
        mean,std = gaussian_interpolation(target_ts,target_ws,plotting=False)
        deviation_from_middle.append(mean)
        
    plt.scatter(ts_sampling_space,np.asarray(deviation_from_middle),label="Distance to ref")
    plt.hlines(0,np.min(ts_sampling_space),np.max(ts_sampling_space),label="Current middle axis")
    plt.legend()
    plt.show()
        
    
    
    
    
    return

if __name__ == "__main__":
    main()
