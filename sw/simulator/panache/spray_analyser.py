#!/usr/bin/env python3

import typing
import argparse
import pathlib,itertools

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from scipy.linalg import lstsq
from scipy.optimize import least_squares,curve_fit

from netCDF4 import Dataset
import sampleParser

########## Nice legend handler for lines ##########

# Found at https://stackoverflow.com/questions/31544489/two-line-styles-in-legend
from matplotlib.legend_handler import HandlerTuple

class HandlerTupleVertical(HandlerTuple):
    def __init__(self, **kwargs):
        HandlerTuple.__init__(self, **kwargs)

    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize, trans):
        # How many lines are there.
        numlines = len(orig_handle)
        handler_map = legend.get_legend_handler_map()

        # divide the vertical space where the lines will go
        # into equal parts based on the number of lines
        height_y = (height / numlines)

        leglines = []
        for i, handle in enumerate(orig_handle):
            handler = legend.get_legend_handler(handler_map, handle)

            legline = handler.create_artists(legend, handle,
                                             xdescent,
                                             (2*i + 1)*height_y,
                                             width,
                                             2*height,
                                             fontsize, trans)
            leglines.extend(legline)

        return leglines


#################### Data extraction ####################

def extract_2d_stripped_sample_lists(d_file:str) -> typing.Tuple[np.ndarray,np.ndarray,np.ndarray]:
    path_obj = pathlib.Path(d_file)
    if path_obj.suffix == '.nc':
        d = Dataset(d_file,'r')
        xs = np.ma.getdata(d.variables['x'][:])
        ys = np.ma.getdata(d.variables['y'][:])
        max_h_pm10 = np.ma.getdata(d.variables['MaxFlatPM10'][:])
        
        d.close()
        
        meshgrid = np.meshgrid(xs,ys)
    
        return meshgrid[0],meshgrid[1],max_h_pm10
    
    if path_obj.suffix == ".json":
        reader = sampleParser.SampleReader_TopoJSON(path_obj)
    elif path_obj.suffix == ".pkl":
        reader = sampleParser.SampleReader_pickle(path_obj)
    
    samples = iter([])
    samples = itertools.chain(samples,(np.array([s.x, s.y, s.z, s.val], dtype=float) for s in reader))
    samples = np.stack(list(samples))
        
    return samples.T[0],samples.T[1],samples.T[3]
    
    

def extract_3d_sample_lists(d_file:str,time:int=2) -> typing.Tuple[np.ndarray,np.ndarray,np.ndarray,np.ndarray]:
    path_obj = pathlib.Path(d_file)
    if path_obj.suffix == '.nc':
        d = Dataset(d_file,'r')
        xs = d.variables['x'][:]
        ys = d.variables['y'][:]
        zs = d.variables['Z'][:,0,0]
        
        _,m_ys,m_xs = np.meshgrid(zs,ys,xs)
        m_zs = np.ma.getdata(d.variables['Z'][:])
        vals = np.ma.getdata(d.variables['PM10'][time])
        d.close()
        
        return np.ma.getdata(m_xs),np.ma.getdata(m_ys),m_zs,vals
    
    if path_obj.suffix == ".json":
        reader = sampleParser.SampleReader_TopoJSON(path_obj)
    elif path_obj.suffix == ".pkl":
        reader = sampleParser.SampleReader_pickle(path_obj)
    
    samples = iter([])
    samples = itertools.chain(samples,(np.array([s.x, s.y, s.z, s.val], dtype=float) for s in reader))
    samples = np.stack(list(samples))
    
    return samples.T[0],samples.T[1],samples.T[2],samples.T[3]

#################### Data analysis ####################

def simple_src_estimator(xs:np.ndarray,ys:np.ndarray,ws:np.ndarray,tol:float=0.99) -> np.ndarray:
    high_concentration_indexes = ws > np.max(ws)*tol
    estimated_src = np.asarray([np.average(xs[high_concentration_indexes]),np.average(ys[high_concentration_indexes])])
    return estimated_src
    
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
    mean_estimated = np.average(xs,weights=ys)
    
    def sigma_estimator(xxs:np.ndarray,yys:np.ndarray, tol:float=1)->float:
        A = np.max(yys)
        mu = np.average(xxs,weights=yys)
        candidates = xxs[np.abs(yys-A*np.exp(-0.5)) < tol*np.std(yys)]
        sigma = np.average(np.abs(candidates-mu))
        return sigma
    
    sigma_estimated = np.nan
    tol = 1.
    tol_step = 0.01
    while not(np.isfinite(sigma_estimated)):
        sigma_estimated = sigma_estimator(xs,ys,tol)
        tol += tol_step
    
    def my_scaled_gauss(x,c,s):
        return max_ys*np.exp(-np.square((x-c)/s)/2)
    
    def my_scaled_gauss_jac(x,c,s):
        return np.asarray([
                max_ys*(x-c)/(np.square(s))*np.exp(-np.square((x-c)/s)/2),
                max_ys/(4*s*s*s)*np.exp(-np.square((x-c)/s)/2)]).T
    
    try:
        val,_ = curve_fit(my_scaled_gauss,xs,ys,
                        p0=(mean_estimated,sigma_estimated),
                        jac=my_scaled_gauss_jac,
                        method='trf')
    except ValueError as e:
        print(f"Initial (mean,std) guesses: ({mean_estimated},{sigma_estimated})")
        raise e
    
    if plotting:
        print(f"Center, dispersion: {val}")
        plt.plot(xs,ys,'x k',label=f"Datapoints")
        plt.plot(np.sort(xs),my_scaled_gauss(np.sort(xs),val[0],val[1]),'-b',label="Interpolated gaussian")
        plt.plot([mean_estimated,mean_estimated],[0,max_ys],'--r',label=f'Weighted X barycenter (avg-gaussian_mean = {mean_estimated-val[0]:.3f})')
        plt.plot([val[0],val[0]],[0,max_ys],'-r',label=f'Gaussian mean  (avg-gaussian_mean = {mean_estimated-val[0]:.3f})')
        plt.plot([val[0]-val[1],val[0]+val[1]],my_scaled_gauss([val[0]-val[1],val[0]+val[1]],val[0],val[1]),'|-g',label="One standard deviation away from mean")
        plt.legend()
        plt.show()
        
    return val[0],val[1]

def successive_gaussians(ds:np.datetime64,ts:np.ndarray, ws:np.ndarray, lin_abs_tol:float,sample_points:np.ndarray,show:bool=False) -> np.ndarray:
    deviation_from_middle = []
    gaussians_std = []
    for target in sample_points:
        target_indices = np.abs(ds - target) < lin_abs_tol
        target_ws = ws[target_indices]
        target_ts = ts[target_indices]
        
        mean,std = gaussian_interpolation(target_ts,target_ws,plotting=False)
        deviation_from_middle.append(mean)
        gaussians_std.append(std)
    
    if show:
        plt.scatter(sample_points,np.asarray(deviation_from_middle),label="Distance to ref")
        plt.hlines(0,np.min(sample_points),np.max(sample_points),label="Current middle axis")
        plt.title("Distances between middle line and estimated gaussians distributions center points")
        plt.xlabel("Linear abscissa (re-oriented lon-lat)")
        plt.ylabel("Orthogonal distance (re-oriented lon-lat)")
        plt.legend()
        plt.show()
    
    return np.stack([sample_points,np.asarray(deviation_from_middle),np.asarray(gaussians_std)])

def linear_bspline_interpolation(xs:np.ndarray,ys:np.ndarray,stds:np.ndarray, tol:float=1., show:bool=False):
    from scipy.interpolate import UnivariateSpline
    
    spline = UnivariateSpline(xs,ys,w=1/stds,k=1,s=tol*len(xs))
    
    if show:
        plt.scatter(xs,ys,marker='x',color='k',label='Datapoints')
        plt.plot(xs,spline(xs),'-b',label='Resulting Linear spline')
        plt.title("Regression using linear spline")
        plt.legend()
        plt.show()
    
    return spline
    

#################### Plotting ####################

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
    
def show_data_comp(xs1:np.ndarray,ys1:np.ndarray,vals1:np.ndarray,
                   xs2:np.ndarray,ys2:np.ndarray,vals2:np.ndarray,
                   delta_direction:float):
    fig,(ax1,ax2) = plt.subplots(1,2,sharex=True,sharey=True)
    ax1:Axes
    ax2:Axes
    
    
    minmax_xs = np.asarray([min(np.min(xs1),np.min(xs2)),max(np.max(xs1),np.max(xs2))])
    minmax_ys = np.asanyarray([min(np.min(ys1),np.min(ys2)),max(np.max(ys1),np.max(ys2))])
    
    plot1 = ax1.scatter(xs1,ys1,
                      c=vals1,norm='log',
                      cmap='inferno_r',
                      s=1.6)
    fig.colorbar(plot1)
    
    plot2 = ax2.scatter(xs2,ys2,
                      c=vals2,norm='log',
                      cmap='inferno_r',
                      s=1.6)
    
    fig.colorbar(plot2)
    
    estimated_src1 = simple_src_estimator(xs1,ys1,vals1)
    estimated_src2 = simple_src_estimator(xs2,ys2,vals2)
    coefs1 = find_middleline_coefs(xs1-estimated_src1[0],ys1-estimated_src1[1],vals1)
    coefs2 = find_middleline_coefs(xs2-estimated_src2[0],ys2-estimated_src2[1],vals2)
    
    
    src11 = ax1.scatter(estimated_src1[0],estimated_src1[1],marker='x',c='lightgreen')#,label='Estimated source location 1')
    line11, = ax1.plot(minmax_xs,(minmax_xs-estimated_src1[0])*coefs1[0] + estimated_src1[1]+coefs1[1],'-g')#,label='Estimated plume direction 1')
    
    src12 = ax1.scatter(estimated_src2[0],estimated_src2[1],marker='+',c='skyblue')#,label='Estimated source location 2')
    line12, = ax1.plot(minmax_xs,(minmax_xs-estimated_src2[0])*coefs2[0] + estimated_src2[1]+coefs2[1],'--',color='deepskyblue')#,label='Estimated plume direction 2')
    
    src21 = ax2.scatter(estimated_src1[0],estimated_src1[1],marker='x',c='lightgreen')#,label='Estimated source location 1')
    line21, = ax2.plot(minmax_xs,(minmax_xs-estimated_src1[0])*coefs1[0] + estimated_src1[1]+coefs1[1],'--g')#,label='Estimated plume direction 1')
    
    src22 = ax2.scatter(estimated_src2[0],estimated_src2[1],marker='+',c='skyblue')#,label='Estimated source location 2')
    line22, = ax2.plot(minmax_xs,(minmax_xs-estimated_src2[0])*coefs2[0] + estimated_src2[1]+coefs2[1],'-',color='deepskyblue')#,label='Estimated plume direction 2')
    
    
    r1_plus = (coefs1[0] + delta_direction)/(1-coefs1[0]*delta_direction)
    r1_minus = (coefs1[0] - delta_direction)/(1+coefs1[0]*delta_direction)
    
    r2_plus = (coefs2[0] + delta_direction)/(1-coefs2[0]*delta_direction)
    r2_minus = (coefs2[0] - delta_direction)/(1+coefs2[0]*delta_direction)
    
    border11, = ax1.plot(minmax_xs,(minmax_xs-estimated_src1[0])*r1_plus + estimated_src1[1]+coefs1[1],color='g',linestyle='dashed')#,label='Border 1')
    ax1.plot(minmax_xs,(minmax_xs-estimated_src1[0])*r1_minus + estimated_src1[1]+coefs1[1],color='g',linestyle='dashed')
    
    border12, = ax1.plot(minmax_xs,(minmax_xs-estimated_src2[0])*r2_plus + estimated_src2[1]+coefs2[1],color='deepskyblue',linestyle='dotted')#,label='Border 2')
    ax1.plot(minmax_xs,(minmax_xs-estimated_src2[0])*r2_minus + estimated_src2[1]+coefs2[1],color='deepskyblue',linestyle='dotted')
    
    border21, = ax2.plot(minmax_xs,(minmax_xs-estimated_src1[0])*r1_plus + estimated_src1[1]+coefs1[1],color='g',linestyle='dotted')#,label='Border 1')
    ax2.plot(minmax_xs,(minmax_xs-estimated_src1[0])*r1_minus + estimated_src1[1]+coefs1[1],color='g',linestyle='dotted')
    
    border22, = ax2.plot(minmax_xs,(minmax_xs-estimated_src2[0])*r2_plus + estimated_src2[1]+coefs2[1],color='deepskyblue',linestyle='dashed')#,label='Border 2')
    ax2.plot(minmax_xs,(minmax_xs-estimated_src2[0])*r2_minus + estimated_src2[1]+coefs2[1],color='deepskyblue',linestyle='dashed')
    
    ax1.set_xlim(xmin=minmax_xs[0],xmax=minmax_xs[1])
    ax1.set_ylim(ymin=minmax_ys[0],ymax=minmax_ys[1])
    
    ax2.set_xlim(xmin=minmax_xs[0],xmax=minmax_xs[1])
    ax2.set_ylim(ymin=minmax_ys[0],ymax=minmax_ys[1])
    
    ax1.set_aspect('equal')
    ax1.set_title('1 : Airborne gas mass per ground surface, log scale (g/m²)')
    
    ax2.set_aspect('equal')
    ax2.set_title('2 : Airborne gas mass per ground surface, log scale (g/m²)')
    
    ax1.legend([src11,src12,(line11,line12),(border11,border12)],
               ["Source 1","Source 2","Middleline 1,2","Borders 1,2"],
               handler_map={tuple : HandlerTupleVertical(ndivide=None)})
    
    ax2.legend([src21,src22,(line21,line22),(border21,border22)],
               ["Source 1","Source 2","Middleline 1,2","Borders 1,2"],
               handler_map={tuple : HandlerTupleVertical(ndivide=None)})
    plt.show()
    
def show_refit_centerline(xs:np.ndarray,ys:np.ndarray,vals:np.ndarray,estimated_src:np.ndarray,plume_coefs:np.ndarray,new_centerline:np.ndarray):
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
    ax.plot(minmax_xs,(minmax_xs-estimated_src[0])*plume_direction + estimated_src[1]+plume_ord_at_0,':g',label='Estimated plume direction')
    
    ax.plot(new_centerline[0],new_centerline[1],'-g',label='Refit plume centerline')
    
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

def plot_3d(xs:np.ndarray,ys:np.ndarray,zs:np.ndarray,ws:np.ndarray,estimated_src:np.ndarray,plume_coefs:np.ndarray,cutoff:float=1e-3):
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ws += np.min(ws[ws > 0])/1000
    
    wanted = ws > cutoff
    
    cs = ax.scatter(xs[wanted],ys[wanted],zs[wanted],c=ws[wanted],
                    cmap='inferno_r',
               norm='log',label='PM10 plume' + f', points with concentration above {cutoff:.2E} g/m³' if cutoff > 0 else '')
    
    
    min_xs = np.min(xs)
    max_xs = np.max(xs)
    # min_ys = np.min(ys)
    # max_ys = np.max(ys)
    
    minmax_xs = np.asarray([min_xs,max_xs])
    
    plume_direction = plume_coefs[0]
    plume_ord_at_0 = plume_coefs[1]
    
    ax.scatter(estimated_src[0],estimated_src[1],0.,marker='x',c='lightgreen',label='Estimated source location')
    ax.plot(minmax_xs,(minmax_xs-estimated_src[0])*plume_direction + estimated_src[1]+plume_ord_at_0,'-g',label='Estimated plume direction')
    
    ax.set_aspect('equal')
    ax.set_xlabel('Lambert93 lon')
    ax.set_ylabel('Lambert93 lat')
    ax.set_zlabel('Altitude (m)')
    
    
    fig.colorbar(cs,pad=0.2).ax.set_ylabel("PM10 concentration (g/m³)",rotation=-90,va="bottom")
    plt.legend()
    plt.tight_layout()
    plt.show()

#################### Core programs ####################

##### Standalone analysis #####

def analysis_3d(file:str,tol:float,hour:typing.Optional[str]):
    if hour is None and pathlib.Path(file).suffix == '.nc':
        d = Dataset(file,'r')
    
        all_h_pm10 = np.sum(d.variables['PM10'],axis=1)
        max_hour = np.argmax(np.sum(all_h_pm10,axis=(1,2)))
        print(f"Time index with maximal concentration: {max_hour}")
        max_h_pm10 = all_h_pm10[max_hour]
        d.close()
        
        hour = max_hour
    else:
        hour = int(hour) if hour != None else None
        
    xs,ys,zs,ws = extract_3d_sample_lists(file,hour)
    
    flat_xs = xs.flatten()
    flat_ys = ys.flatten()
    flat_zs = zs.flatten()
    flat_ws = ws.flatten()
    
    ### Estimate the source location (highest concentration point)
    
    estimated_src = simple_src_estimator(flat_xs,flat_ys,flat_ws)
    print(f"Estimated source location:\n\t{estimated_src}")
    
    ### Estimate main axis ('middle line' of the points)  
    
    coefs = find_middleline_coefs(flat_xs-estimated_src[0],flat_ys-estimated_src[1],flat_ws)
    print(f"Center line (m,b) coefficients (y = m*x+b):\n\t{coefs}")
    
    plot_3d(flat_xs,flat_ys,flat_zs,flat_ws,estimated_src,coefs,tol)
    
    ### Weighted average altitude
    
    significant_gas = ws > tol
    
    avg_zs = np.sum(zs*significant_gas,axis=0) * np.nan_to_num(1/np.count_nonzero(significant_gas,axis=0),posinf=0,neginf=0)
    avg_zs = avg_zs.flatten()
    if len(np.shape(ys)) > 1:
        ys_2d = ys[:,0,:].flatten()
    else:
        ys_2d = ys
        
    if len(np.shape(xs)) > 1:
        xs_2d = xs[:,0,:].flatten()
    else:
        xs_2d = xs
    
    above_ground = avg_zs > 10
    
    fig,ax = plt.subplots()
    
    cs = ax.scatter(xs_2d[above_ground],ys_2d[above_ground],c=avg_zs[above_ground])
    fig.colorbar(cs).ax.set_ylabel("Average altitude (m)",rotation=-90,va="bottom")
    ax.set_xlabel('Lambert93 lon')
    ax.set_ylabel('Lambert93 lat')
    plt.show()
    
    ### Change of referential: use the middle line as x-axis, y-axis becomes distance to line
    
    line_d_vector = np.asarray([1,coefs[0]])
    line_d_vector = line_d_vector/np.linalg.norm(line_d_vector)
    line_n_vector = np.asarray([-line_d_vector[1],line_d_vector[0]])
    
    to_line_ref_matrix = np.stack([line_d_vector,line_n_vector])
    
    ds_ts_list = to_line_ref_matrix @ np.stack([flat_xs-estimated_src[0],flat_ys-estimated_src[1] - coefs[1]])
    
    return
    

def analysis_2d(file:str):
    
    ### Load dataset from  file
    
    xs,ys,ws = extract_2d_stripped_sample_lists(file)
    xs_flat = xs.flatten()
    ys_flat = ys.flatten()
    ws_list = ws.flatten()
        
    # ### Plot the gas concentration repartition
    
    # show_histogram(ws_list,100,0.1,f'Airborne gas concentration distribution (g/m², values higher than 0.1)')
    
    ### Estimate the source location (highest concentration point)
    estimated_src = simple_src_estimator(xs_flat,ys_flat,ws_list)
    print(f"Estimated source location:\n\t{estimated_src}")
    
    ### Estimate main axis ('middle line' of the points)  
    
    coefs = find_middleline_coefs(xs_flat-estimated_src[0],ys_flat-estimated_src[1],ws_list)
    print(f"Center line (m,b) coefficients (y = m*x+b):\n\t{coefs}")
    
    delta_line_coeff = np.arctan(np.pi/8)
    # show_data(xs_flat,ys_flat,ws_list,estimated_src,coefs,delta_line_coeff)
    print(f"Delta line coef for cone: {delta_line_coeff}")
    
    ### Change of referential: use the middle line as x-axis, y-axis becomes distance to line
    
    line_d_vector = np.asarray([1,coefs[0]])
    line_d_vector = line_d_vector/np.linalg.norm(line_d_vector)
    line_n_vector = np.asarray([-line_d_vector[1],line_d_vector[0]])
    
    to_line_ref_matrix = np.stack([line_d_vector,line_n_vector])
    
    ds_ts_list = to_line_ref_matrix @ np.stack([xs_flat-estimated_src[0],ys_flat-estimated_src[1] - coefs[1]])
    
    
    ### Fit gaussian at regular linear abscissa to study how well-aligned is the middle line
    ds_ts_gaussian_middles = successive_gaussians(ds_ts_list[0],ds_ts_list[1],ws_list,5,np.linspace(0,6000,1000),show=False)
    
    bspline = linear_bspline_interpolation(ds_ts_gaussian_middles[0],ds_ts_gaussian_middles[1],ds_ts_gaussian_middles[2],1e-3,show=True)
    rectified_dt_ts_middleline = np.stack([np.sort(ds_ts_list[0]),bspline(np.sort(ds_ts_list[0]))])
    rectified_middleline = (to_line_ref_matrix.T @ rectified_dt_ts_middleline)
    rectified_middleline[0] += estimated_src[0]
    rectified_middleline[1] += estimated_src[1] + coefs[1]
    
    show_refit_centerline(xs_flat,ys_flat,ws_list,estimated_src,coefs,rectified_middleline)
    
    
    
    return

##### Data comparison #####

def compare_3d(file_1:str,file_2:str,tol:float):
    raise NotImplementedError()


def compare_2d(file_1:str,file_2:str):
    
    # Data reading
    xs1,ys1,ws1 = extract_2d_stripped_sample_lists(file_1)
    xs1_flat = xs1.flatten()
    ys1_flat = ys1.flatten()
    ws1_list = ws1.flatten()
    
    xs2,ys2,ws2 = extract_2d_stripped_sample_lists(file_2)
    xs2_flat = xs2.flatten()
    ys2_flat = ys2.flatten()
    ws2_list = ws2.flatten()
    
    ### Estimate the source location (highest concentration point)
    
    estimated_src1 = simple_src_estimator(xs1_flat,ys1_flat,ws1_list)
    estimated_src2 = simple_src_estimator(xs2_flat,ys2_flat,ws2_list)
    print(f"Estimated source locations:\n1:\t{estimated_src1}\n2:\t{estimated_src2}")
    
    ### Estimate main axis ('middle line' of the points)  
    
    coefs1 = find_middleline_coefs(xs1_flat-estimated_src1[0],ys1_flat-estimated_src1[1],ws1_list)
    coefs2 = find_middleline_coefs(xs2_flat-estimated_src2[0],ys2_flat-estimated_src2[1],ws2_list)

    print(f"Center line (m,b) coefficients (y = m*x+b):\n1:\t{coefs1}\n2:\t{coefs2}")
    
    delta_line_coeff = np.arctan(np.pi/8)
    show_data_comp(xs1_flat,ys1_flat,ws1_list,xs2_flat,ys2_flat,ws2_list,delta_line_coeff)
    
    
    
    
#################### Main entry point ####################

def main():
    parser = argparse.ArgumentParser("Spray Analyser",
                                     description="Try to find good parameters for the exploring the gass cone provided")
    
    parser.add_argument("file_in",help="Input .nc file")
    parser.add_argument("-3d",dest='d3',help="If this flag is set, show a 3D scatter plot then close. The value given is the tolerance\
        (only points with concentration above are plotted).")
    parser.add_argument('--hour',default=None,help="For 3D analysis, the selected hour among the 20 existing hour timestamp.\
                        Default to selecting the one with maximal total concentration")
    parser.add_argument("-c","--compare",dest='compare',default=None,
                        help="Another file to analyse. Plot the results of both simultaneously, for comparison.")
    
    args = parser.parse_args()
    

    ### 3D plot if requested
    
    if args.d3:
        if args.compare:
            compare_3d(args.file_in,args.compare,float(args.d3))
        else:
            analysis_3d(args.file_in,float(args.d3),args.hour)
    else:
        if args.compare:
            compare_2d(args.file_in,args.compare)
        else:
            analysis_2d(args.file_in)
    
    

if __name__ == "__main__":
    main()
