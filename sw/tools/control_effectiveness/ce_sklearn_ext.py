#!/usr/bin/env python3

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize
from sklearn import linear_model,preprocessing

from control_effectiveness_estimation import Configuration,genfromtxt,extract_virtual_cmd,get_time_from_conf,make_virtual_cmd
import control_effectiveness_utils as ut 

def find_eff_matrix(conf:Configuration, start:int, end:int, data:np.ndarray, virtual_cmd:Optional[np.ndarray], verbose:bool=False):
    inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(conf, data, start, end)
    if virtual_cmd is not None:
        commands, v_inv = make_virtual_cmd(virtual_cmd, commands)
    else:
        v_inv = np.identity(conf.nb_out)
            
    return fit_eff_matrix(conf, inputs, commands, v_inv, verbose, 'huber')

def fit_eff_matrix(conf:Configuration, inputs:np.ndarray, commands:np.ndarray, v_inv, verbose:bool=False, method:str='classic'):
    
    mixing = np.array(conf.mixing)
    (nb_in, nb_out) = np.shape(mixing)
    
    output = np.zeros((nb_in, nb_out))
    
    residuals = []

    scaler = preprocessing.StandardScaler(with_mean=False).fit(commands)

    if method == 'classic':
        for i in range(nb_in):
            cmd = np.multiply(commands, mixing[[i],:])
            axis_fit,res = ut.fit_axis(cmd, inputs[:,[i]], str(i), False)
            output[[i],:] = np.matmul(v_inv, axis_fit).T
            residuals.append(res)
    elif method == 'lasso':
        for i in range(nb_in):
            cmd = np.multiply(commands, mixing[[i],:])
            # axis_fit,res = ut.fit_axis(cmd, inputs[:,[i]], str(i), False)
            lasso = linear_model.Lasso(alpha=0.1)
            lasso.fit(scaler.transform(commands,True),inputs[:,[i]].ravel())
            res = np.sum(np.square(inputs[:,[i]] - lasso.predict(cmd)))
            axis_fit = scaler.transform(lasso.coef_.reshape((1,-1)))
            output[[i],:] = np.matmul(v_inv, axis_fit.T).T
            residuals.append(res)
    elif method == 'huber':
        for i in range(nb_in):
            cmd = np.multiply(commands, mixing[[i],:])
            # axis_fit,res = ut.fit_axis(cmd, inputs[:,[i]], str(i), False)
            huber = linear_model.HuberRegressor()
            huber.fit(scaler.transform(commands,True),inputs[:,[i]].ravel())
            res = np.sum(np.square(inputs[:,[i]] - huber.predict(cmd)))
            axis_fit = scaler.transform(huber.coef_.reshape((1,-1)))
            output[[i],:] = np.matmul(v_inv, axis_fit.T).T
            residuals.append(res)
    
    return output,np.array(residuals)

def find_eff_matrix_with_meta_opt(conf:Configuration, start:int, end:int, data:np.ndarray, virtual_cmd:Optional[np.ndarray], verbose:bool=False):
    param_names = [k for k in conf.ranges.keys()]
    bounds = [(conf.ranges[k][0],conf.ranges[k][1]) for k in param_names]
    
    rranges = [slice(conf.ranges[k][0],conf.ranges[k][1]+conf.ranges[k][2],conf.ranges[k][2]) for k in param_names]
        
    def obj_fun(xs):
        for i,x in enumerate(xs):
            conf.variables[param_names[i]] = x
                
        inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                    conf, data, start, end)
        if virtual_cmd is not None:
            commands, v_inv = make_virtual_cmd(virtual_cmd, commands)
        else:
            v_inv = np.identity(conf.nb_out)
                
        _,residuals = fit_eff_matrix(conf, inputs, commands, v_inv, False)
        return sum(residuals)
    
    x,v,grid,fgrid = optimize.brute(obj_fun, rranges, full_output=True,
                          finish=optimize.fmin)
    
    plt.scatter(grid,fgrid)
    plt.show()
    
    # print(logs)
    # plt.plot([e[0][0] for e in logs],[e[1] for e in logs],linestyle=' ', marker='+')
    # plt.show()
    
    # if not res.success:
        # raise ValueError("Failed SHG optimization accross ranges")
        
    for i,x in enumerate(x):
        conf.variables[param_names[i]] = x
            
    inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                conf, data, start, end)
    if virtual_cmd is not None:
        commands, v_inv = make_virtual_cmd(virtual_cmd, commands)
    else:
        v_inv = np.identity(conf.nb_out)
            
    return fit_eff_matrix(conf, inputs, commands, v_inv, verbose)
        

def main():
    from argparse import ArgumentParser
    import json,os

    parser = ArgumentParser(description="Control effectiveness estimation tool")
    parser.add_argument("config", help="JSON configuration file")
    parser.add_argument("data", help="Log file for parameter estimation")
    parser.add_argument("-f", "--sample_freq", dest="freq",
                      help="Sampling frequency, trying auto freq if not set")
    parser.add_argument("-var", "--variable", dest="vars", action='append', nargs=2,
                      metavar=('var_name','value'),
                      help="Set variables by name, 'None' for config file default")
    parser.add_argument("-s", "--start",
                      help="Start time",
                      action="store", dest="start", default="0")
    parser.add_argument("-e", "--end",
                      help="End time (-1 for unlimited time)",
                      action="store", dest="end", default=-1)
    parser.add_argument("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_argument("-r", "--use-ranges",
                      action="store_true", dest="use_ranges")
    parser.add_argument("-v", "--verbose",
                      action="store_true", dest="verbose")
    args = parser.parse_args()
 
 
    # Display parameters
    verbose = args.verbose
    plot = args.plot
    
    # Set up time parameters
    start = int(args.start)
    end = int(args.end)
    freq = args.freq
    if freq is not None:
        freq = float(freq)
        

    # Read data
    if not os.path.isfile(args.data):
        raise FileNotFoundError(args.data)
    data = genfromtxt(args.data, delimiter=',', skip_header=1)
    
    # Read configuration
    if not os.path.isfile(args.config):
        raise FileNotFoundError(args.config)
    with open(args.config, 'r') as f:
        conf:dict = json.load(f)
        
    configuration = Configuration.from_dict(conf,args.vars)
    
    # Create a virtual command
    virtual_cmd = None
    if 'virtual_cmd' in conf:
        virtual_cmd = extract_virtual_cmd(conf)
        # nb_out = virtual_cmd.shape[1]
    
    # Search for time vector in data from presets
    start, end, freq, time = get_time_from_conf(conf, start, end, freq, data)
    if time is None:
        start = int(start * freq)
        end = int(end * freq)
        time = np.arange(end-start) / freq # default time vector if not in data
    configuration.variables['freq'] = freq
    
    
    if len(configuration.ranges) > 0 and args.use_ranges:
        output,residuals = find_eff_matrix_with_meta_opt(configuration, start, end, data, virtual_cmd, verbose)
    else:
        output,residuals = find_eff_matrix(configuration, start, end, data, virtual_cmd, verbose)

    if plot:
        inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(configuration, data, start, end)
        for i in range(configuration.nb_in):
            name = ut.get_name_by_index(conf, 'input', i)
            estimated_accel = commands @ output[i]
            lin_fit = ut.fit_lin(estimated_accel, inputs[:,[i]][:,0], name, verbose)
            ut.plot_results(estimated_accel, inputs[:,[i]], raw_inputs[:,[i]], lin_fit, time, freq, name)

        plt.show()
        
    ut.print_results(conf, configuration.variables, output)


if __name__ == "__main__":
    main()