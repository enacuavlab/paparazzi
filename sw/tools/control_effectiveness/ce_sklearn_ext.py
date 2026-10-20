#!/usr/bin/env python3

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize
from sklearn import linear_model,preprocessing

from control_effectiveness_estimation import Configuration,genfromtxt,get_time_from_conf,make_virtual_cmd
import control_effectiveness_utils as ut 

def find_eff_matrix(conf:Configuration, start:int, end:int, data:np.ndarray, method:str, verbose:bool=False):
    inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(conf, data, start, end)
    print(f"Handling {commands.shape} input datapoints with {method}...")
    if conf.virtual_cmd is not None:
        commands = make_virtual_cmd(conf.virtual_cmd, commands)
        v_inv = np.linalg.pinv(conf.virtual_cmd)
    else:
        v_inv = np.identity(conf.nb_out)
            
    return fit_eff_matrix(conf, inputs, commands, v_inv, verbose, method)

def fit_eff_matrix(conf:Configuration, inputs:np.ndarray, commands:np.ndarray, v_inv, verbose:bool=False, method:str='classic'):
    
    output = np.zeros((conf.nb_in, conf.nb_out))
    
    RMSEs = []

    scaler = preprocessing.StandardScaler(with_mean=False).fit(commands)

    scaled_cmd = scaler.transform(commands,True)

    if method == 'classic':
        model = linear_model.LinearRegression()
    elif method == 'ridge':
        model = linear_model.Ridge()
    elif method == 'lasso':
        model = linear_model.Lasso()
    elif method == 'huber':
        model = linear_model.HuberRegressor()
    elif method == 'ransac':
        model = linear_model.RANSACRegressor()
    elif method == 'sgd':
        model = linear_model.SGDRegressor()
    else:
        raise NotImplementedError(f"Unkown method: {method}")

    
    minibatch_size = 1000

    for i in range(conf.nb_in):
        # cmd = np.multiply(commands, mixing[[i],:])
        model.fit(scaled_cmd,inputs[:,i].ravel())
        if verbose:
            print("Fitted!")
        
        res = 0
        for k in range(0,len(scaled_cmd),minibatch_size):
            res += np.sum(np.square(inputs[k:k+minibatch_size,[i]] - model.predict(scaled_cmd[k:k+minibatch_size])))
        res = np.sqrt(res)/len(inputs[:,[i]])
        if isinstance(model,linear_model.RANSACRegressor):
            axis_fit = scaler.transform(model.estimator_.coef_.reshape((1,-1)))
        else:
            axis_fit = scaler.transform(model.coef_.reshape((1,-1)))
        output[[i],:] = np.matmul(v_inv, axis_fit.T).T
        RMSEs.append(res)
        
    return output,np.array(RMSEs)

def find_eff_matrix_with_meta_opt(conf:Configuration, start:int, end:int, data:np.ndarray, method:str, verbose:bool=False):
    """Extract input and command data, then use them for estimating efficency coefficients.
    If given, convert to virtual commands first. Also perform a brute-force sweep of some
    estimation parameters to try improving the final accuracy. This relies on the "ranges" parameter in the configuration file.

    Args:
        conf (Configuration):   Configuration details
        start (int):            Start time for data extraction
        end (int):              End time for data extraction
        data (np.ndarray):      Source data array
        verbose (bool, optional): Display detailed estimation info. Defaults to False.

    Returns:
        (np.ndarray,np.ndarray): Efficiency matrix and estimation residuals
    """
    param_names = [k for k in conf.ranges.keys()]
    rranges = [slice(conf.ranges[k][0],conf.ranges[k][1]+conf.ranges[k][2],conf.ranges[k][2]) for k in param_names]
    
    ## Parse for printing problem size
    _, _, commands_print, _ = ut.extract_filtered_data(
                    conf, data, start, end)
    print(f"Handling {commands_print.shape} input datapoints with {method}...")
    ## End parse
    
    def obj_fun(xs):
        if len(param_names) > 1:
            for i,x in enumerate(xs):
                conf.variables[param_names[i]] = x
        else:
            conf.variables[param_names[0]] = xs
                
        inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                    conf, data, start, end)
        if conf.virtual_cmd is not None:
            commands = make_virtual_cmd(conf.virtual_cmd, commands)
            v_inv = np.linalg.pinv(conf.virtual_cmd)
        else:
            v_inv = np.identity(conf.nb_out)
                
        _,residuals = fit_eff_matrix(conf, inputs, commands, v_inv, False, method)
        return sum(residuals)
    
    if len(rranges) == 1:
        pname = param_names[0]
        res = optimize.minimize_scalar(obj_fun, (conf.ranges[pname][0],conf.ranges[pname][1]),tol=conf.ranges[pname][2]/2,
                                       options={'disp':3 if verbose else 0})
        
        x = [res.x]
    else:
        x,_,_,_ = optimize.brute(obj_fun, rranges, full_output=True,
                          finish=optimize.fmin)
    
    
    for i,x in enumerate(x):
        conf.variables[param_names[i]] = x
            
    inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                conf, data, start, end)
    if conf.virtual_cmd is not None:
        commands = make_virtual_cmd(conf.virtual_cmd, commands)
        v_inv = np.linalg.pinv(conf.virtual_cmd)
    else:
        v_inv = np.identity(conf.nb_out)
            
    return fit_eff_matrix(conf, inputs, commands, v_inv, verbose)
        

def main():
    from argparse import ArgumentParser
    import json,os

    parser = ArgumentParser(description="Control effectiveness estimation tool")
    parser.add_argument("config", help="JSON configuration file")
    parser.add_argument("data", help="Log file for parameter estimation")
    parser.add_argument("method", help="Linear regression method to be used", choices=['classic','ridge','lasso','huber','ransac','sgd'])
    parser.add_argument("-f", "--sample_freq", dest="freq",
                      help="Sampling frequency, trying auto freq if not set")
    parser.add_argument("-var", "--variable", dest="vars", action='append', nargs=2,
                      metavar=('var_name','value'),
                      help="Set variables by name, 'None' for config file default")
    parser.add_argument("--no-virtual", help="Ignore use virtual commands",
                        action='store_true',dest="no_virtual")
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
        
    configuration = Configuration.from_dict(conf,args.vars,args.no_virtual)
    
    # Search for time vector in data from presets
    start, end, freq, time = get_time_from_conf(conf, start, end, freq, data)
    if time is None:
        start = int(start * freq)
        end = int(end * freq)
        time = np.arange(end-start) / freq # default time vector if not in data
    configuration.variables['freq'] = freq
    
    
    if len(configuration.ranges) > 0 and args.use_ranges:
        output,residuals = find_eff_matrix_with_meta_opt(configuration, start, end, data, args.method, verbose)
    else:
        output,residuals = find_eff_matrix(configuration, start, end, data, args.method, verbose)

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