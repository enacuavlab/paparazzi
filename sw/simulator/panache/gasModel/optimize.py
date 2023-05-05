import typing
import functools

import numpy as np

from gasModel.gasModel import gaussian_model,gaussian_model_jac,continued_gaussian_model,continued_gaussian_model_jac,log_gaussian_model,log_gaussian_model_jac
from gasModel.wlineModel import logistic,d_logistic,dd_logistic,weighted_dist2_from_hline,weighted_dist2_from_hline_jac

def my_soft_l1(z:float):
    return 2 * (np.sqrt(1 + z)- 1)

def d_my_soft_l1(z:float):
    return 1 / np.sqrt(z+1)

def dd_my_soft_l1(z:float):
    return -1/(2*(z+1)*np.sqrt(z+1))


def fit_gas_model(model:typing.Union[typing.Literal['gaussian_model'],typing.Literal['continued_gaussian_model'],typing.Literal['log_gaussian_model'],
                                     typing.Literal['hline_model']],
                  optimizer:typing.Union[typing.Literal['curve_fit'],typing.Literal['shgo'],typing.Literal['basinhopping']],
                  samples:np.ndarray,
                  centering:bool=True,
                  ) -> np.ndarray:
    
    norm_samples = samples.transpose()
    
    if centering:
        xmin = np.amin(norm_samples[0])
        ymin = np.amin(norm_samples[1])
        zmin = np.amin(norm_samples[2])
        norm_samples[0] = norm_samples[0] - xmin
        norm_samples[1] = norm_samples[1] - ymin
        norm_samples[2] = norm_samples[2] - zmin
        
        scale = 1.
        # scale = max(np.amax(norm_samples[0]),np.amax(norm_samples[1]),np.amax(norm_samples[2]))
        # norm_samples[0] /= scale
        # norm_samples[1] /= scale
        # norm_samples[2] /= scale
    else:
        xmin = 0.
        ymin = 0.
        zmin = 0.
        scale = 1.
        
    barycenter = np.average(norm_samples,axis=1)
    
    if model == 'gaussian_model':
        partial_fun = functools.partial(gaussian_model,norm_samples[:3])
        partial_jac = functools.partial(gaussian_model_jac,norm_samples[:3])

        fun = gaussian_model
        jac = gaussian_model_jac
        loss = 'soft_l1'

        gfun = lambda x : np.sum(np.square(my_soft_l1(norm_samples[-1] - partial_fun(*x))))
        d_gfun = lambda x : 2*np.sum((my_soft_l1(norm_samples[-1] - partial_fun(*x)))*(d_my_soft_l1(norm_samples[-1] - partial_fun(*x)))*partial_jac(*x).transpose(),axis=1)

        p0 = [barycenter[0],barycenter[1],barycenter[2],0.,1.,1.,1.]
        bounds=([-1e5,-1e5,0,-np.pi,0,0,0],[1e5,1e5,1e4,np.pi,1,10,10])

    elif model == 'continued_gaussian_model':
        partial_fun = functools.partial(continued_gaussian_model,norm_samples[:3])
        partial_jac = functools.partial(continued_gaussian_model_jac,norm_samples[:3])

        fun = continued_gaussian_model
        jac = continued_gaussian_model_jac
        loss = 'soft_l1'

        gfun = lambda x : np.sum(np.square(my_soft_l1(norm_samples[-1] - partial_fun(*x))))
        d_gfun = lambda x : 2*np.sum((my_soft_l1(norm_samples[-1] - partial_fun(*x)))*(d_my_soft_l1(norm_samples[-1] - partial_fun(*x)))*partial_jac(*x).transpose(),axis=1)

        p0 = [barycenter[0],barycenter[1],barycenter[2],0.,1.,1.,1.]
        bounds=([-1e5,-1e5,0,-np.pi,0,0,0],[1e5,1e5,1e4,np.pi,1,10,10])

    elif model == 'log_gaussian_model':
        partial_fun = functools.partial(log_gaussian_model,norm_samples[:3])
        partial_jac = functools.partial(log_gaussian_model_jac,norm_samples[:3])

        fun = log_gaussian_model
        jac = log_gaussian_model_jac
        loss = 'soft_l1'

        gfun = lambda x : np.sum(np.square(my_soft_l1(norm_samples[-1] - partial_fun(*x))))
        d_gfun = lambda x : 2*np.sum((my_soft_l1(norm_samples[-1] - partial_fun(*x)))*(d_my_soft_l1(norm_samples[-1] - partial_fun(*x)))*partial_jac(*x).transpose(),axis=1)

        p0 = [barycenter[0],barycenter[1],barycenter[2],0.,1.,1.,1.]
        bounds=([-1e5,-1e5,0,-np.pi,0.,0.,0.],[1e5,1e5,1e4,np.pi,1.,10.,10.])

        
        # Add the smallest positive float to avoid computing log of 0
        norm_samples[-1] += np.finfo(float).eps
        np.log(norm_samples[-1],out=norm_samples)
    elif model == 'hline_model':    
        # weights = norm_samples[-1]
        # max_positive_w = np.amax(weights[np.nonzero(weights)])
        # norm_samples[-1] /= max_positive_w 
        # norm_samples[-1] += min_positive_w/1000
        # np.log(norm_samples[-1],out=norm_samples[-1])
        # norm_samples[-1] += np.log(1000) # set the 0 as midpoint between the actual minimal sample and the offset zero-measurements
        
        partial_fun = functools.partial(weighted_dist2_from_hline,norm_samples)
        partial_jac = functools.partial(weighted_dist2_from_hline_jac,norm_samples)
        
        fun = lambda x : partial_fun(*x)
        jac = lambda x : partial_jac(*x)
    
        # my_logistic = functools.partial(logistic,L=1.,k=1.,center=np.log(1000))
        # my_d_logistic = functools.partial(d_logistic,L=1.,k=1.,center=np.log(1000))
        # my_dd_logistic = functools.partial(dd_logistic,L=1.,k=1.,center=np.log(1000))
        
        # def my_logistic_loss(val:float) -> np.ndarray:
        #     return np.stack([my_logistic(val),my_d_logistic(val),my_dd_logistic(val)])
        loss = 'soft_l1'
        
        
        p0 = [barycenter[0],barycenter[1],barycenter[2],0.]
        bounds=([-1e5,-1e5,0,-np.inf],[1e5,1e5,1e4,np.inf])
        
        gfun = lambda x : np.sum(np.square(my_soft_l1(fun(x))),axis=0)
        d_gfun = lambda x : 2*np.sum(my_soft_l1(fun(x))*(d_my_soft_l1(fun(x))*jac(x).transpose()),axis=1)
        
        # gfun = lambda x : np.sum(np.square(fun(x)),axis=0)
        # d_gfun = lambda x : 2*np.sum((fun(x)*jac(x).transpose()),axis=1)
        
    else:
        raise ValueError(f"Unknown model: {model}")
    
    
    
    
    if optimizer == 'curve_fit':
        import scipy.optimize as opt

        if model == 'hline_model':
            popt = opt.least_squares(fun,
                                     x0=p0,
                                     method='dogbox',
                                     jac=jac,
                                     #bounds=bounds,
                                     loss=loss,
                                     tr_solver='exact',
                                     verbose=2).x
        else:
            popt,_ = opt.curve_fit(fun,norm_samples[:-1],norm_samples[-1],
                                method='dogbox',
                                jac=jac,
                                bounds=bounds,
                                p0=p0,
                                loss=loss,
                                tr_solver='exact',
                                verbose=2,
                                )
    elif optimizer == 'shgo':
        import scipy.optimize as opt
        tr_bounds = np.stack([bounds[0],bounds[1]]).transpose()
        
        res = opt.shgo(gfun,
                        bounds=tr_bounds,
                        minimizer_kwargs={'method':'BFGS',
                                          'jac':d_gfun},
                        options={'jac':d_gfun,
                                 'minimize_every_iter':True,
                                 'disp':True})
        
        print("Success? :",res.success)
        print("Info:\n",res.message)
        popt = res.x
        
    elif optimizer == 'basinhopping':
        import scipy.optimize as opt
        
        res = opt.basinhopping(gfun,
                        x0=p0,
                        minimizer_kwargs={'method':'BFGS',
                                          'jac':d_gfun,
                                          'options':{'disp':True}},
                        disp=True)
        
        print("Success? :",res.success)
        print("Info:\n",res.message)
        popt = res.x
    else:
        raise ValueError(f"Unknown optimizer: {optimizer}")
    
    popt[0] *= scale
    popt[1] *= scale
    popt[2] *= scale
    
    popt[0] += xmin
    popt[1] += ymin
    popt[2] += zmin
    return popt