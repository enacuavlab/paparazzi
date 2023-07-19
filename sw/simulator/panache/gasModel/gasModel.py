import typing

import functools
import subprocess

import numpy as np
import scipy.optimize as opt
from scipy.spatial.transform import Rotation

######################################## Models' definitions ########################################

maximal_float = np.finfo(float).max
minimal_float = np.finfo(float).min
eps_float = np.finfo(float).eps

def referential_transform(pos:np.ndarray,xo:float,yo:float,zo:float,angle:float) -> np.ndarray:
    """The isometric transformation used to change from the 'base' referential to the 'plume' referential

    Args:
        pos (np.ndarray): Position to transform
        xo (float): Source position in base referential, x coordinate
        yo (float): Source position in base referential, y coordinate
        zo (float): Source position in base referential, z coordinate
        angle (float): Rotation along the z-axis (in radiant) to align the x-axis with the emission axis

    Returns:
        np.ndarray: Transformed position
    """
    rot = Rotation.from_euler('z',[angle],degrees=False).as_matrix()
    
    pos[0] = pos[0] - xo
    pos[1] = pos[1] - yo
    pos[2] = pos[2] - zo
    
    tr_pos = (rot @ pos)[0]
    return tr_pos

def inverse_referential_transform(pos:np.ndarray,xo:float,yo:float,zo:float,angle:float) -> np.ndarray:
    """The isometric transformation used to change to the 'base' referential from the 'plume' referential

    Args:
        pos (np.ndarray): Position to transform
        xo (float): Source position in base referential, x coordinate
        yo (float): Source position in base referential, y coordinate
        zo (float): Source position in base referential, z coordinate
        angle (float): Rotation along the z-axis (in radiant) to align the x-axis with the emission axis

    Returns:
        np.ndarray: Transformed position
    """
    rot = Rotation.from_euler('z',[-angle],degrees=False).as_matrix()
    
    r_pos = (rot @ pos)[0]
    tr_pos = r_pos 
    tr_pos[0] = tr_pos[0] + xo
    tr_pos[1] = tr_pos[1] + yo
    tr_pos[2] = tr_pos[2] + zo
    
    return tr_pos

########## The infinitely derivable version of the Gaussian plume model (no elevation) ##########

def continued_gaussian_model(pos:np.ndarray, xo:float, yo:float, zo:float, angle:float, intensity:float, ay:float, az:float) -> float:
    """Same as `gaussian_model`, but the plume is not set to 0 for x < 0. Divergence and negative concentration may happen...

    
    Args:
        pos (np.ndarray): Input position
        xo (float): Source position in base referential, x coordinate
        yo (float): Source position in base referential, y coordinate
        zo (float): Source position in base referential, z coordinate
        angle (float): Rotation along the z-axis (in radiant) to align the x-axis with the emission axis
        intensity (float): Gas emission intensity
        ay (float): Affine horizontal spreading factor (sy = ay * x + 1)
        az (float): Affine vertical spreading factor (sz = az * x + 1)

    Returns:
        float: Gas concentration at the given point
    """
    
    tr_pos = referential_transform(pos,xo,yo,zo,angle)
    
    # Use 1 a offset to avoid unbounded behavior at tr_x = 0
    sy = ay * tr_pos[0] + 1
    sz = az * tr_pos[0] + 1
    
    exponent = - (np.square(tr_pos[1]/sy)/2 + np.square(tr_pos[2]/sz)/2)
    output = (intensity / (2*np.pi * sy * sz)) * np.exp(exponent)
    
    return output

def continued_gaussian_model_jac(pos:np.ndarray, xo:float, yo:float, zo:float, angle:float, intensity:float, ay:float, az:float) -> np.ndarray:
    val = continued_gaussian_model(pos,xo,yo,zo,angle,intensity,ay,az)
    
    it = intensity 
    
    cos = np.cos
    sin = np.sin
    
    tr_pos = referential_transform(pos,xo,yo,zo,angle)
    
    tr_x = tr_pos[0]
    tr_y = tr_pos[1]
    tr_z = tr_pos[2]
    
    d_xo = val * ay*cos(angle)/(tr_x*ay + 1) - tr_y**2*ay*cos(angle)/(tr_x*ay + 1)**3 + az*cos(angle)/(tr_x*az + 1) - az*(tr_z)**2*cos(angle)/(tr_x*az + 1)**3 + tr_y*sin(angle)/(tr_x*ay + 1)**2
    d_yo = -val * ay*sin(angle)/(tr_x*ay + 1) + tr_y**2*ay*sin(angle)/(tr_x*ay + 1)**3 - az*sin(angle)/(tr_x*az + 1) + az*(tr_z)**2*sin(angle)/(tr_x*az + 1)**3 + tr_y*cos(angle)/(tr_x*ay + 1)**2
    d_zo = val * (tr_z)/(tr_x*az + 1)**2
    d_angle = val * tr_y*(ay/(tr_x*ay + 1) - tr_y**2*ay/(tr_x*ay + 1)**3 + az/(tr_x*az + 1) - az*(tr_z)**2/(tr_x*az + 1)**3 - tr_x/(tr_x*ay + 1)**2) 
    d_intensity = val / it
    d_ay = val * tr_x*(tr_y**2/(tr_x*ay + 1)**2 - 1)/(tr_x*ay + 1) 
    d_az = val * tr_x*((tr_z)**2/(tr_x*az + 1)**2 - 1)/(tr_x*az + 1) 

    return np.stack([d_xo,d_yo,d_zo,d_angle,d_intensity,d_ay,d_az]).transpose()

########## The truncated version of the Gaussian plume model (no elevation, x < 0 inthe plume referential implies no gas measured) ##########

def gaussian_model(pos:np.ndarray, xo:float, yo:float, zo:float, angle:float, intensity:float, ay:float, az:float) -> float:
    """Standard Gaussian model (with appropriate transformation for source localization and orientation)

    We use an offset of 1 of the gas standard deviation to avoid divergence at x = 0 (in the plume referential).
    Also, the plume is set to 0 for x <= 0
    
    Args:
        pos (np.ndarray): Input position
        xo (float): Source position in base referential, x coordinate
        yo (float): Source position in base referential, y coordinate
        zo (float): Source position in base referential, z coordinate
        angle (float): Rotation along the z-axis (in radiant) to align the x-axis with the emission axis
        intensity (float): Gas emission intensity
        ay (float): Affine horizontal spreading factor (sy = ay * x + 1)
        az (float): Affine vertical spreading factor (sz = az * x + 1)

    Returns:
        float: Gas concentration at the given point
    """
    tr_pos = referential_transform(pos,xo,yo,zo,angle)
    
    # Use 1 a offset to avoid unbounded behavior at tr_x = 0
    sy = ay * tr_pos[0] + 1
    sz = az * tr_pos[0] + 1
    
    exponent = - (np.square(tr_pos[1]/sy)/2 + np.square(tr_pos[2]/sz)/2)
    output = (intensity / (2*np.pi * sy * sz)) * np.exp(exponent)
    
    return output * (tr_pos[0] > 0)

def gaussian_model_jac(pos:np.ndarray, xo:float, yo:float, zo:float, angle:float, intensity:float, ay:float, az:float) -> np.ndarray:
    tr_pos = referential_transform(pos,xo,yo,zo,angle)
    
    return (continued_gaussian_model_jac(pos,xo,yo,zo,angle,intensity,ay,az).transpose() * (tr_pos[0] > 0)).transpose()

########## Logarithm of the Gaussian plume model ##########

def log_gaussian_model(pos:np.ndarray, xo:float, yo:float, zo:float, angle:float, intensity:float, ay:float, az:float) -> float:
    """Logarithm of the Standard Gaussian model
    
    Args:
        pos (np.ndarray): Input position
        xo (float): Source position in base referential, x coordinate
        yo (float): Source position in base referential, y coordinate
        zo (float): Source position in base referential, z coordinate
        angle (float): Rotation along the z-axis (in radiant) to align the x-axis with the emission axis
        intensity (float): Gas emission intensity
        ay (float): Affine horizontal spreading factor (sy = ay * x + 1)
        az (float): Affine vertical spreading factor (sz = az * x + 1)

    Returns:
        float: Gas concentration at the given point
    """
    tr_pos = referential_transform(pos,xo,yo,zo,angle)
    
    # Use 1 a offset to avoid unbounded behavior at tr_x = 0
    sy = ay * tr_pos[0] + 1
    sz = az * tr_pos[0] + 1
    
    exponent = - (np.square(tr_pos[1]/sy)/2 + np.square(tr_pos[2]/sz)/2)
    # output = (intensity / (2*np.pi * sy * sz)) * np.exp(exponent)
    output = np.log(intensity / (2*np.pi * sy * sz)) + exponent
    
    # Remove infinite values and use the largest negative defined float instead
    output[output == -np.inf] = minimal_float
    return output

def log_gaussian_model_jac(pos:np.ndarray, xo:float, yo:float, zo:float, angle:float, intensity:float, ay:float, az:float) -> np.ndarray:
    it = intensity
    
    cos = np.cos
    sin = np.sin
    
    tr_pos = referential_transform(pos,xo,yo,zo,angle)
    
    tr_x = tr_pos[0]
    tr_y = tr_pos[1]
    tr_z = tr_pos[2]
    
    
    
    d_xo = ay*cos(angle)/(tr_x*ay + 1) - tr_y**2*ay*cos(angle)/(tr_x*ay + 1)**3 + az*cos(angle)/(tr_x*az + 1) - az*(tr_z)**2*cos(angle)/(tr_x*az + 1)**3 + tr_y*sin(angle)/(tr_x*ay + 1)**2
    d_yo = -ay*sin(angle)/(tr_x*ay + 1) + tr_y**2*ay*sin(angle)/(tr_x*ay + 1)**3 - az*sin(angle)/(tr_x*az + 1) + az*(tr_z)**2*sin(angle)/(tr_x*az + 1)**3 + tr_y*cos(angle)/(tr_x*ay + 1)**2 
    d_zo = (tr_z)/(tr_x*az + 1)**2 
    d_angle = tr_y*(ay/(tr_x*ay + 1) - tr_y**2*ay/(tr_x*ay + 1)**3 + az/(tr_x*az + 1) - az*(tr_z)**2/(tr_x*az + 1)**3 - tr_x/(tr_x*ay + 1)**2) 
    
    repetitions = np.shape(d_xo)[0]
    d_it = np.repeat(1/it,repetitions)
    
    d_ay = tr_x*(tr_y**2/(tr_x*ay + 1)**2 - 1)/(tr_x*ay + 1) 
    d_az = tr_x*((tr_z)**2/(tr_x*az + 1)**2 - 1)/(tr_x*az + 1) 

    return (np.stack([d_xo,d_yo,d_zo,d_angle,d_it,d_ay,d_az]) * (tr_x > 0)).transpose()

######################################## SageMath, help me ! ########################################

def ask_gaussian_derivative_to_sage(print_to_stdout:bool) -> str:
    sage_transform_fun_str = "my_transform(x,y,z,xo,yo,zo,angle) = ((x-xo) * cos(angle) - (y - yo) * sin(angle), (x - xo) * sin(angle) + (y - yo) * cos(angle), z - zo)"
    sage_sigma_fun_str = "my_sigma(x,a) = a*x+1"
    gaussian_sage_str = "gaussian(t,s) = exp(-t*t/(2*s*s))/(sqrt(2 * pi) * s)"
    simple_gaussian_model_sage_str = "simple_gaussian_model(x,y,z,it,ay,az) = it * gaussian(y,my_sigma(x,ay)) * gaussian(z,my_sigma(x,az))"
    continued_gaussian_model_sage_str = "continued_gaussian_model(x,y,z,xo,yo,zo,angle,it,ay,az) =\
        simple_gaussian_model(my_transform(x,y,z,xo,yo,zo,angle)[0],my_transform(x,y,z,xo,yo,zo,angle)[1],my_transform(x,y,z,xo,yo,zo,angle)[2],it,ay,az)"
    log_continued_gaussian_model_sage_str = "log_continued_gaussian_model(x,y,z,xo,yo,zo,angle,it,ay,az) = log(continued_gaussian_model(x,y,z,xo,yo,zo,angle,it,ay,az))"
    
    sage_cmd = f"""{sage_transform_fun_str}\n{sage_sigma_fun_str}\n{gaussian_sage_str}\n{simple_gaussian_model_sage_str}\n{continued_gaussian_model_sage_str}\n{log_continued_gaussian_model_sage_str}
print(continued_gaussian_model.collect_common_factors())
print("--------------------\\nDerivatives:\\n")
for i,v in enumerate(['x','y','z','xo','yo','zo','angle','it','ay','az']):
    print("d/d",v," : ",diff(continued_gaussian_model)[i].collect_common_factors(),'\\n')
print("--------------------\\nNormalized Derivatives (diff(gaussian)/gaussian):\\n")
for i,v in enumerate(['x','y','z','xo','yo','zo','angle','it','ay','az']):
    print("d/d",v," : ",(diff(continued_gaussian_model)/continued_gaussian_model)[i].collect_common_factors(),'\\n')
print("====================================================================================================")
print("Log-Gaussian model (help with normalisation and model-error estimation)")
print(log_continued_gaussian_model.expand_log().collect_common_factors())
print("--------------------\\nDerivatives:\\n")
for i,v in enumerate(['x','y','z','xo','yo','zo','angle','it','ay','az']):
    print("d/d",v," : ",diff(log_continued_gaussian_model)[i].collect_common_factors(),'\\n')"""
    try:
        sage_answer = subprocess.run(['sage','-c',sage_cmd],capture_output=True,check=True,text=True)
    except subprocess.CalledProcessError as e:
        print("Sage STDOUT:",e.stdout)
        print("Sage STDERR:",e.stderr)
        raise e
    
    if print_to_stdout:
        print(sage_answer.stdout)
    return sage_answer.stdout

if __name__ == "__main__":
    ask_gaussian_derivative_to_sage(True)
        
    