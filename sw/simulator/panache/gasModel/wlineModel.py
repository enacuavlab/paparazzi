from __future__ import annotations

import typing
import subprocess

import numpy as np

from scipy.spatial.transform import Rotation

#################### The logistic function ####################
# Define its generalized version (with scale, slope and centering parameters)
# and its derivatives, knowning that it is the solution of the following differential equation:
#       f'(x) = f(x)*(1-f(x))

def logistic(x:float,L:float,k:float,center:float) -> float:
    return L/(1+np.exp(-k*(x-center)))

def d_logistic(x:float,L:float,k:float,center:float) -> float:
    val = logistic(x,L,k,center)
    return val * (1-val)

def dd_logistic(x:float,L:float,k:float,center:float) -> float:
    val = logistic(x,L,k,center)
    return val * (1-val) * (1-2*val)

#################### Distance from an horizontal line ####################
    
def dist2_from_hline(pos:np.ndarray,x0:float,y0:float,z0:float,z_angle:float) -> float:
    # i.e distance (squared) between `pos` and its orthogonal projection on the line 
    tr = np.asarray([x0,y0,z0])
    
    d_pos_x = pos[0] - tr[0]
    d_pos_y = pos[1] - tr[1]
    d_pos_z = pos[2] - tr[2]
    
    d_pos_dist2 = np.square(d_pos_x) + np.square(d_pos_y) + np.square(d_pos_z)
    scalar_product = np.cos(z_angle) * d_pos_x + np.sin(z_angle) * d_pos_y
    return d_pos_dist2 - np.square(scalar_product)

def dist2_from_hline_jac(pos:np.ndarray,x0:float,y0:float,z0:float,z_angle:float) -> np.ndarray:
    x = pos[0]
    y = pos[0]
    z = pos[0]
    
    angle = z_angle
    cos = np.cos
    sin = np.sin
    
    d_x0 = 2*((x - x0)*cos(angle) + (y - y0)*sin(angle))*cos(angle) - 2*x + 2*x0 
    d_y0 = 2*((x - x0)*cos(angle) + (y - y0)*sin(angle))*sin(angle) - 2*y + 2*y0 
    d_z0 = -2*z + 2*z0
    d_angle = -2*((x - x0)*cos(angle) + (y - y0)*sin(angle))*((y - y0)*cos(angle) - (x - x0)*sin(angle)) 
        
    return np.stack([d_x0,d_y0,d_z0,d_angle]).transpose()

def weighted_dist2_from_hline(pt:np.ndarray,x0:float,y0:float,z0:float,z_angle:float) -> float:
    return dist2_from_hline(pt[:3],x0,y0,z0,z_angle) * pt[3]

def weighted_dist2_from_hline_jac(pt:np.ndarray,x0:float,y0:float,z0:float,z_angle:float) -> np.ndarray:
    return (dist2_from_hline_jac(pt[:3],x0,y0,z0,z_angle).transpose() * pt[3]).transpose()

def ask_hline_derivative_to_sage(print_to_stdout:bool) -> str:
    hline_dist2_sage_str = "dist2_hline(x,y,z,x0,y0,z0,angle) = (x - x0)^2 + (y - y0)^2 + (z - z0)^2 - (cos(angle)*(x-x0) + sin(angle)*(y-y0))^2"
    sage_cmd = f"""{hline_dist2_sage_str}\n
print(dist2_hline)
print("--------------------\\nDerivatives:\\n")
for i,v in enumerate(['x','y','z','x0','y0','z0','angle']):
    print("d/d",v," : ",diff(dist2_hline)[i].collect_common_factors(),'\\n')
print("--------------------\\nSecond Derivatives:\\n")
for i,v in enumerate(['x','y','z','x0','y0','z0','angle']):
    if v in ['x','y','z']:
        print('Skipped derivative by ',v)
        continue
    for j,w in enumerate(['x','y','z','x0','y0','z0','angle']):
        if w in ['x','y','z']:
            print('Skipped derivative by ',w)
            continue
        print("d²/d",v,w," : ",dist2_hline.hessian()[i][j].collect_common_factors(),'\\n')"""
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
    ask_hline_derivative_to_sage(True)


    
    