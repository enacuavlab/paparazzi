# Simulator for the gvf_parametric algorithm in Paparazzi (fixed-wing).
# Here you can check the demanded climbing and heading rates for your aircraft
# and the expected trajectories as well.

import numpy as np
from scipy import linalg as la

import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D

# Happy pdf for a happy submission without complains in paperplaza, arxiv, etc
font = {'size'   : 20}

matplotlib.rc('font', **font)

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

# Simulation parameters
tf = 500
dt = 5e-2
time = np.linspace(0, tf, tf/dt)
it = 1

# Data log
X_h = np.zeros((time.size, 5))
theta_h = np.zeros((time.size, 1))
e1_h = np.zeros((time.size, 1))
e2_h = np.zeros((time.size, 1))
e3_h = np.zeros((time.size, 1))
u_theta_h = np.zeros((time.size, 1))
u_z_h = np.zeros((time.size, 1))
u_w1_h = np.zeros((time.size, 1))
u_w2_h = np.zeros((time.size, 1))

# Initial conditions
X = np.array([[100], [-53], [40], [0], [0]])
X_dot = np.array([[0], [0], [0], [0], [0]])
theta = np.pi/4

X_h[0, :] = X.transpose()
theta_h[0] = theta

# Desired trajectory
xo = 0
yo = 0
zo = 100
rh = 100
rv = 10

# Controller
L = 5e-2
beta1 = 1e-2
beta2 = 4e-2
k1 = 1e-3
k2 = 1e-3
k3 = 1e-3
ktheta = 0.5

# Vehicle
s = 15

for t in time[:-1]:

    x = X[0][0]
    y = X[1][0]
    z = X[2][0]
    w1 = X[3][0]
    w2 = X[4][0]

    w1b = w1*beta1
    w2b = w2*beta2

    #f
    f1 = (rh + rv*np.cos(w2b))*np.cos(w1b) + xo
    f2 = (rh + rv*np.cos(w2b))*np.sin(w1b) + yo
    f3 = rv*np.sin(w2b) + zo

    f1dw1 = -np.sin(w1b)*(rh + rv*np.cos(w2b))
    f2dw1 =  np.cos(w1b)*(rh + rv*np.cos(w2b))
    f3dw1 =  0

    f1ddw1 = -np.cos(w1b)*(rh + rv*np.cos(w2b))
    f2ddw1 = -np.sin(w1b)*(rh + rv*np.cos(w2b))
    f3ddw1 = 0

    f1dw1dw2 =  rv*np.sin(w1b)*np.sin(w2b)
    f2dw1dw2 = -rv*np.cos(w1b)*np.sin(w2b)
    f3dw1dw2 = 0

    f1dw2 = -rv*np.sin(w2b)*np.cos(w1b)
    f2dw2 = -rv*np.sin(w2b)*np.sin(w1b)
    f3dw2 =  rv*np.cos(w2b)

    f1ddw2 = -rv*np.cos(w2b)*np.cos(w1b)
    f2ddw2 = -rv*np.cos(w2b)*np.sin(w1b)
    f3ddw2 = -rv*np.sin(w2b)

    f1dw2dw1 =  rv*np.sin(w2b)*np.sin(w1b)
    f2dw2dw1 = -rv*np.sin(w2b)*np.cos(w1b)
    f3dw2dw1 = 0

    #phi
    phi1 = L*(x - f1)
    phi2 = L*(y - f2)
    phi3 = L*(z - f3)

    #Chi, J
    Chi = L*np.array([[-L*L*(beta1*f1dw1 - beta2*f1dw2) -k1*phi1],
                      [-L*L*(beta1*f2dw1 - beta2*f2dw2) -k2*phi2],
                      [-L*L*(beta1*f3dw1 - beta2*f3dw2) -k3*phi3],
                      [-L*L + beta1*(k1*phi1*f1dw1 + k2*phi2*f2dw1 + k3*phi3*f3dw1)],
                      [ L*L + beta2*(k1*phi1*f1dw2 + k2*phi2*f2dw2 + k3*phi3*f3dw2)]])

    j03 = L*(beta1*beta2*L*f1dw1dw2 - beta1*beta1*L*f1ddw1 + k1*beta1*f1dw1)
    j13 = L*(beta1*beta2*L*f2dw1dw2 - beta1*beta1*L*f2ddw1 + k2*beta1*f2dw1)
    j23 = L*(beta1*beta2*L*f3dw1dw2 - beta1*beta1*L*f3ddw1 + k3*beta1*f3dw1)
    j04 = L*(-beta1*beta2*L*f1dw2dw1 + beta2*beta2*L*f1ddw2 + k1*beta2*f1dw2)
    j14 = L*(-beta1*beta2*L*f2dw2dw1 + beta2*beta2*L*f2ddw2 + k2*beta2*f2dw2)
    j24 = L*(-beta1*beta2*L*f3dw2dw1 + beta2*beta2*L*f3ddw2 + k3*beta2*f3dw2)
    j33 = beta1*beta1*(k1*(phi1*f1ddw1-L*f1dw1*f1dw1) + k2*(phi2*f2ddw1-L*f2dw1*f2dw1) + k3*(phi3*f3ddw1-L*f3dw1*f3dw1))
    j43 = beta1*beta2*(k1*(phi1*f1dw1dw2-L*f1dw1*f1dw2) + k2*(phi2*f2dw1dw2-L*f2dw1*f2dw2) + k3*(phi3*f3dw1dw2-L*f3dw1*f3dw2))
    j34 = beta2*beta1*(k1*(phi1*f1dw2dw1-L*f1dw1*f1dw2) + k2*(phi2*f2dw2dw1-L*f2dw1*f2dw2) + k3*(phi3*f3dw2dw1-L*f3dw1*f3dw2))
    j44 = beta2*beta2*(k1*(phi1*f1ddw2-L*f1dw2*f1dw2) + k2*(phi2*f2ddw2-L*f2dw2*f2dw2) + k3*(phi3*f3ddw2-L*f3dw2*f3dw2))

    J = L*np.array([[-k1*L,        0,      0, j03, j04],
                   [     0,    -k2*L,      0, j13, j14],
                   [     0,      0,    -k3*L, j23, j24],
                   [beta1*L*k1*f1dw1, beta1*L*k2*f2dw1, beta1*L*k3*f3dw1, j33, j34],
                   [beta2*L*k1*f1dw2, beta2*L*k2*f2dw2, beta2*L*k3*f3dw2, j43, j44]])

    #G, Fp, Gp
    G = np.array([[1,0,0,0,0],
                  [0,1,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0]])

    Fp = np.array([[0, -1, 0, 0, 0],
                   [1,  0, 0, 0, 0]])

    Gp = np.array([[0, -1, 0, 0, 0],
                   [1,  0, 0, 0, 0],
                   [0,  0, 0, 0, 0],
                   [0,  0, 0, 0, 0],
                   [0,  0, 0, 0, 0]])

    h = np.array([[np.cos(theta)],[np.sin(theta)]])
    ht = h.transpose()

    Chit = Chi.transpose()
    Chinorm = np.sqrt(Chi.transpose().dot(Chi))[0][0]
    Chih = Chi / Chinorm

    u_theta = (-(1/(Chit.dot(G).dot(Chi))*Chit.dot(Gp).dot(np.eye(5) - Chih.dot(Chih.transpose())).dot(J).dot(X_dot)) - ktheta*ht.dot(Fp).dot(Chi) / np.sqrt(Chit.dot(G).dot(Chi)))[0][0]

    u_zeta = Chi[2][0]*s / np.sqrt(Chi[0][0]*Chi[0][0] + Chi[1][0]*Chi[1][0])
    u_w1   = Chi[3][0]*s / np.sqrt(Chi[0][0]*Chi[0][0] + Chi[1][0]*Chi[1][0])
    u_w2   = Chi[4][0]*s / np.sqrt(Chi[0][0]*Chi[0][0] + Chi[1][0]*Chi[1][0])

    # Euler integration
    theta = theta + u_theta*dt
    X_dot = np.array([[s*np.cos(theta)],[s*np.sin(theta)], [u_zeta], [u_w1], [u_w2]])
    X = X + X_dot*dt


    # Log
    X_h[it, :] = X.transpose()
    theta_h[it] = theta
    e1_h[it] = phi1 / L
    e2_h[it] = phi2 / L
    e3_h[it] = phi3 / L
    u_theta_h[it] = u_theta
    u_z_h[it] = u_zeta
    u_w1_h[it] = u_w1
    u_w2_h[it] = u_w2

    it = it + 1

# Plots

fig3d = plt.figure()
ax3d = fig3d.gca(projection='3d')
ax3d.plot(X_h[:,0], X_h[:,1], X_h[:,2])

f, (axe1,axe2,axe3) = plt.subplots(3,1)
axe1.plot(time[1:], e1_h[1:])
axe2.plot(time[1:], e2_h[1:])
axe3.plot(time[1:], e3_h[1:])
axe3.set_xlabel("Time [s]")
axe1.set_xlabel("error X")
axe2.set_xlabel("error Y")
axe3.set_xlabel("error Z")

f, (axut,axuz,axuw1,axuw2) = plt.subplots(4,1)
axut.plot(time[1:], u_theta_h[1:])
axuz.plot(time[1:], u_z_h[1:])
axuw1.plot(time[1:], u_w1_h[1:])
axuw2.plot(time[1:], u_w2_h[1:])
axuw1.set_xlabel("Time [s]")
axuw2.set_xlabel("Time [s]")
axut.set_ylabel("Heading rate [rad/s]")
axuz.set_ylabel("Climbing rate [m/s]")
axuw1.set_ylabel("Virual coord 1 rate")
axuw2.set_ylabel("Virual coord 2 rate")

plt.show()

