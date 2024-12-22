# Student ID  : 2658466
# Project Name: Tunneling Earth
# Project ID  : S-TunnelingEarth
''' 
Description : This code simulates the oscillatory motion of a person through a hypothetical tunnel passing through the Earth, 
assuming Earth's varying density layers. It calculates gravitational force based on enclosed mass, 
integrates equations of motion using numerical methods, and visualizes acceleration, velocity, and position over time with plots.
'''
from scipy import constants
import math
import matplotlib.pyplot as plt

'''
Force calculation:
==================
F = GMm/r^2
F = -(Gm/r^2) * M_enc
M_enc = rho * V
V = 4/3 * pi * r^3
M_enc = rho * 4/3 * pi * r^3
F = -(Gm/r^2) * M_enc
F = -(Gm/r^2) * (4/3 * rho * pi * r^3)
'''

uplim_radius = {"crust":6378000,"upmant":6343000,"lomant":5718000,"ocore":3488000,"icore":1278000} # m
lolim_radius = {"crust":6343000,"upmant":5718000,"lomant":3488000,"ocore":1278000,"icore":0} # m
density = {"crust":2550,"upmant":3900,"lomant":4500,"ocore":11050,"icore":12950} # kg/m^3


G = constants.gravitational_constant
Me = 5.972e24 # mass of earth (kg)
pi = math.pi
r0 = 6378000 # starting position
m = 70 # mass of person (kg)
dt = 100 # delta time
total_t = 10000 # total time for which the graph runs


# finds and returns density according to radius using the dictionaries
def find_density(r):
    if abs(r) < uplim_radius["icore"]:
        rho = density["icore"]
    elif abs(r) < uplim_radius["ocore"]:
        rho = density["ocore"]
    elif abs(r) < uplim_radius["lomant"]:
        rho = density["lomant"]
    elif abs(r) < uplim_radius["upmant"]:
        rho = density["upmant"]
    # elif abs(r) <= uplim_radius["crust"]:
    else:
        rho = density["crust"]  
    return rho

# calculates and returns enclosed mass at r. calls find_density().
def M_enc(r):
    M_enc = 0
    dr = 100
    for i in range(0,int(abs(r)+1),dr):
        # INTEGRATION: summation[0,r] (rho(i)*4*pi*i^2)*dr ---> this integral gives us: M_enc = (rho*4/3*pi*r^3)
        M_enc += 4 * find_density(i) * pi * i**2 * dr
    return M_enc

# caclulates gravitational force at r. calls M_enc()
def force(r,m):
    F = -(G*m/(r**2)) * M_enc(r)
    # if r = 0, GMm/r^2 would give undefined, but at center force is zero, so return 0.
    if r == 0:
        return 0
    # if r is -ve, it crossed the center, so switch the sign of force to change direction
    elif r < 0:
        return -F
    return F

# simulates the motion of the person. calls force()
def motion(dt,total_t):
    acc = []
    vel = []
    pos = []
    time = [] 
    v = 0
    r = r0

    for t in range (0,total_t,dt):
        a = force(r,m) / m
        v = v + (a * dt)
        r = r + (v * dt)
        acc.append(a)
        vel.append(v)
        pos.append(r)
        time.append(t)
    # returns acceleration, velocity, position, and time as different arrays
    return acc, vel, pos, time

# plots the graphs. calls force()      
def plot():
    acc, vel, pos, time = motion(dt,total_t)
    plt.plot(time,acc)
    plt.xlabel("time (s)")
    plt.ylabel("acceleration (m/s^2)")
    plt.show()
    


plot()


# the fluctuations in the acceleration graph are caused due to the gibbs phenomenon.