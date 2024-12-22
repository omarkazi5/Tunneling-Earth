# Student ID  : 2658466
# Project Name: Tunneling Earth
# Project ID  : S-TunnelingEarth
# Description : This code ...

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

# radius = {"crust":35000,"upmant":660000,"lomant":2890000,"ocore":5100000,"icore":6378000} # m


G = constants.gravitational_constant
Me = 5.972e24 # mass of earth (kg)
pi = math.pi
r0 = 6378000 # starting position
m = 70 # mass of person (kg)
dt = 100
total_t = 10000


# calls find_density
def M_enc(r):
    return 4/3 * 5515 * pi * r**3 

# calls M_enc
def force(r,m):
    # constant density is assumed
    F = -((G*m)) * 4/3 * 5515 * pi * r
    return F

# calls force
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
    return acc, vel, pos, time
        
def plot():
    acc, vel, pos, time = motion(dt,total_t)
    plt.figure(figsize=(10,5))
    plt.subplot(3,1,1)
    plt.plot(time,acc)
    plt.xlabel("time")
    plt.ylabel("acc")
    plt.subplot(3,1,2)
    plt.plot(time,vel)
    plt.xlabel("time")
    plt.ylabel("vel")
    plt.subplot(3,1,3)
    plt.plot(time,pos)
    plt.xlabel("time")
    plt.ylabel("pos")
    plt.show()
    return

plot()
