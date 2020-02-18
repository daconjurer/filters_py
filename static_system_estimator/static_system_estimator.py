from math import *
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

""" State estimator of a static system

  Estimation algorithm outline: using the measurements of the variable
  or set of variables (Zn) that make up the state of the system, the
  state of the system (Xn) will be updated using the State Update
  Equation and a gain (AKA Kalman gain); the System State Estimate
  also depends on an initial guess. A prediction of the state for the
  next iteration is then calculated using the System's Dynamic Model.
  Subsequent updates of the state of the system are then calculated
  for the following time steps.
"""

""" The filter will be used to estimate the values of the simulated
  scans of the LIDAR Hokuyo in ROS Kinetic/Gazebo 7. All the values of
  the dataset (range values AKA ranges) are suposed to beconstant
  during the simulation time but gaussian noise is added to every
  measurement.
"""

""" Data loading (this data was obtained from a rosbag and then
  converted into a .csv file)
"""

scans = pd.read_csv("./data/data_jackal.csv")
""" Noise is then added """
mu = 0
sigma = 0.5
noise = np.random.normal(mu,sigma,scans.shape)
scans = scans + noise

""" Initialization """
k_gain = 0.0

""" The first value of each class will be used as the initial guess
  i.e. the first prediction is equal to the intial guess for every
  class
"""
initial_guess = scans.iloc[0]
x_prev = initial_guess

for i in range(len(scans.index)):
  """ The measurement at time step n (Zn) is processed """
  z_curr = scans.iloc[i]
  
  """ The value of the gain is updated after each iteration. Notice
  that this equation is the particular case for a static state system
  i.e. future state = current state or current state = previous state
  """
  k_gain = 1/(i+1)

  """ Using the State Update Equation, the current estimate is
  calculated (particular case for a static state system)
  """
  x_curr = x_prev + k_gain*(z_curr - x_prev)
  
  """ For a static model, the next state estimate (prediction) is
  equal to the current state estimate
  """
  x_prev = x_curr

print(x_curr)
""" Plot of the LIDAR scans """

""" The resolution of the scan is 0.0065540750511 rads/step and the
  sensor makes 719 steps to retrieve 720 range values covering 270
  degrees.
"""
theta_l = np.deg2rad(np.linspace(135,0.375,num = 360))
theta_r = np.deg2rad(np.linspace(360,225.375,num = 360))
theta =np.concatenate((theta_l,theta_r))
r_estimates = x_curr

theta_max = 135
theta_min = -135

size = np.full((1,len(r_estimates)),1)

theta2 = np.deg2rad(np.linspace(135,100,20))
r2 = np.linspace(1,3,20)

fig = plt.figure()
ax1 = fig.add_subplot(121,projection='polar')
theta1 = theta[0:9]
r1 = r.values[0:9]
c = ax1.scatter(theta,r_estimates,s = size,cmap = 'hsv',alpha = 1.0)
ax1.set_thetamin(theta_min)
ax1.set_thetamax(theta_max)
ax1.set_theta_zero_location('N')
ax1.set_theta_direction(-1)

ax2 = fig.add_subplot(122,projection='polar')
c = ax2.scatter(theta,initial_guess,s = size,cmap = 'hsv',alpha = 1.0)
ax2.set_thetamin(theta_min)
ax2.set_thetamax(theta_max)
ax2.set_theta_zero_location('N')
ax2.set_theta_direction(-1)

plt.show()
