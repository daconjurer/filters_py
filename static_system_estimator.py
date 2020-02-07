from math import *
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

""" State estimator of a static system

  Estimation algorithm outline: using the measurements of the variable
  or set of variables (Zn) that make up the state of the system, the state
  of the system (Xn) will be updated using the State Update Equation and a
  gain (aka Kalman gain); the System State Estimate also depends on an
  initial guess. A prediction of the state for the next iteration is then
  calculated using the System's Dynamic Model. Subsequent updates of the
  state of the system are then calculated for the following time steps.
"""

"""  Prediction function """
def prediction(x):
  """ prediction represents the dynamic model of the system, it takes in
  the System State Estimate at time step n and returns the predicted state
  for the next time step n+1.
  """
  return x

""" The filter will be used to estimate the values of the simulated scans
  of the LIDAR Hokuyo in ROS Kinetic/Gazebo 7. All the values of the dataset
  (range values aka ranges) are suposed to beconstant during the simulation time.
"""

""" Data loading
  (this data was obtained from a rosbag and then converted into a .csv file)
"""

scans = pd.read_csv("./data/data_jackal.csv")
#print(scans.index)
#print(scans.columns)
#print(scans.iloc[0,0])

""" Initialization """
k_gain = 0.0

""" The first value of each class will be used as the initial guess
  i.e. the first predcition is equal to the intial guess for every class
"""
estimates = scans.iloc[0]
x_prev = estimates
#print(estimates)

#print(len(scans.index))
#print(len(scans.columns))

for i in range(len(scans.index)):
  """ The measurement at time step n (Zn) is processed """
  z_curr = scans.iloc[i]
  
  """ The value of the gain is updated after each iteration, considering
  the iteration step
 """
  k_gain = 1/(i+1)

  """ Using the State Update Equation, the current estimate is calculated """
  x_curr = x_prev + k_gain*(z_curr - x_prev)
  
  """ For a static model, the next state estimate (prediction) is equal to
    the current state estimate
  """
  x_prev = x_curr

#print(x_curr)

""" Plot of the LIDAR scans """

""" The resolution of the scan is 0.0065540750511 rads/step and the sensor
   makes 719 steps to retrieve 720 range values covering 270 degrees.
 """
res = 0.0065540750511
theta_max = 135
theta_min = -135

theta = np.linspace(-135.0,135.0,num = 720)
print(theta)
r = x_curr

area = r
colors = r

fig = plt.figure()
ax = fig.add_subplot(111,projection='polar')
c = ax.scatter(theta,r,c = colors,s = area,cmap = 'hsv',alpha = 0.75)
ax.set_thetamin(theta_min)
ax.set_thetamax(theta_max)
ax.set_theta_zero_location('W', offset=-90)
plt.show()
