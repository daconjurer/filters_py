from math import *
import matplotlib.pyplot as plt
import numpy as np

""" For randomness """
from random import seed
from random import random
from random import gauss

""" One dimensional Standard Kalman filter for a system
  with a static model

  Kalman filter relies on five sets of equations:
  - State Update equation
  - State Extrapolation equations (AKA Dynamic model)
  - Kalman Gain equation
  - Covariance Update equation
  - Covariance Extrapolation equation

  Kalman filter algorithm outline:
  - Step 0 (Initialization):
      This step provides two parameters:
        - Initial System State
        - Initial System Uncertainty
    A prediction should be made using these parameters
  - Step 1 (Measurement):
      This step provides two parameters:
        - Measured System State
        - Measurement Uncertainty
  - Step 2 (State Update):
      The system's current state estimation is made in this step
      INPUTS:
        - Measurement
        - Measurement Uncertainty
        - Previous System State Estimate
        - Estimate Uncertainty
      OUTPUTS:
        - Current System's State Estimate
        - Current State Estimate Uncertainty
  - Step 3 (Prediction):
      The prediction process extrapolates the current system state
      and the uncertainty of the current system estiamate to the next
      system state, based on the system's dynamics.
"""

""" The system will be a point moving in one dimension with constant
  velocity, the estimates are computed using the constant speed motion
  model and (noisy) position measurements
"""

""" Gaussian function """
def f(mu, sigma2, x):
  ''' f takes in a mean and squared variance, and an input x
    and returns the corresponding gaussian value.'''
  coefficient = 1.0 / sqrt(2.0 * pi *sigma2)
  exponential = exp(-0.5 * (x-mu) ** 2 / sigma2)
  return coefficient * exponential

""" 
Dynamic motion model:
x_curr = x_prev + v_prev * delta_t + w
x_curr = A*x_prev + B*u + w

transition matrix A = 1
control input matrix B = delta_t
w models the noise of the process with a 0 mean normal distribution
and covariance Q (could be, for example, the unknown value of the
instant acceleration at every time step if we consider that speed
is not perfectly constant)

Sensor model:
z = H*x_curr + v

where z is the sensor measurement, H is the transformation matrix,
x_curr is the current state and v models the measurement noise with
a 0 mean normal distribution with covariance R

"""

""" System definition """
dt = 0.1    # time step
X = 0       # temp state vector (1D)
A = 1       # state transition matrix
A_t = A
B = dt      # control input matrix
P = 4       # std_dev^2 = 10^2
Q = 0.1     # process noise covariance matrix
R = 0.15    # measurement noise covariance matrix
H = 1       # sensor transformation matrix
H_t = H
u = 2       # control input (2 m/s)

seed(1)
k = 50                  # number of steps
xx_pred = np.zeros(k)   # new state prediction
xx = np.zeros(k)        # corrected state vector
PP_pred = np.zeros(k)   # predicted state uncertainty vector
PP = np.zeros(k)        # corrected state uncertainty vector
XX = np.zeros(k)        # system state vector
tt = np.zeros(k)        # time vector
yy = np.zeros(k)        # measurements array

""" Simulation """
for n in range(k):
  w = sqrt(Q)*(gauss(0,0.2)/(1.0/sqrt(2.0*pi*(0.2**2))))
  X = X + u*dt + w
  v = sqrt(R)*(gauss(0,0.5)/(1.0/sqrt(2.0*pi*(0.5**2))))
  y = H*X + v
  
  """ Prediction step """
  x_pred = A*X + B*u
  P = A*P*A_t + Q
  PP_pred[n] = P    # Save the prediction uncertainty
  
  """ Update step """
  e = H*x_pred      # expectation
  E = H*P*H_t       # covariance of expectation
  z = y - e         # innovation
  Z = R + E
  K = P*H_t*(1/Z)
  
  x = x_pred + K*z  # final corrected state
  P = P - K*H*P     # uncertainty in the corrrected state
  
  """ Outputs saving """
  xx_pred[n] = x_pred
  xx[n] = x
  PP[n] = P
  XX[n] = X
  tt[n] = n+1
  yy[n] = y

""" Results display """
for n in range(k):
  print('Predict {}: [{},{}]'.format(n+1,xx_pred[n],PP_pred[n]))
  print('Update {}: [{},{}]'.format(n+1,xx[n],PP[n]))

""" Observations and corrected state display """
x_axis = np.arange(-5,15,0.1)
predicted = np.zeros((k,len(x_axis)))
corrected = np.zeros((k,len(x_axis)))

fig0 = plt.figure()
fig0.add_subplot(131)
plt.scatter(tt,XX,color = 'blue')
fig0.add_subplot(132)
plt.scatter(tt,yy,color = 'green')
fig0.add_subplot(133)
plt.scatter(tt,xx,color = 'red')

plt.show()

fig1 = plt.figure()
for n in range(k):
  for m in range(len(x_axis)):
    predicted[n][m] = f(xx_pred[n],PP_pred[n],x_axis[m])

  # plot the results
  plt.plot(x_axis,predicted[n],color = 'blue')
  plt.pause(0.1)

fig2 = plt.figure()
for n in range(k):
  for m in range(len(x_axis)):
    corrected[n][m] = f(xx[n],PP[n],x_axis[m])

  # plot the results
  plt.plot(x_axis,corrected[n],color = 'red')
  plt.pause(0.1)
