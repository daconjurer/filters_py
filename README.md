filters_py
=========

This repo contains some optimal filters implemented in Python 3.6.6. The repo relies on an Anaconda virtual environment with Python3.6.6 and the following python modules:
 - numpy
 - pandas
 - matplotlib

The scripts take in the previously formatted ROS data (.bag files converted into .csv files)

## Overview

The first filter *static_system_estimator*, as the name says, estimates the values of a range of laser scans retrieved by a SICK LMS111 LIDAR sensor in a ROS Kinetic/Gazebo 7 simulation. During the simulation, the *Jackal robot* that integrates the laser does not move and therefore the readings should be the same. The filter is used to estimate more accurate ranges.

## Authors

* **Victor Sandoval** - [daconjurer](https://github.com/daconjurer)

## Acknowledgments

* The *static_system_estimator* uses data collected by running the [Jackal robot](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html) simulation node on Gazebo.
