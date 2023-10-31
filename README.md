# Quadrotor-Autonomy-Stack-with-Trajectory-Optimization-and-Control

###### Author: Sahachar Reddy Tippana

The files in the package "proj3" contains work done by Sahachar on Visual Inertial Odomoetry (VIO) based autonomous indoor quadrotor navigation as part of coursework at the University of Pennsylvania. This work integrates a non-linear geometric controller with linear backstepping, VIO based state estimation, Planning using A-star and Dijkstra's, Trajectory Optimization using Minimum-Snap and custom algorithms to precisely track the CrazyFlie 2.0 quadcopter, using ROS interfacing, which is evaluated against the Vicon groundtruth. 

## Usage
Make sure the dependencies listed in the setup.py are installed. Run proj3/code/sandbox.py to test the maps from proj3/util directory. We use the flighsim simulator to simulate the quadrotor in a custom environment and visualize the trajectory tracking.

## Performance
The key factors accounting to aggressive maneuverabilty and high performance by flight time achieved are reported in the Project_part2_report.pdf.
