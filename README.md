Quarotor-Autonomy-Stack-with-Trajectory-Optimization-and-Control
========================================

###### Author: Sahachar Reddy Tippana

These files contain Sahachar Reddy Tippana's work 

1. dijkstra.m

2. trajectory_generator.m

3. crazyflie.m

4. controller.m

   

## DIJKSTRA.m

This 

1. Data

d to improve the function of the 1.2 Dijkstra implementation.


## TRAJECTORY_GENERATOR.m


2. A maximum acceleration is assigned (based on experiments), and this value is used to calculate the time required for each step, assuming acceleration from 0 and deceleration to 0. That is:
   $$
   \Delta t = 2\sqrt{\frac{d}{a}}
   $$
   

   where we have used the kinematic equation 
   $$
   d = \frac{1}{2} a t^2
   $$
   and assumed that we want to be at the half-way point in distance at the half-way point in time (credit to Luca Scheuer for this method). Although the quadrotor is not accelerating  from and decelerating to zero on every segment, this approaches gives sufficient time to accelerate and decelerate overall.




## CREDITS

