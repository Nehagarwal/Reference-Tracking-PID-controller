# Reference-Tracking-PID-controller
This repository includes PID controller design for reference path tracking on a unicycle vehicle model. This is a very popular question asked in Control Systems Coding challenges. The logic used in this problem can be applied to Motion planning and tracking of autonomous vehicles.
## Vehicle Model
dot(x) = v0*cos(theta)

dot(y) = v0*sin(theta)

dot(theta) = v0*tan(u)/L
 
where, x: car's x position
       y: car's y position
       theta: heading angle theta == 0,  car in x direction
                      theta == PI/2, car in y direction
       L: wheelbase ==  2m,
       v0: velocity == 1m/s.
## Function Descriptions
- CarPose Step: returns the new pose of the car by integrating the current pose forwards by timestep seconds when applying a given
                steering angle to the car at a given velocity using the first order forward Euler method.
- double cross: returns the cross product of vectors
- double point_to_line: returns the perpendicular distance between a point and a line
- void Simulate:simulates the car tracking the refernce path by setting the car's steering angles, PID controller is implemeneted to track                 the reference trajectory.

### Cross track error calculation
- Perpendicular distance is calculated between the current position and two consecutive points on reference trajectory.
- In order to decide left or right steer, right hand thumb rule (cross product) is used to find the sign.

### Note
Ki, Kp, Kd are examples and not the correct values, needs to be tuned.

### Install
The code is written in C++ and so can be easily written using gedit or Eclipse.
