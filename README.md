# RML: Robotic Mathematical Library 
This library is built on top of the *Eigen* library to provide all the necessary tools to operate with mobile robotic manipulators.

## Features
Its major features are: 

0. Definition of Rotation and Transformation matrices, Euler angles (using the zyx convention, i.e. the yaw-pitch-roll order), and all the functions to convert from one type to another.
0. Implementation of a very fast SVD algorithm (for robot-wise matrix dimensions) with optional regularization parameters. 
0. Implementation of a Pseudo Inversion algorithm. 
0. Implementation of a slim Robot Model class able to manage multi arm mobile robotic platforms, including dJdq and manipulability measurements. 
0. Providing a set of robotic related functions such as: lemma versor, bell shaped functions, point to plane distance and more. 

## Dependencies 

- Eigen 3
