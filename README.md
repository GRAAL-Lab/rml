//! \mainpage Main Page

# RML: Robotic Mathematical Library
This library is built on top of the *Eigen* library to provide all the necessary tools to operate with mobile robotic manipulators.

## Features
Its major features are:

1. Definition of derived Eigen types such as **Rotation** and **Homogeneus Transformation** matrices (Eigen::RotMatrix and Eigen::TransfMatrix), **Euler angles** (rml::EulerYPR, using the zyx convention, i.e. the yaw-pitch-roll order), and all the functions to convert from one type to another.
2. Implementation of a very fast **SVD algorithm** (for robot-wise matrix dimensions) with optional regularization parameters (rml::SVD).
3. Implementation of a **Pseudo-Inversion** algorithm (rml::RegularizedPseudoInverse).
4. Implementation of a slim **RobotModel** class able to manage multi arm mobile robotic platforms, including dJdq and manipulability measurements.
5. Providing a set of robotic related functions such as: lemma versor, bell shaped functions, point to plane distance and more.


## Dependencies
Before building the repository you will have to install the following dependencies:

* Eigen 3 (`sudo apt install libeigen3-dev`)


## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install
