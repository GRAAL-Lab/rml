# RML: Robotics Mathematical Library
The **RML** library is built on top of the *Eigen* library to provide all the necessary tools to operate with mobile robotic manipulators.

## Documentation
Full doxygen-generated documentation can be found at: https://merosss.bitbucket.io/rml.

## Features
Its major features are:

1. Definition of derived Eigen types such as **Rotation** and **Homogeneus Transformation** matrices (Eigen::RotMatrix and Eigen::TransfMatrix), **Euler angles** (rml::EulerRPY, using the z-y-x convention, i.e. the yaw-pitch-roll rotations order), and all the functions to convert from one type to another.
2. Implementation of a very fast **SVD algorithm** (for robot-wise matrix dimensions) with optional regularization parameters (rml::SVD()).
3. Implementation of a **Pseudo-Inversion** algorithm (rml::RegularizedPseudoInverse()).
4. Implementation of a slim **RobotModel** class able to manage multi arm mobile robotic platforms (rml::RobotModel), in which we can load a rml::VehicleModel and many rml::ArmModel(s), providing frame and jacobian utilities, also including dJdq and manipulability measurements.
5. Implementation of the **Newton-Euler** algorithm (rml::NewtonEuler), with some utility functions to simulate dynamic systems.
6. Providing a set of robotic related functions such as: lemma versor, bell shaped functions, point to plane distance and more ().

Check out the doxygen ::rml namespace documentation for a brief overview of all functionalities.
Where not explicitly stated so, the default units are the SI ones (meters, kg, seconds, etc.).

## Dependencies
Before building the repository you will have to install the following dependencies:

* Eigen 3 (`sudo apt install libeigen3-dev`)


## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install
    
The CMakeLists.txt provides also an additional BUILD_TESTS option, which by default is set to OFF. If you want to build also the tests just run:
    
    $ cmake .. -DBUILD_TESTS=ON
