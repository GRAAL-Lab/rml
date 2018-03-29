/*
 *  RML.h
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 *
 *      @defgroup RML RoboticMathLib
 */

/**
 *  @brief Types and algorithms for robotic mobile manipulation
 *
 *  @details The "Robotic Mathematical Library" is a minimal and portable math lib for robotics that relies
 *  on Eigen. It includes tools for matrix pseudo inversion and consequently the SVD algorithm. It
 *  also provides utility functions for type conversion and data extraction for transformation and
 *  rotation matrices, rpy representation, rigid body matrices and matrix juxtaposition.
 */

#ifndef INCLUDE_RML_RML_H_
#define INCLUDE_RML_RML_H_

#include "Types.h"
#include "MatrixOperations.h"
#include "Functions.h"
#include "SVD.h"
#include "PseudoInverse.h"
#include "RobotModel.h"
#include "NewtonEuler.h"

#endif /* INCLUDE_RML_RML_H_ */
