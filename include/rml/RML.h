/*
 * RML.h
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 */

/**
 *  The "Robotic Mathematical Library" is a minimal and portable math lib for robotics that relies
 *  on Eigen. It includes tools formatrix pseudo inversion and consequently the SVD algorithm. It
 *  also provides utility functions for type conversion and data extraction for transformation and
 *  rotation matrices, rpy representation, rigid body matrices and matrix juxtaposition.
 */

#ifndef INCLUDE_RML_RML_H_
#define INCLUDE_RML_RML_H_

#define EIGEN_MATRIXBASE_PLUGIN <rml/MatrixBaseAddons.h>

#include <rml/Futils.h>
#include <rml/PseudoInverse.h>
#include <rml/MatrixOperations.h>


#endif /* INCLUDE_RML_RML_H_ */
