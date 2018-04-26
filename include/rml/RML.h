/**
 * \file
 *
 * \date 	Feb 15, 2018
 * \author 	Francesco Wanderlingh
 */


#ifndef INCLUDE_RML_RML_H_
#define INCLUDE_RML_RML_H_

/**
 * \namespace rml
 *
 * \brief Types and algorithms for robotic mobile manipulation
 *
 * \details The "Robotic Mathematical Library" is a minimal and portable math lib for robotics that relies
 *  on Eigen. It includes tools for matrix pseudo inversion and the SVD algorithm. It
 *  also provides utility functions for type conversion and data extraction for transformation and
 *  rotation matrices, RPY angle representation, rigid body matrices and matrix juxtaposition.
 *
 *  **Types**:
 *  	- EulerRPY
 *  	- (Eigen::RotMatrix, Eigen::TransfMatrix and Eigen::Vector6d have been defined inside the Eigen namespace)
 *
 *  **Structs**:
 *  	- PlaneParameters
 *
 *  **Strong Enums**:
 *  	- ::JointType
 *
 *  **Classes**:
 *  	- RobotLink
 *		- ArmModel
 *		- VehicleModel
 *		- RobotModel
 *		- NewtonEuler
 *
 *  **Functions**:
 *  	- Algorithms:
 *			- SVD()
 *			- RegularizedPseudoInverse()
 *		- Geometric:
 *			- VersorLemma()
 *			- CartesianError()
 *			- DistancePointToPlane()
 *			- ClosestPointOnPlane()
 *			- GetRigidBodyMatrix()
 *
 *		- Utilities:
 *			- GreatestNormElement()
 *			- Vect3ToSkew()
 *			- SaturateScalar()
 *			- SaturateVector()
 *		 	- DecreasingBellShapedFunction()
 *			- IncreasingBellShapedFunction()
 *
 */

#include "Types.h"
#include "MatrixOperations.h"
#include "Functions.h"
#include "SVD.h"
#include "PseudoInverse.h"
#include "RobotModel.h"
#include "NewtonEuler.h"


#endif /* INCLUDE_RML_RML_H_ */
