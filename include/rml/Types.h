/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_TYPES_H_
#define INCLUDE_RML_TYPES_H_

#include <iostream>
#define EIGEN_MATRIXBASE_PLUGIN <rml/MatrixBaseAddons.h>
#include <eigen3/Eigen/Dense>

/**
 * \namespace Eigen
 *
 * \brief This namespace is used to extend the Eigen Dense library functionalities
 *
 * \details In order to maintain uniformity with the eigen library, the extensions of it have
 * been included in the Eigen namespace. In particular the additions regard the definition of
 * the following types:
 *
 *   1. Eigen::RotMatrix: represents a cartesian rotation matrix. Is an extension of the Matrix3d
 *   class that defaults the constructor to an identity matrix, with the addition of member
 *   functions to convert to different representations.
 *
 *   2. Eigen::TransfMatrix:: represents an homogeneous transformation matrix an extension of the
 *   Matrix4d class that defaults the constructor to an identity matrix, with the addition of member
 *   functions to convert do different representations, set and extract rotation and translational
 *   parts of it separately (rot and transl parts).
 *
 *   \note Full Eigen documentation can be found at http://eigen.tuxfamily.org/dox/index.html.
 */
namespace Eigen {
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6dBase;
}

// Forward Declarations
namespace Eigen {
class RotMatrix;
class TransfMatrix;
class Vector6d;
}

namespace rml {
class EulerRPY;
}

#include "EulerRPY.h"
#include "RotMatrix.h"
#include "TransfMatrix.h"
#include "Vector6d.h"

#endif /* INCLUDE_RML_TYPES_H_ */
