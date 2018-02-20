/*
 * Defines.h
 *
 *  Created on: Feb 20, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_DEFINES_H_
#define INCLUDE_RML_DEFINES_H_

#include <eigen3/Eigen/Dense>

namespace Eigen{

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::RotationBase<double, 3> RotationMatrix;

}



#endif /* INCLUDE_RML_DEFINES_H_ */
