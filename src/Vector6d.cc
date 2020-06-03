/*
 * Vector6d.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "Vector6d.h"

namespace Eigen {

Vector6d::Vector6d()
    : Eigen::Vector6dBase()
{
    *this = Eigen::Vector6dBase::Zero();
}

Vector6d::Vector6d(const Vector3d linear, const Vector3d angular)
{
    this->LinearVector(linear);
    this->AngularVector(angular);
}
} // namespace Eigen
