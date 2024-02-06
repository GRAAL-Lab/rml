/*
 * RotMatrix.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "RotMatrix.h"

namespace Eigen {

RotationMatrix::RotationMatrix()
    : Eigen::Matrix<double,3,3,Eigen::DontAlign>()
{
    *this = Eigen::Matrix3d::Identity();
}

RotationMatrix::RotationMatrix(Eigen::Quaterniond q)
    : Eigen::Matrix<double,3,3,Eigen::DontAlign>()
{
    *this = q.toRotationMatrix();
}

Eigen::Matrix6d RotationMatrix::CartesianRotationMatrix() const
{
    Eigen::Matrix6d crmat;
    crmat.block(0, 0, 3, 3) = crmat.block(3, 3, 3, 3) = *this;
    crmat.block(0, 3, 3, 3) = crmat.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
    return crmat;
}

rml::EulerRPY RotationMatrix::ToEulerRPY() const
{
    Eigen::Vector3d ypr = this->eulerAngles(2, 1, 0);
    return rml::EulerRPY(ypr(2), ypr(1), ypr(0));
}

Eigen::Quaterniond RotationMatrix::ToQuaternion() const
{
    return Eigen::Quaterniond(*this);
}

//Computes the integral of a rotation matrix (Out = e^[wdt^] * Rin )
RotationMatrix RotationMatrix::StrapDown(const Vector3d& w, double dt) const
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd((w * dt).norm(), (w * dt));
    R *= (*this);
    return R;
}

} // namespace Eigen
