/*
 * RotMatrix.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "RotMatrix.h"

namespace Eigen {

RotationMatrix::RotationMatrix()
    : Eigen::Matrix<double,3,3>()
{
    *this = Eigen::Matrix3d::Identity();
}

RotationMatrix::RotationMatrix(Eigen::Quaterniond q)
    : Eigen::Matrix<double,3,3>()
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
    const auto& R = *this;
    double yaw = std::atan2(R(1, 0), R(0, 0));
    double pitch = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    double roll = std::atan2(R(2, 1), R(2, 2));

    return rml::EulerRPY(roll, pitch, yaw);
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
