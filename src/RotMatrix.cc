/*
 * RotMatrix.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "RotMatrix.h"

namespace Eigen
{

RotMatrix::RotMatrix() :
		Eigen::Matrix3d()
{
	*this = Eigen::Matrix3d::Identity();
}

RotMatrix RotMatrix::Transpose() const
{
	return this->transpose();
}

Eigen::Matrix6d RotMatrix::GetCartesianRotationMatrix() const
{
	Eigen::Matrix6d crmat;
	crmat.block(0, 0, 3, 3) = crmat.block(3, 3, 3, 3) = *this;
	crmat.block(0, 3, 3, 3) = crmat.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	return crmat;
}

rml::EulerRPY RotMatrix::ToEulerRPY() const
{
	return static_cast<rml::EulerRPY>(this->eulerAngles(0, 1, 2));
}

Eigen::Quaterniond RotMatrix::ToQuaternion() const
{
	return Eigen::Quaterniond(*this);
}

//Computes the integral of a rotation matrix (Out = e^[wdt^] * Rin )
RotMatrix RotMatrix::StrapDown(const Vector3d& w, double dt) const
{
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd((w * dt).norm(), (w * dt));
	R *= (*this);
	return R;
}



} // namespace Eigen

