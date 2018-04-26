/*
 * TransfMatrix.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "TransfMatrix.h"

namespace Eigen
{


TransfMatrix::TransfMatrix(void) :
		Eigen::Matrix4d()
{
	*this = Eigen::Matrix4d::Identity();
}

RotMatrix TransfMatrix::GetRotMatrix() const
{
	return this->block(0, 0, 3, 3);
}

void TransfMatrix::SetRotMatrix(const RotMatrix& rot)
{
	this->block(0, 0, 3, 3) = rot;
}

Vector3d TransfMatrix::GetTransl() const
{
	return this->block(0, 3, 3, 1);
}

void TransfMatrix::SetTransl(const Vector3d& transl)
{
	this->block(0, 3, 3, 1) = transl;
}

Vector6d TransfMatrix::GetRPYXYZ() const
{
	Vector6d a;
	a.SetFirstVect3(this->GetRotMatrix().ToEulerRPY().ToVect3());
	a.SetSecondVect3(this->GetTransl());
	return a;
}

TransfMatrix TransfMatrix::Integral(const Vector6d& vin, double dt) const
{
	TransfMatrix temp = *this;
	temp.SetRotMatrix(GetRotMatrix().StrapDown(vin.GetFirstVect3(), dt));
	temp.SetTransl(vin.GetSecondVect3() * dt + GetTransl());
	return temp;
}



} // namespace Eigen

