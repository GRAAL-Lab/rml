/*
 * Vector6d.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "Vector6d.h"

namespace Eigen
{

Vector6d::Vector6d() :
		Eigen::Vector6dBase()
{
	*this = Eigen::Vector6dBase::Zero();
}

Vector6d::Vector6d(const Vector3d& first, const Vector3d& second)
{
	this->SetFirstVect3(first);
	this->SetSecondVect3(second);
}

Vector3d Vector6d::GetFirstVect3() const
{
	return this->block(0, 0, 3, 1);
}

Vector3d Vector6d::GetSecondVect3() const
{
	return this->block(3, 0, 3, 1);
}

Eigen::TransfMatrix Vector6d::ToTransfMatrix() const
{
	Eigen::TransfMatrix TMat;
	TMat.SetRotMatrix(rml::EulerRPY(this->GetFirstVect3()).ToRotMatrix());
	TMat.SetTransl(this->GetSecondVect3());
	return TMat;
}



} // namespace Eigen

