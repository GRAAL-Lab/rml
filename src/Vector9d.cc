

#include "Vector9d.h"

namespace Eigen
{

Vector9d::Vector9d() :
		Eigen::Vector9dBase()
{
	*this = Eigen::Vector9dBase::Zero();
}

Vector9d::Vector9d(const Vector3d& first, const Vector3d& second, const Vector3d& third)
{
	this->SetFirstVect3(first);
	this->SetSecondVect3(second);
	this->SetThirdVect3(third);
}

Vector3d Vector9d::GetFirstVect3() const
{
	return this->block(0, 0, 3, 1);
}

Vector3d Vector9d::GetSecondVect3() const
{
	return this->block(3, 0, 3, 1);
}

Vector3d Vector9d::GetThirdVect3() const
{
	return this->block(6, 0, 3, 1);
}



} // namespace Eigen

