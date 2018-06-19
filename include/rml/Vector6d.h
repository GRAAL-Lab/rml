/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_VECTOR6D_H_
#define INCLUDE_VECTOR6D_H_

#include "Types.h"

namespace Eigen
{

/**
 * @brief A 6d vector generally used for containing pose [x y z r p y] or velocity [wx wy wz vx vy vz] vectors
 */
class Vector6d: public Eigen::Vector6dBase
{
public:
	Vector6d();
	// This constructor allows you to construct TransfMatrix from Eigen expressions
	template<typename OtherDerived>
	Vector6d(const Eigen::MatrixBase<OtherDerived>& other) :
			Eigen::Vector6dBase(other)
	{
	}

	// This method allows you to assign Eigen expressions to TransfMatrix
	template<typename OtherDerived>
	Vector6d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Eigen::Vector6dBase::operator=(other);
		return *this;
	}

	template<typename OtherDerived>
	void SetFirstVect3(const Eigen::MatrixBase<OtherDerived>& vec3)
	{
		this->block(0, 0, 3, 1) = vec3;
	}

	template<typename OtherDerived>
	void SetSecondVect3(const Eigen::MatrixBase<OtherDerived>& vec3)
	{
		this->block(3, 0, 3, 1) = vec3;
	}

	Vector6d(const Vector3d& first, const Vector3d& second);

	Vector3d GetFirstVect3() const;

	Vector3d GetSecondVect3() const;

	TransfMatrix ToTransfMatrix() const;
};


}

#endif /* INCLUDE_VECTOR6D_H_ */
