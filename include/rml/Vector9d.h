/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_VECTOR9D_H_
#define INCLUDE_VECTOR9D_H_

#include "Types.h"
namespace Eigen
{

/**
 * @brief A 6d vector generally used for containing pose [x y z r p y] or velocity [wx wy wz vx vy vz] vectors
 */
class Vector9d: public Eigen::Vector9dBase
{
public:
	Vector9d();
	// This constructor allows you to construct TransfMatrix from Eigen expressions
	template<typename OtherDerived>
	Vector9d(const Eigen::MatrixBase<OtherDerived>& other) :
			Eigen::Vector9dBase(other)
	{
	}

	// This method allows you to assign Eigen expressions to TransfMatrix
	template<typename OtherDerived>
	Vector9d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Eigen::Vector9dBase::operator=(other);
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

	template<typename OtherDerived>
	void SetThirdVect3(const Eigen::MatrixBase<OtherDerived>& vec3)
	{
		this->block(6, 0, 3, 1) = vec3;
	}

	Vector9d(const Vector3d& first, const Vector3d& second, const Vector3d& third);

	Vector3d GetFirstVect3() const;

	Vector3d GetSecondVect3() const;

	Vector3d GetThirdVect3() const;

};


}

#endif /* INCLUDE_VECTOR9D_H_ */
