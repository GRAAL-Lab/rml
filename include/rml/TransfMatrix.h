/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_TRANSFMATRIX_H_
#define INCLUDE_TRANSFMATRIX_H_

#include "Types.h"
#include "rml_internal/Futils.h"

namespace Eigen
{

/**
 * \class TransfMatrix
 *
 * \brief This class extends the Eigen::Matrix4d
 *
 * \details The Eigen::TransfMatrix represents an homogeneus transformation matrix an extension of the
 *   Matrix4d class that defaults the constructor to an identity matrix, with the addition of member
 *   functions to convert do different representaions, set and extract rotation and translational
 *   parts of it separately (rot and transl parts).
 *
 */
class TransfMatrix: public Eigen::Matrix4d
{
public:
	TransfMatrix(void);

	// This constructor allows you to construct TransfMatrix from Eigen expressions
	template<typename OtherDerived>
	TransfMatrix(const Eigen::MatrixBase<OtherDerived>& other) :
			Eigen::Matrix4d(other)
	{
	}
	// This method allows you to assign Eigen expressions to TransfMatrix
	template<typename OtherDerived>
	TransfMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Eigen::Matrix4d::operator=(other);
		return *this;
	}

	RotMatrix GetRotMatrix() const;

    void SetRotMatrix(const Eigen::RotMatrix& rot);

	Vector3d GetTransl() const;

	void SetTransl(const Vector3d& transl);

	Vector6d GetRPYXYZ() const;

	TransfMatrix Integral(const Vector6d& vin, double dt) const;
};

}

#endif /* INCLUDE_TRANSFMATRIX_H_ */
