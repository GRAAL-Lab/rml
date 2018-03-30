/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_TYPES_H_
#define INCLUDE_RML_TYPES_H_

#include <iostream>
#define EIGEN_MATRIXBASE_PLUGIN <rml/MatrixBaseAddons.h>
#include <eigen3/Eigen/Dense>

namespace Eigen{
class RotMatrix;
class TransfMatrix;
}

namespace rml{

/**
 * @brief Euler RPY angle representation.
 *
 * This class stores the orientation of a rigid body in terms of the roll, pitch and yaw
 * rotation. The order in which the rotation are applied follows the z-y-x convention:
 * rotate around yaw, then pitch, then roll.
 */
class EulerRPY
{
public:
	EulerRPY();
	EulerRPY(double yaw, double pitch, double roll);
	EulerRPY(Eigen::Vector3d vec3);

	virtual ~EulerRPY() {}

	friend std::ostream& operator <<(std::ostream& os, EulerRPY const& a)
	{
		return os << a.roll_ <<"\t" << a.pitch_ << "\t" <<  a.yaw_ << "\t";
	}

	double GetYaw() const;
	void SetYaw(double yaw);

	double GetPitch() const;
	void SetPitch(double pitch);

	double GetRoll() const;
	void SetRoll(double roll);

	void SetRPY(double roll, double pitch, double yaw);
	void SetRPY(Eigen::Vector3d& vec3);

	Eigen::Vector3d ToVect3() const;
	Eigen::RotMatrix ToRotMatrix() const;
	Eigen::Vector3d GetDerivative(Eigen::Vector3d omega) const throw (std::exception);

private:
	double roll_, pitch_, yaw_ ;
};
}

/**
 * \namespace Eigen
 *
 * \brief This namespace is used to extend the Eigen Dense library functionalites
 *
 * \details In order to maintain uniformity with the eigen library, the extensions of it have
 * been included in the Eigen namespace. In particular the additions regard the definition of
 * the following types:
 *
 *   1. Eigen::RotMatrix: represents a cartesian rotation matrix. Is an extension of the Matrix3d
 *   class that defaults the constructor to an identity matrix, with the addition of member
 *   functions to convert to different representations.
 *
 *   2. Eigen::TransfMatrix:: represents an homogeneus transformation matrix an extension of the
 *   Matrix4d class that defaults the constructor to an identity matrix, with the addition of member
 *   functions to convert do different representaions, set and extract rotation and translational
 *   parts of it separately (rot and transl parts).
 *
 *   \note Full Eigen documentation can be found at http://eigen.tuxfamily.org/dox/index.html.
 */
namespace Eigen{

typedef Eigen::Matrix<double, 6, 1> Vector6dBase;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
//typedef Eigen::Matrix3d RotationMatrix;

/**
 * @brief A 6d vector that can contain a [r p y x y z] or an [wx wy wz x y z] vector
 */
class Vector6d : public Eigen::Vector6dBase
{
public:
	Vector6d(void):Eigen::Vector6dBase() {
		*this = Eigen::Vector6dBase::Zero();
	}
	// This constructor allows you to construct TransfMatrix from Eigen expressions
	template<typename OtherDerived>
	Vector6d(const Eigen::MatrixBase<OtherDerived>& other)
	: Eigen::Vector6dBase(other)
	{ }

	Vector6d(const Vector3d& first, const Vector3d& second){
		this->SetFirstVect3(first);
		this->SetSecondVect3(second);
	}
	// This method allows you to assign Eigen expressions to TransfMatrix
	template<typename OtherDerived>
	Vector6d& operator=(const Eigen::MatrixBase <OtherDerived>& other)
	{
		this->Eigen::Vector6dBase::operator=(other);
		return *this;
	}

	Vector3d GetFirstVect3() const {
		return this->block(0,0,3,1);
	}

	Vector3d GetSecondVect3() const {
		return this->block(3,0,3,1);
	}

	template<typename OtherDerived>
	void SetFirstVect3(const Eigen::MatrixBase <OtherDerived>& vec3) {
		this->block(0,0,3,1) = vec3;
	}

	template<typename OtherDerived>
	void SetSecondVect3(const Eigen::MatrixBase <OtherDerived>& vec3) {
		this->block(3,0,3,1) = vec3;
	}

	TransfMatrix ToTransfMatrix() const;
};

/**
 * \class RotMatrix
 *
 * \brief This class extends the Eigen::Matrix3d
 *
 * \details The Eigen::RotMatrix represents a cartesian rotation matrix. Is an extension of
 * the Matrix3d class that defaults the constructor to an identity matrix, with the addition
 * of member functions to convert to different representations such as: Cartesian 6x6 rotation
 * matrix, rml::EulerRPY and Eigen::Quaterniond reprensentation.
 *
 */
class RotMatrix : public Eigen::Matrix3d
{
public:
	RotMatrix(void):Eigen::Matrix3d() {
		*this = Eigen::Matrix3d::Identity();
	}
	// This constructor allows you to construct TransfMatrix from Eigen expressions
	template<typename OtherDerived>
	RotMatrix(const Eigen::MatrixBase<OtherDerived>& other)
	: Eigen::Matrix3d(other)
	{ }
	// This method allows you to assign Eigen expressions to TransfMatrix
	template<typename OtherDerived>
	RotMatrix& operator=(const Eigen::MatrixBase <OtherDerived>& other)
	{
		this->Eigen::Matrix3d::operator=(other);
		return *this;
	}

	RotMatrix Transpose() const {
		return this->transpose();
	}

	Eigen::Matrix6d GetCartesianRotationMatrix() const {
		Eigen::Matrix6d crmat;
		crmat.block(0,0,3,3) = crmat.block(3,3,3,3) = *this;
		crmat.block(0,3,3,3) = crmat.block(3,0,3,3) = Eigen::Matrix3d::Zero();
		return crmat;
	}

	rml::EulerRPY ToEulerRPY() const{
		return this->eulerAngles(0, 1, 2);
	}

	Eigen::Quaterniond ToQuaternion() const{
		return Eigen::Quaterniond(*this);
	}
};

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
class TransfMatrix : public Eigen::Matrix4d
{
public:
	TransfMatrix(void):Eigen::Matrix4d() {
		*this = Eigen::Matrix4d::Identity();
	}
	// This constructor allows you to construct TransfMatrix from Eigen expressions
	template<typename OtherDerived>
	TransfMatrix(const Eigen::MatrixBase<OtherDerived>& other)
	: Eigen::Matrix4d(other)
	{ }
	// This method allows you to assign Eigen expressions to TransfMatrix
	template<typename OtherDerived>
	TransfMatrix& operator=(const Eigen::MatrixBase <OtherDerived>& other)
	{
		this->Eigen::Matrix4d::operator=(other);
		return *this;
	}

	RotMatrix GetRotMatrix() const {
		return this->block(0,0,3,3);
	}

	void SetRotMatrix(const RotMatrix& rot) {
		this->block(0,0,3,3) = rot;
	}

	Vector3d GetTransl() const {
		return this->block(0,3,3,1);
	}

	void SetTransl(const Vector3d& transl) {
		this->block(0,3,3,1) = transl;
	}

	Vector6d GetRPYXYZ() const {
		Vector6d a;
		a.SetFirstVect3(this->GetRotMatrix().ToEulerRPY().ToVect3());
		a.SetSecondVect3(this->GetTransl());
		return a;
	}
};

}




#endif /* INCLUDE_RML_TYPES_H_ */
