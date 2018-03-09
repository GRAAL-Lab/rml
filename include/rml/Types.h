/*
 * Defines.h
 *
 *  Created on: Feb 20, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_TYPES_H_
#define INCLUDE_RML_TYPES_H_

#include <iostream>
#define EIGEN_MATRIXBASE_PLUGIN <rml/MatrixBaseAddons.h>
#include <eigen3/Eigen/Dense>

/// Forward declaration
namespace Eigen{
class RotMatrix;
class TransfMatrix;
}

namespace rml{

/**
 * @brief Euler angle representation.
 *
 * This class stores the orientation of a rigid body using the z-y-x (yaw, pitch, roll) rotation order
 */
class EulerYPR
{
public:
	EulerYPR();
	EulerYPR(double yaw, double pitch, double roll);
	EulerYPR(Eigen::Vector3d vec3);

	virtual ~EulerYPR() {}

	friend std::ostream& operator <<(std::ostream& os, EulerYPR const& a)
	{
		return os << a.yaw_ << "\t" << a.pitch_ << "\t" << a.roll_ << "\t";
	}

	double GetYaw() const;
	void SetYaw(double yaw);

	double GetPitch() const;
	void SetPitch(double pitch);

	double GetRoll() const;
	void SetRoll(double roll);

	void SetYPR(double yaw, double pitch, double roll);
	void SetYPR(Eigen::Vector3d& vec3);

	Eigen::Vector3d ToVect3() const;
	Eigen::RotMatrix ToRotMatrix() const;

private:
	double yaw_, pitch_, roll_;
};
}

namespace Eigen{

typedef Eigen::Matrix<double, 6, 1> Vector6dBase;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
//typedef Eigen::Matrix3d RotationMatrix;

/**
 * @brief A 6d vector that can contain a [y p r x y z] or an [wx wy wz x y z] vector
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

	rml::EulerYPR ToEulerYPR() const{
		return this->eulerAngles(2, 1, 0);
	}
};


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

	Vector6d GetYPRXYZ() const {
		Vector6d a;
		a.SetFirstVect3(this->GetRotMatrix().ToEulerYPR().ToVect3());
		a.SetSecondVect3(this->GetTransl());
		return a;
	}
};

}




#endif /* INCLUDE_RML_TYPES_H_ */
