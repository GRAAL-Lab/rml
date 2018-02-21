/*
 * Defines.h
 *
 *  Created on: Feb 20, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_DEFINES_H_
#define INCLUDE_RML_DEFINES_H_

#include <eigen3/Eigen/Dense>

namespace Eigen{

typedef Eigen::Matrix<double, 6, 1> Vector6d;
//typedef Eigen::Matrix3d RotationMatrix;


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

    RotMatrix GetRotMatrix() {
    	return this->block(0,0,3,3);
    }

    Vector3d GetTransl() {
    	return this->block(0,3,3,1);
    }

    // This method allows you to assign Eigen expressions to TransfMatrix
    template<typename OtherDerived>
    TransfMatrix& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Matrix4d::operator=(other);
        return *this;
    }
};

}




#endif /* INCLUDE_RML_DEFINES_H_ */
