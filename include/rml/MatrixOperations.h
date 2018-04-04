/**
 * \file
 *
 * \date 	Feb 16, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_MATRIXOPERATIONS_H_
#define INCLUDE_RML_MATRIXOPERATIONS_H_

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

#include "Types.h"

namespace rml {

inline void Double2Matrix(double dmat[], int rows, int cols, Eigen::MatrixXd& MatT){
	MatT = Eigen::Map<Eigen::MatrixXd>(dmat, rows, cols);
}

inline void Matrix2Double(const Eigen::MatrixXd& MatT, int rows, int cols, double dmat[]){
	Eigen::Map<Eigen::MatrixXd>(dmat, rows, cols) = MatT;
}

inline void Double2Vector(double dmat[], int rows, Eigen::VectorXd& MatT){
	MatT = Eigen::Map<Eigen::VectorXd>(dmat, rows, 1);
}

inline void Vector2Double(const Eigen::VectorXd& MatT, int rows, double dmat[]){
	Eigen::Map<Eigen::VectorXd>(dmat, rows, 1) = MatT;
}

inline void SetDiagonalFromDouble(Eigen::MatrixXd& MatT, double diag[]){
	Eigen::ArrayXd wdiag = Eigen::Map<Eigen::ArrayXd>( diag, MatT.cols() );
	MatT.diagonal() = wdiag;
}

inline Eigen::MatrixXd RightJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	Eigen::MatrixXd res = A;
	if(res.rows() == 0 && res.cols() == 0){
		res.resize(B.rows(), Eigen::NoChange);
	}
	eigen_assert(res.rows() == B.rows());
	res.conservativeResize(Eigen::NoChange, A.cols() + B.cols());
	res.block(0, A.cols(), B.rows(), B.cols()) = B;
	return res;
}

/*inline Eigen::MatrixXd LeftJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	return RightJuxtapose(B, A);
}
*/

inline Eigen::MatrixXd UnderJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	Eigen::MatrixXd res = A;
	if(res.rows() == 0 && res.cols() == 0){
		res.resize(Eigen::NoChange, B.cols());
	}
	eigen_assert(res.cols() == B.cols());
	res.conservativeResize(A.rows() + B.rows(), Eigen::NoChange);
	res.block(A.rows(), 0, B.rows(), B.cols()) = B;
	return res;
}

/*inline Eigen::MatrixXd UpperJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	return UnderJuxtapose(B, A);
}*/
/*
inline Eigen::Vector3d GetFirstVect3(const Eigen::Vector6d& vect6) {
	return vect6.block(0,0,3,1);// = vect3;
}

inline Eigen::Vector3d GetSecondVect3(const Eigen::Vector6d& vect6) {
	return vect6.block(3,0,3,1);// = vect3;
}

inline void SetFirstVect3(Eigen::Vector6d& vect6, const Eigen::Vector3d& vect3){
	vect6.block(0,0,3,1) = vect3;
}

inline void SetSecondVect3(Eigen::Vector6d& vect6, const Eigen::Vector3d& vect3){
	vect6.block(3,0,3,1) = vect3;
}

inline Eigen::Vector3d GetTransl(Eigen::Matrix4d& tMat){
	return tMat.block(0,3,3,1);
}

inline Eigen::Matrix3d GetRotMatrix(Eigen::Matrix4d& tMat){
	return tMat.block(0,0,3,3);
}*/


}


#endif /* INCLUDE_RML_MATRIXOPERATIONS_H_ */
