/*
 * MatrixOperations.h
 *
 *  Created on: Feb 16, 2018
 *      Author: fw
 */

#ifndef INCLUDE_RML_MATRIXOPERATIONS_H_
#define INCLUDE_RML_MATRIXOPERATIONS_H_

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <rml/Futils.h>

namespace rml {

template<class MatT>
void PrintMatrix(const MatT& A, std::string name){
	std::cout << tc::white << name << tc::none << std::endl << A << std::endl;
}

inline void SetDiagonalFromDouble(Eigen::MatrixXd& MatT, double diag[]){
	Eigen::ArrayXd wdiag = Eigen::Map<Eigen::ArrayXd>( diag, MatT.cols() );
	MatT.diagonal() = wdiag;
}

//template<class MatT>
inline Eigen::MatrixXd RightJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	Eigen::MatrixXd res;
	eigen_assert(A.rows() == B.rows());
    res = A;
	res.conservativeResize(Eigen::NoChange, A.cols() + B.cols());
	res.block(0, A.cols(), B.rows(), B.cols()) = B;
	return res;
}

inline Eigen::MatrixXd LeftJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	return RightJuxtapose(B, A);
}

inline Eigen::MatrixXd UnderJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	Eigen::MatrixXd res;
	eigen_assert(A.cols() == B.cols());
    res = A;
	res.conservativeResize(A.rows() + B.rows(), Eigen::NoChange);
	res.block(A.rows(), 0, B.rows(), B.cols()) = B;
	return res;
}

inline Eigen::MatrixXd UpperJuxtapose(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
	return UnderJuxtapose(B, A);
}

}


#endif /* INCLUDE_RML_MATRIXOPERATIONS_H_ */
