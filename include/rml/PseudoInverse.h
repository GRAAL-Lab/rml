/*
 * PseudoInverse.h
 *
 *  Created on: Feb 15, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_PSEUDOINVERSE_H_
#define INCLUDE_RML_PSEUDOINVERSE_H_

#include <eigen3/Eigen/Dense>

namespace rml {

/**
 * @internal for internal use only
 *
 * @brief Computes the SVD-based regularized matrix pseudoinversion (a = U*S*V')
 */
void GT_RegPinv(const double *J, int m, int n, double *JPInv, double treshold, double lambda, double* prod, int* flag);



template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> RegularizedPseudoInverse(
		const MatT &mat, const double threshold, const double lambda, double *mu, int *flag) // choose appropriately
		{

	int m = mat.rows(), n = mat.cols();
	double J[m * n];                // NULL pointer
	double JPInv[n * m];

	/**
	 * Here we convert the input type to a double array which is the type used by the GT_RegPinv
	 */
	Eigen::Map<MatT>(J, m, n) = mat;

	//double mu;
	//int flag;

	GT_RegPinv(J, m, n, JPInv, threshold, lambda, mu, flag);

	/**
	 * Here the results of the GT_RegPinv algorithm are mapped back to the input type
	 */
	MatT eigenPinv = Eigen::Map<MatT>(JPInv, n, m);

	return eigenPinv;
}

template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> RegularizedPseudoInverse(
		const MatT &mat, const double threshold, const double lambda, double *mu) // choose appropriately
		{

	int flag;
	return RegularizedPseudoInverse(mat, threshold, lambda, mu, &flag);
}

template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> RegularizedPseudoInverse(
		const MatT &mat, const double threshold, const double lambda) // choose appropriately
		{

	double mu;
	int flag;
	return RegularizedPseudoInverse(mat, threshold, lambda, &mu, &flag);

}

} //namespace rml

#endif /* INCLUDE_RML_PSEUDOINVERSE_H_ */
