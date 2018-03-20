/*
 * PseudoInverse.h
 *
 *  Created on: Feb 15, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_PSEUDOINVERSE_H_
#define INCLUDE_RML_PSEUDOINVERSE_H_

#include <eigen3/Eigen/Dense>
#include <rml/SVD.h>
#include <rml/Types.h>

namespace rml
{

/**
 * @internal for internal use only
 *
 * @brief Computes the SVD-based regularized matrix pseudoinversion (A = U*S*V')
 */
void GT_RegPinv(const double *J, int m, int n, double *JPInv, double treshold, double lambda, double* prod, int* flag);

/**
 * @brief Computes the SVD-based regularized matrix pseudoinversion (A = U*S*V')
 *
 * @param mat			The matrix to be inverted
 * @param svdParams		The SVD decomposition parameters
 * @return				The pseudo-inverse matrix of \p mat
 */
template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> RegularizedPseudoInverse(
		const MatT& mat, SVDData& svdData) // choose appropriately
{

	int m = mat.rows(), n = mat.cols();
	double J[m * n];                // NULL pointer
	double JPInv[n * m];

	//Here we convert the input type to a double array which is the type used by the GT_RegPinv
	Eigen::Map<MatT>(J, m, n) = mat;

	GT_RegPinv(J, m, n, JPInv, svdData.params.threshold, svdData.params.lambda, &(svdData.results.mu), &(svdData.results.flag));

	//Here the results of the GT_RegPinv algorithm are mapped back to the input type
	MatT eigenPinv = Eigen::Map<MatT>(JPInv, n, m);

	return eigenPinv;
}

/**
 * @brief Computes the SVD-based matrix pseudoinversion without regularization (A = U*S*V')
 *
 * @param mat			The matrix to be inverted
 * @return				The pseudo-inverse matrix of \p mat
 */
template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> PseudoInverse(
		const MatT& mat)
{
	SVDData svdData;
	RegularizedPseudoInverse(mat, svdData);
}

} //namespace rml

#endif /* INCLUDE_RML_PSEUDOINVERSE_H_ */
