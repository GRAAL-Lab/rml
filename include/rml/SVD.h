/**
 * \file
 *
 * \date 	Feb 17, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_SVD_H_
#define INCLUDE_RML_SVD_H_

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "MatrixOperations.h"

namespace rml{

/**
 * @internal for internal use only
 *
 * @brief Computes the SVD decomposition of a matrix a[m*n]	(a = U*S*V')
 *
 * The matrix V (NOT the transpose V') replaces "a" on output.
 * The diagonal matrix of singular values S is output as a vector w[1..n].
 * The matrix U is output as V_[n*n].
 */
void SVD_NumericalRecipes(double *a, int m, int n, double *w, double *v, double *rv1);


/**
 *
 * @brief Singular Value Decomposition
 *
 * The methods computes the singular value decomposition of the given matrix A, defined as
 * A = USV'. If A is of size m x n, then U is m x m, S is m x n and V is n x n
 *
 * @param[in] A the matrix to be decomposed
 * @param[out] U the m x m rotation matrix
 * @param[out] S the m x n singular values matrix
 * @param[out] V the n x n rotation matrix
 */
template<class MatT>
void SVD(const MatT& A, MatT& U, MatT& S, MatT& V) {

	int m = A.rows(), n = A.cols();
	int MaxMatDim = std::pow(std::max(m,n),2);
	double SVD[3 * MaxMatDim];
	double w[MaxMatDim];

	int index[MaxMatDim];

	Eigen::MatrixXd Utmp;
	Eigen::MatrixXd Vtmp;

	double U_temp[MaxMatDim], V_temp[MaxMatDim];

	Eigen::Map<MatT>(U_temp, m, n) = A;

	SVD_NumericalRecipes(U_temp, m, n, w, V_temp, SVD);

	Utmp = Eigen::Map<MatT>(U_temp, m, m);
	Vtmp = Eigen::Map<MatT>(V_temp, n, n);

	for (int i = 0; i < MaxMatDim; i++)
		index[i] = i;// + 1;

	double max = 0;
	int index_max;
	for (int i = 0; i < n; i++) {
		max = w[i];
		index_max = i;
		for (int j = i+1; j < n; j++) {
			if (w[j] > max) {
				index_max = j;
				max = w[j];
			}
		}

		double tmp = w[i];
		int tmpindex = index[i];
		w[i] = w[index_max];
		index[i] = index[index_max];
		w[index_max] = tmp;
		index[index_max] = tmpindex;
	}

	Vtmp.conservativeResize(n, n);
	Utmp.conservativeResize(m, std::max(m, n));

	U = Utmp.block(0, index[0], m, 1);
	V = Vtmp.block(0, index[0], n, 1);

	for (int i = 1; i < n; i++) {
		V = RightJuxtapose(V, Vtmp.block(0, index[i], n, 1));
	}

	for (int i = 1; i < m; i++) {
		U = RightJuxtapose(U, Utmp.block(0, index[i], m, 1));
	}

	U = U.block(0, 0, m, m);

	S.resize(m, n);
	S.setZero();

	SetDiagonalFromDouble(S,w);

	S = S.block(0, 0, m, n);
}

}



#endif /* INCLUDE_RML_SVD_H_ */
