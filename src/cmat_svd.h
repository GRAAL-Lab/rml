#ifndef CMATSVD_H_
#define CMATSVD_H_

#include <cmat/cmat.h>

namespace CMATtest
{
	/**
	 * @brief Singular Value Decomposition
	 *
	 * The methods computes the singular value decomposition of the given matrix A, defined as
	 * A = USV'
	 * if A is of size m x n, then U is m x m, S is m x n and V is n x n
	 *
	 * @param[in] A the matrix to be decomposed
	 * @param[out] U the m x m rotation matrix
	 * @param[out] S the m x n singular values matrix
	 * @param[out] V the n x n rotation matrix
	 */
	void SVD(const CMAT::Matrix& A, CMAT::Matrix& U, CMAT::Matrix& S, CMAT::Matrix& V);

	/**
	 * @internal for internal use only
	 *
	 * @return  sqrt(a*a+b*b) (@see CMAT::GT_svdcmp)
	 */
	double GT_pythag(double a, double b);
	/**
	 * @internal for internal use only
	 *
	 * @brief Computes the SVD decomposition of a matrix a[m*n]	(a = U*S*V')
	 *
	 * The matrix V (NOT the transpose V') replaces "a" on output.
	 * The diagonal matrix of singular values S is output as a vector w[1..n].
	 * The matrix U is output as V_[n*n].
	 */
	void GT_svdcmp(double *a, int m, int n, double *w, double *v, double *rv1);
	/**
	 * @internal for internal use only
	 *
	 * @brief Raised cosine function  (out = lambda*cos[(in/th)*(PI/2)]).
	 */
	double GT_RaisedCos(double in, double th, double lambda);
	/**
	 * @internal for internal use only
	 *
	 * @brief Product of two matrices
	 */
	short GT_MultMatrix(const double* A, int m, int n, const double* B, char k, char p, double* OUT);
	/**
	 * @internal for internal use only
	 *
	 * @brief Transpose a matrix
	 *
	 * @note the input and the output matrices must be different!!!
	 */
	void GT_TransMatrix(const double* A, int m, int n, double* OUT);
	/**
	 * @internal for internal use only
	 *
	 * @brief Computes the SVD-based regularized matrix pseudoinversion (a = U*S*V')
	 */
	void GT_RegPinv(const double *J, int m, int n, double *JPInv, double treshold, double lambda, double* prod, int* flag);
	/**
	 * @internal for internal use only
	 */
	void RegPinvEnrico(const double *J, int m, int n, double *JPInv, double M, double xidotmax, double* prod, int* flag);
	/**
	 * @internal for internal use only
	 */
	void RegPinvAlpha(const double *J, int m, int n, double *JPInv, double M, double xidotmax, const double* alpha, double* prod, int* flag);
	/**
	 * @internal for internal use only
	 */
	void RegPinvSmooth(const double *J, int m, int n, double *JPInv, double threshold, double lambda, double* prod, int* flag);
	/**
	 * @internal for internal use only
	 */
	void RegPinvNullSpaceOriented(const double *J, int m, int n, double *JPInv, double M, double xidotmax, const double* alpha, const double* G, const double* Q, double* prod, int* flag);
	/**
	 * @internal for internal use only
	 */
	double SmoothTransition(double x, double beta, double xmin, double lambda);
	/**
	 * @internal for internal use only
	 */
	double SmoothFunction(double x, double beta, double lambda);
} //namespace CMAT

#endif /* __CMAT_SVD_H__ */
