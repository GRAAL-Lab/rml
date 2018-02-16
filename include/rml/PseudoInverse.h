/*
 * PseudoInverse.h
 *
 *  Created on: Feb 15, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_PSEUDOINVERSE_H_
#define INCLUDE_RML_PSEUDOINVERSE_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <rml/MatrixOperations.h>

namespace rml
{
const double RPY2VectEpsilon = 0.000000001;
const double VersorLemmaThreshold = 0.000000001;
const int MaxMatrixDim = 1300;           ///< Max matrices dimension
const double SVDEpsilon = 0.00000001;
//#define PI					3.14159265358979323846

//#define MATRIXLIB_SIGN(a,b)	((b) >= 0.0 ? fabs(a) : -fabs(a))
//#define MATRIXLIB_MAX(a,b)	((a) > (b)  ? (a) : (b))
//#define MATRIXLIB_MIN(a,b)	((a) < (b)  ? (a) : (b))

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


	/*
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
	//void SVD(const CMAT::Matrix& A, CMAT::Matrix& U, CMAT::Matrix& S, CMAT::Matrix& V);

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
	void SVD_NumericalRecipes(double *a, int m, int n, double *w, double *v, double *rv1);
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

	/*template<class MatT>
	void SVD(const MatT& A, MatT& U, MatT& S, MatT& V) {

		int m = A.rows(), n = A.cols();
		std::cout << "nxm = " << n <<"x" << m << std::endl;

		double SVD[3 * MaxMatrixDim];
		double w[MaxMatrixDim];

		int index[MaxMatrixDim];

		Eigen::MatrixXd Utmp;
		Eigen::MatrixXd Vtmp;

		// Matrix to double
		// Eigen::Map<MatT>(theDoubleWhereMatrixIsCopied, rows, columns) = MatrixToBeCopied;

		// double to Matrix
		// MatT eigenPInv = Eigen::Map<MatT>(JPInv, n, m);

		double U_temp[MaxMatrixDim], V_temp[MaxMatrixDim];
		//A.CopyTo(U_temp);
		Eigen::Map<MatT>(U_temp, m, n) = A;

		SVD_NumericalRecipes(U_temp, m, n, w, V_temp, SVD);

//		Utmp.CopyFrom(U_temp);
//		Vtmp.CopyFrom(V_temp);
		Utmp = Eigen::Map<MatT>(U_temp, m, m);
		Vtmp = Eigen::Map<MatT>(V_temp, n, n);



		for (int i = 0; i < MaxMatrixDim; i++)
			index[i] = i;// + 1;

		double max = 0;
		int index_max;
		for (int i = 0; i < n; i++) {
			max = w[i];
			index_max = i;
			for (int j = i + 1; j < n; j++) {
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

		Vtmp.resize(n, n);
		Utmp.resize(m, std::max(m, n));

		//Vtmp.PrintToDebugConsole("Vtmp");
		//U = Utmp.GetSubMatrix(1, index[0], m, index[0]);
		//V = Vtmp.GetSubMatrix(1, index[0], n, index[0]);

		PrintMatrix(Utmp.block(0, index[0], m, 1), "Utmp.block");
		PrintMatrix(Vtmp.block(0, index[0], n, 1), "Vtmp.block");

		U = Utmp.block(0, index[0], m, 1);
		V = Vtmp.block(0, index[0], n, 1);
		//Utmp.PrintToDebugConsole("Utmp");

		for (int i = 0; i < n; i++) {
			std::cout << "i=" << i <<  ", index[i]=" << index[i] <<std::endl;
			//KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "A1", "%d", i);
			//V = V.RightJuxtapose(Vtmp.GetSubMatrix(1, index[i], n, index[i]));

			//V = V.RightJuxtapose(Vtmp.block(0, index[i], n, 1));
			V = RightJuxtapose(V, Vtmp.block(0, index[i], n, 1));
		}

		/*
		for (int i = 1; i < m; i++) {
			//KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "A2", "%d", i);
			//U = U.RightJuxtapose(Utmp.GetSubMatrix(1, index[i], m, index[i]));
			U = U.RightJuxtapose(Utmp.block(0, index[i], m,1));
		}

		//U = U.GetSubMatrix(1, 1, m, m);
		U = U.block<m, m>(0, 0);

		S.resize(m, n);
		S.setZero();
		S.diagonal() = Eigen::Map<Eigen::MatrixXd>( w, 1, S.cols() ); //Create a Set Diagonal?

		//S = S.GetSubMatrix(1, 1, m, n);
		S = S.block(0, 0, m, n);
	}*/

	/**
	 *
	 * @param mat
	 * @param threshold
	 * @param lambda
	 * @return
	 */
	template<class MatT>
	Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> RegularizedPseudoInverse(const MatT &mat,
			double threshold, double lambda) // choose appropriately
			{

		int m = mat.rows(), n = mat.cols();
		double J[m * n];                // NULL pointer
		double JPInv[n * m];

		/**
		 * Here we convert the input type to a double array which is the type used by the GT_RegPinv
		 */
		Eigen::Map<MatT>(J, m, n) = mat;

		double mu;
		int flag;

		GT_RegPinv(J, m, n, JPInv, threshold, lambda, &mu, &flag);

		/**
		 * Here the results of the GT_RegPinv algorithm are mapped back to the input type
		 */
		MatT eigenPinv = Eigen::Map<MatT>(JPInv, n, m);

		return eigenPinv;
	}

} //namespace rml



#endif /* INCLUDE_RML_PSEUDOINVERSE_H_ */
