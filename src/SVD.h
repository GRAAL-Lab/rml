/*
 * SVD.h
 *
 *  Created on: Feb 17, 2018
 *      Author: Francesco Wanderlingh
 */

#ifndef SRC_SVD_H_
#define SRC_SVD_H_

namespace rml{

const double RPY2VectEpsilon = 0.000000001;
const double VersorLemmaThreshold = 0.000000001;
const unsigned long int MaxMatrixDim = 1300;           ///< Max matrices dimension
const double SVDEpsilon = 0.00000001;

//#define PI					3.14159265358979323846

//#define MATRIXLIB_SIGN(a,b)	((b) >= 0.0 ? fabs(a) : -fabs(a))
//#define MATRIXLIB_MAX(a,b)	((a) > (b)  ? (a) : (b))
//#define MATRIXLIB_MIN(a,b)	((a) < (b)  ? (a) : (b))


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

}



#endif /* SRC_SVD_H_ */
