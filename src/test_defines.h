/*
 * test_defines.h
 *
 *  Created on: Feb 2, 2017
 *      Author: fraw
 */

#ifndef TEST_DEFINES_H_
#define TEST_DEFINES_H_

#include <cstdio>
#include <iostream>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include "cmat_svd.h"
#include "futils.h"

const std::string out_folder = "./out/";
const std::string svd_file = out_folder + "svd.txt";
const std::string mul_file = out_folder + "multiply.txt";
const std::string add_file = out_folder + "addition.txt";
const std::string inv_file = out_folder + "invert.txt";
const std::string tran_file = out_folder + "transpose.txt";

struct TimeResults {
	double svd;
	double mul;
	double add;
	double inv;
	double tran;
};

struct PinvSpecs {
	int nRows;
	int nCols;
	double thresh;
	double lambda;

	PinvSpecs() :
			nRows(0), nCols(0), thresh(0.01), lambda(0.0001) {
	}
};

template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> EigenEigenPinv(const MatT &mat,
		typename MatT::Scalar tolerance = typename MatT::Scalar { 1e-4 }) // choose appropriately
		{
	typedef typename MatT::Scalar Scalar;
	Eigen::JacobiSVD<MatT> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
	//auto svd = mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	const auto &singularValues = svd.singularValues();
	Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
	singularValuesInv.setZero();
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > tolerance) {
			singularValuesInv(i, i) = Scalar { 1 } / singularValues(i);
		} else {
			singularValuesInv(i, i) = Scalar { 0 };
		}
	}
	return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> EigenRegPinvWrap(const MatT &mat,
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

	CMATtest::GT_RegPinv(J, m, n, JPInv, threshold, lambda, &mu, &flag);

	/**
	 * Here the results of the CMAT algorithm are mapped back to the input type
	 */
	MatT eigenPInv = Eigen::Map<MatT>(JPInv, n, m);

	return eigenPInv;
}

template<class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> EigenCMATPinv(const MatT &mat,
		double thresh, double lambda) // choose appropriately
		{

	int m = mat.rows(), n = mat.cols();
	double J[m * n];

	Eigen::Map<MatT>(J, m, n) = mat;

	typedef typename MatT::Scalar Scalar;
	//auto svd = mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	int tempswap;
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double V[CMAT_MAX_MATRIX_DIM];
	static double S[CMAT_MAX_MATRIX_DIM];
	static double U[CMAT_MAX_MATRIX_DIM];
	static double temp[CMAT_MAX_MATRIX_DIM];

	int i, j;
	double Reg;

	//short flag = 0;

	CMATtest::GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, CMAT_MAX_MATRIX_DIM);

	CMATtest::GT_svdcmp(U, m, n, S, V, SVD);

	MatT eigenU = Eigen::Map<MatT>(U, m, m);
	MatT eigenS = Eigen::Map<MatT>(S, m, n);
	MatT eigenV = Eigen::Map<MatT>(V, n, n);

	const auto &singularValues = eigenS.diagonal();

	Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
	singularValuesInv.setZero();
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > thresh) {
			singularValuesInv(i, i) = Scalar { 1 } / singularValues(i);
		} else {
			singularValuesInv(i, i) = Scalar { 0 };
		}
	}

	/**
	 * TODO Replace
	 */
	//return eigenV() * singularValuesInv * eigenU().adjoint();
	//return mat;

}

/**----------------------------------END OF PINV FUNCTIONS-------------------------------------**/

void HTMLOutput(std::ofstream &os, const std::vector<std::vector<double> > &results, const std::vector<std::string> &labels);
void PrintResult(const std::string type, const PinvSpecs specs, const double time);
double TimeDiff(timeval t1, timeval t2);

#endif /* TEST_DEFINES_H_ */
