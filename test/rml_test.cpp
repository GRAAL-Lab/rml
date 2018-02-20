/*
 *  rml_test.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 *
 *  This executable tests all the functionalities of the Robotic Mathematical Library (RML)
 */

#include "test/rml_test_defines.h"

using std::cout;
using std::endl;

int main(int argc, char* argv[]){

	const int m = 4, n = 4;

	Eigen::MatrixXd A(m, n), U(m, m), S(m, n), V(n, n);
	timeval t1, t2;
	double d;

	A.setRandom();
	U.setRandom();
	S.setRandom();
	V.setRandom();

	///////////////////////////////
	///    MATRIX OPERATIONS    ///
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### MATRIX OPERATIONS Test ###" << tc::none << std::endl;

	S = rml::RightJuxtapose(A,U);
	V = rml::UnderJuxtapose(A,U);
	rml::PrintMatrix(A, "A");
	rml::PrintMatrix(U, "U");
	rml::PrintMatrix(S, "rml::RightJuxtapose(A,U)");
	rml::PrintMatrix(V, "rml::UnderJuxtapose(A,U)");

	Eigen::Vector6d vect6;
	Eigen::Vector3d vect3_1, vect3_2;
	vect3_1.setConstant(1);
	vect3_2.setConstant(2);
	vect6.setZero();
	rml::PrintMatrix(vect6.transpose(), "vect6'");
	rml::PrintMatrix(vect3_1.transpose(), "vect3_1'");
	rml::PrintMatrix(vect3_2.transpose(), "vect3_2'");

	rml::SetFirstVect3(vect6, vect3_1);
	rml::PrintMatrix(vect6.transpose(), "vect6' after SetFirstVect3()");
	rml::SetSecondVect3(vect6, vect3_2);
	rml::PrintMatrix(vect6.transpose(), "vect6' after SetSecondVect3()");
	///////////////////////////////
	//////     PINV TEST     //////
	///////////////////////////////

	Eigen::MatrixXd Apinv;

	int iterations = 1000;
	int rowMinSize = 4, colMinSize = 4;
	int rowMaxSize = 16, colMaxSize = 16;
	int step = 4;

	cout << tc::yel << "Pseudo-inversion execution times:" << tc::none << endl;

	for (int rows = rowMinSize; rows <= rowMaxSize; rows += step) {
		for (int cols = colMinSize; cols <= colMaxSize; cols += step) {

			TimeResults timings;
			PinvSpecs pinvSpecs;

			pinvSpecs.nRows = rows;
			pinvSpecs.nCols = cols;
			pinvSpecs.thresh = 0.01;
			pinvSpecs.lambda = 0.0001;

			PseudoInverseTest(iterations, A, pinvSpecs, Apinv, timings);
		}
	}

	///////////////////////////////
	//////     SVD TEST      //////
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### SVD Test ###" << tc::none << std::endl;

    for (int i = 0; i < A.rows(); i++){
        for (int j = 0; j < A.cols(); j++){
            A(i, j) = i * A.cols() + j;
        }
    }

	rml::SVD(A, U, S, V);



	rml::PrintMatrix(A, "A");
	rml::PrintMatrix(U, "U");
	rml::PrintMatrix(S, "S");
	rml::PrintMatrix(V, "V");

	Eigen::MatrixXd A_usv = U * S * V.transpose();
	rml::PrintMatrix(A_usv, "A as the result of: A = U*S*V'");

	///////////////////////////////
	//////     DJDQ TEST     //////
	///////////////////////////////




	return 0;
}



