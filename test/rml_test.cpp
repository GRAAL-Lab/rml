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

	S = rml::RightJuxtapose(A,U);
	V = rml::UnderJuxtapose(A,U);
	rml::PrintMatrix(A, "A");
	rml::PrintMatrix(U, "U");
	rml::PrintMatrix(S, "rml::RightJuxtapose(A,U)");
	rml::PrintMatrix(V, "rml::UnderJuxtapose(A,U)");


	///////////////////////////////
	//////     PINV TEST     //////
	///////////////////////////////

	int iterations = 1000;
	int cols = 4, rows = 4;

	vector<double> input(cols*rows), output(rows*cols);

	// Fill with random values
	for (int c = 0; c < cols; c++) {
		for (int r = 0; r < rows; r++) {
			input.at(r + c*rows) = (rand() / (1.0 + RAND_MAX));
		}
	}

	TimeResults r1, r2;
	PinvSpecs pinvSpecs;

	pinvSpecs.nRows = rows;
	pinvSpecs.nCols = cols;
	pinvSpecs.thresh = 0.01;
	pinvSpecs.lambda = 0.0001;

	PseudoInverseTest(iterations, input, pinvSpecs, output, r1);

	//	///////////////////////////////
	//	//////     SVD TEST      //////
	//	///////////////////////////////
	//
	//	const int m = 4, n = 4;
	//
	//	Eigen::MatrixXd A(m, n), U(m, m), S(m, n), V(n, n);
	//		timeval t1, t2;
	//		double d;
	//
	//		for (int col = 0; col < n; col++) {
	//			for (int row = 0; row < m; row++) {
	//				A(row, col) = rand() / (1.0 + RAND_MAX);
	//			}
	//		}
	//
	//	rml::PrintMatrix(A, "A:");
	//
	//	rml::SVD(A, U, S, V);

	///////////////////////////////
	//////     DJDQ TEST     //////
	///////////////////////////////



	return 0;
}



