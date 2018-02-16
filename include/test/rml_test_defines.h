/*
 * rml_test_defines.h
 *
 *  Created on: Feb 16, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_TEST_RML_TEST_DEFINES_H_
#define INCLUDE_TEST_RML_TEST_DEFINES_H_

#include <iostream>
#include <sys/time.h>
#include <vector>
#include "rml/RML.h"

using std::vector;

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

double TimeDiff(timeval t1, timeval t2) {
	double t;
	t = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	t += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

	return t;
}

void PrintResult(const std::string type, const int iter, const PinvSpecs specs, const double time){
	std::cout << type << "\t" << specs.nRows << "x" << specs.nCols << " \t " << tc::white << time << " ms"
			<< tc::none << "\t(" << iter << " iterations)" << std::endl;
}

void PseudoInverseTest(const int iterations, const std::vector<double> &input, const PinvSpecs specs, std::vector<double> &output, TimeResults &results) {

	Eigen::MatrixXd A(specs.nRows, specs.nCols), B(specs.nCols, specs.nRows);
	timeval t1, t2;
	double d;

	for (int col = 0; col < specs.nCols; col++) {
		for (int row = 0; row < specs.nRows; row++) {
			A(row, col) = input[row + col * specs.nRows];
		}
	}

	gettimeofday(&t1, NULL);
	for (int i = 0; i < iterations; i++) {
		B = rml::RegularizedPseudoInverse(A, specs.thresh, specs.lambda);
	}
	gettimeofday(&t2, NULL);
	d = TimeDiff(t1, t2);
	results.inv = d;
	PrintResult("PseudoInverseTest", iterations, specs, d);
	//cout << B << endl;
}


#endif /* INCLUDE_TEST_RML_TEST_DEFINES_H_ */
