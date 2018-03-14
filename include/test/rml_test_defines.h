/*
 * rml_test_defines.h
 *
 *  Created on: Feb 16, 2018
 *      Author: Francesco Wanderlingh
 */

#ifndef INCLUDE_TEST_RML_TEST_DEFINES_H_
#define INCLUDE_TEST_RML_TEST_DEFINES_H_

#include <iostream>
#include <sys/time.h>
#include <vector>
#include <rml/RML.h>

#include "rml_internal/Futils.h"

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
	rml::SVDParameters SVDparams;

	PinvSpecs() :
		nRows(0), nCols(0) {
		SVDparams.threshold = 0.01;
		SVDparams.lambda = 0.0001;
	}
};

double TimeDiff(timeval t1, timeval t2) {
	double t;
	t = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	t += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

	return t;
}

void PrintResult(const std::string type, const int iter, const PinvSpecs specs, const double time){
	std::cout << type << "\t" << specs.nRows << "x" << specs.nCols << " \t " << tc::white << time/1000 << " us"
			<< tc::none << "\t(avg. on " << iter << " iterations)" << std::endl;
}

void PseudoInverseTest(const int iterations, Eigen::MatrixXd& A, PinvSpecs &specs, Eigen::MatrixXd& Apinv, TimeResults &results) {

	timeval t1, t2;
	double d;

	A.resize(specs.nRows, specs.nCols);
	A.setRandom();

	gettimeofday(&t1, NULL);
	for (int i = 0; i < iterations; i++) {
		Apinv = rml::RegularizedPseudoInverse(A, specs.SVDparams);
	}
	gettimeofday(&t2, NULL);
	d = TimeDiff(t1, t2);
	results.inv = d;
	PrintResult("PseudoInverseTest", iterations, specs, d);
}




#endif /* INCLUDE_TEST_RML_TEST_DEFINES_H_ */
