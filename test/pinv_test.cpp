/*
 * pinv_test.cpp
 *
 *  Created on: Feb 2, 2017
 *      Author: francesco
 */

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

#include "test_defines.h"

using std::vector;
using std::cout;
using std::endl;

int iterations = 1000;

void RunCMATCMAT(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results);
void RunEigenEigen(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results);
void RunEigenCMATWrap(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results);
void RunEigenCMAT(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results);
int main(int argc, char **argv) {

	int rowMinSize = 2, colMinSize = 2;
	int rowMaxSize = 32, colMaxSize = 32;
	int step = 2;

	if (argc < 5) {
		printf("%s** ERROR **\n"
				"argv[1]: Rows min size\n"
				"argv[2]: Columns min size\n"
				"argv[3]: Rows MAX size\n"
				"argv[4]: Columns MAX size\n"
				"argv[5]: Step size (optional) \n%s", TC_RED, TC_NONE);
		exit(EXIT_FAILURE);
	} else {
		rowMinSize = strtol(argv[1], NULL, 0);
		colMinSize = strtol(argv[2], NULL, 0);
		rowMaxSize = strtol(argv[3], NULL, 0);
		colMaxSize = strtol(argv[4], NULL, 0);
		if(argc == 6) step = strtol(argv[5], NULL, 0);
		if ((rowMaxSize * colMaxSize) > CMAT_MAX_MATRIX_DIM) {
			rowMaxSize = std::min(rowMaxSize, static_cast<int>(floor(sqrt(CMAT_MAX_MATRIX_DIM))));
			colMaxSize = std::min(colMaxSize, static_cast<int>(floor(sqrt(CMAT_MAX_MATRIX_DIM))));
			cout << tc::magL << "Max size was too big, clipped to: " << rowMaxSize << "x" << colMaxSize << tc::none
					<< endl;
		}
	}

	// Check if output directory exist and create it if needed
	struct stat st = { 0 };

	if (stat("./out_pinv", &st) == -1) {
		mkdir("./out_pinv", 0700);
	}

	timeval t1, t2;

	vector<vector<double> > inv_results;

	vector<std::string> labels;

	cout << tc::yel << "Pseudo-inversion execution times:" << tc::none << endl;

	for (int rows = rowMinSize; rows <= rowMaxSize; rows += step) {
		for (int cols = colMinSize; cols <= colMaxSize; cols += step) {

		labels.push_back(std::to_string(rows) + "x" + std::to_string(cols));

		vector<double> input(cols*rows), output(rows*cols);

		// Fill with random values
		for (int c = 0; c < cols; c++) {
			for (int r = 0; r < rows; r++) {
				input.at(r + c*rows) = (rand() / (1.0 + RAND_MAX));
			}
		}

		TimeResults r1, r2, r3, r4;
		PinvSpecs pinvSpecs;

		pinvSpecs.nRows = rows;
		pinvSpecs.nCols = cols;
		pinvSpecs.thresh = 0.01;
		pinvSpecs.lambda = 0.0001;

		RunCMATCMAT(input, pinvSpecs, output, r1);
		RunEigenEigen(input, pinvSpecs, output, r2);
		RunEigenCMATWrap(input, pinvSpecs, output, r3);
		//RunEigenCMAT(n, values, r4);

		vector<double> inv;

		inv.push_back(r1.inv / iterations);
		inv.push_back(r2.inv / iterations);
		inv.push_back(r3.inv / iterations);
		//inv.push_back(r4.inv / ITERATIONS);
		inv_results.push_back(inv);

		cout << endl;
		}
	}

	// Output HTML table
	std::ofstream output_inv(inv_file.c_str());
	HTMLOutput(output_inv, inv_results, labels);
}

void RunCMATCMAT(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results) {

	CMAT::Matrix A(specs.nRows, specs.nCols), B(specs.nCols, specs.nRows);
	timeval t1, t2;
	double d;

	for (int col = 1; col <= specs.nCols; col++) {
		for (int row = 1; row <= specs.nRows; row++) {
			A(row, col) = input[(row-1) + (col-1) * specs.nRows];
		}
	}

	gettimeofday(&t1, NULL);
	for (int i = 0; i < iterations; i++) {
		B = A.RegPseudoInverse(specs.thresh, specs.lambda);
	}
	gettimeofday(&t2, NULL);
	d = TimeDiff(t1, t2);
	results.inv = d;
	PrintResult("CMAT only  ", specs, d);
	//A.PrintMtx("A");
	//B.PrintMtx("B");
}

void RunEigenEigen(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results) {

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
		B = EigenEigenPinv(A, specs.thresh);
	}
	gettimeofday(&t2, NULL);
	d = TimeDiff(t1, t2);
	results.inv = d;
	PrintResult("Eigen only  ", specs, d);
	//cout << B << endl;
}

void RunEigenCMATWrap(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results) {

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
		B = EigenRegPinvWrap(A, specs.thresh, specs.lambda);
	}
	gettimeofday(&t2, NULL);
	d = TimeDiff(t1, t2);
	results.inv = d;
	PrintResult("Eigen-CMAT wrap", specs, d);
	//cout << B << endl;
}

void RunEigenCMAT(const vector<double> &input, const PinvSpecs specs, vector<double> &output, TimeResults &results) {

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
		B = EigenCMATPinv(A, specs.thresh, specs.lambda);
	}
	gettimeofday(&t2, NULL);
	d = TimeDiff(t1, t2);
	results.inv = d;
	PrintResult("Eigen-CMAT mixed", specs, d);
}

/*void RunPseudoInverseTest(PinvFunc pinvFunc, InvSpecs specs, const double input[], double output[], TimeResults &timeRes){

 }
 */
