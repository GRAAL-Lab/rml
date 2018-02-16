/*
 *  rml_test.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 *
 *  This executable tests all the functionalities of the Robotic Mathematical Library (RML)
 */

#include "test/rml_test_defines.h"

int main(int argc, char* argv[]){

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


	///////////////////////////////
	//////     DJDQ TEST     //////
	///////////////////////////////



	return 0;
}



