/*
 * PseudoInverse.cc
 *
 *  Created on: Feb 15, 2018
 *      Author: fraw
 */

#include <iostream>
#include "rml/PseudoInverse.h"

#include "../include/rml/Types.h"
#include "rml/MatrixOperations.h"
#include "rml/SVD.h"
#include "rml_internal/SVD_Internal.h"
#include "rml_internal/PseudoInverse_Internal.h"

namespace rml {

//  Computes the SVD-based regularized matrix pseudoinversion (a = U*S*V')
void GT_RegPinv(const double *J, int m, int n, double *JPInv, double treshold, double lambda, double* prod, int* flag) {
	int tempswap;
	static double SVD[3 * MaxMatrixDim];
	static double V[MaxMatrixDim];
	static double S[MaxMatrixDim];
	static double U[MaxMatrixDim];
	static double temp[MaxMatrixDim];

	int i, j;
	double Reg;
	//short flag = 0;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, MaxMatrixDim);

	SVD_NumericalRecipes(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		double singularValue = fabs(S[i]);
		if (singularValue <= SVDEpsilon) {
			if (singularValue < 0) {
				std::cout << "WARNING: in GT_svdcmp - DAFUQ S[" << i << "] < 0 -> S =" << S[i] << std::endl;
			}
			*prod = (*prod) * singularValue;
			S[i] = 0;
			//flag += 1;
			*flag += 1;
		} else {
			Reg = GT_RaisedCos(S[i], treshold, lambda);

			*prod = (*prod) * S[i];

			if (Reg != 0) {
				//flag += 1;
				*flag += 1;
			}

			S[i] = S[i] / (S[i] * S[i] + Reg);
		}

	}

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++)
			temp[n * i + j] = V[n * j + i] * S[j];
	}

	// Jpinv = V*S*U'
	GT_MultMatrix(U, m, n, temp, n, n, JPInv);

}

}

