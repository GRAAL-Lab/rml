/*
 * PseudoInverse_Internal.cc
 *
 *  Created on: Feb 19, 2018
 *      Author: fraw
 */

#include "SVD.h"
#include "rml_internal/SVD_Internal.h"

namespace rml {
void RegPinvEnrico(const double *J, int m, int n, double *JPInv, double M, double xidotmax, double* prod, int* flag) {
	int tempswap;
	static double SVD[3 * MaxMatrixDim];
	static double V[MaxMatrixDim];
	static double S[MaxMatrixDim];
	static double U[MaxMatrixDim];
	static double temp[MaxMatrixDim];

	int i, j;
	double Reg;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, MaxMatrixDim);

	// computing J = U*S*V'
	// however this function returns
	// V in the parameter U
	// S in the parameter S
	// U in the parameter V
	SVD_NumericalRecipes(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVDEpsilon) {
			*prod = (*prod) * S[i];
			S[i] = 0;
			//flag += 1;
			*flag += 1;
		} else {
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Threshold[%d] = %lf", i, threshold);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Lambda[%d] = %lf", i, lambda);

			double threshold = fabs(xidotmax) / M;

			if (S[i] > threshold) {
				Reg = 0;
			} else {
				if (S[i] > (threshold / 2)) {
					Reg = (threshold - S[i]) * (S[i]);
				} else {
					Reg = threshold * threshold / 4.0;
				}
				//Reg = GT_RaisedCos(S[i],threshold,lambda); //(threshold - S[i])*(S[i]);
			}

			*prod = (*prod) * S[i];

			if (Reg != 0) {
				*flag += 1;
			}

			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "S[%d] = %lf", i, S[i]);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Reg[%d] = %lf", i, Reg);
			S[i] = S[i] / (S[i] * S[i] + Reg);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Sinv[%d] = %lf", i, S[i]);

		}

	}

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++)
			temp[n * i + j] = V[n * j + i] * S[j];
	}

	// Jpinv = V*S*U'
	GT_MultMatrix(U, m, n, temp, n, n, JPInv);

}

void RegPinvAlpha(const double *J, int m, int n, double *JPInv, double M, double xidotmax, const double* alpha,
		double* prod, int* flag) {
	int tempswap;
	static double SVD[3 * MaxMatrixDim];
	static double V[MaxMatrixDim];
	static double S[MaxMatrixDim];
	static double U[MaxMatrixDim];
	static double temp[MaxMatrixDim];

	int i, j;
	double Reg;
	double RegAlpha;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, MaxMatrixDim);

	// computing J = U*S*V'
	// however this function returns
	// V in the parameter U
	// S in the parameter S
	// U in the parameter V
	SVD_NumericalRecipes(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVDEpsilon) {
			*prod = (*prod) * S[i];
			S[i] = 0;

			*flag += 1;
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "S[%d] = %lf - alpha[%d] = %lf -  Salpha[%d] = %lf", i, S[i], i, alpha[i], i, Salpha[i]);
		} else {
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Threshold[%d] = %lf", i, threshold);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Lambda[%d] = %lf", i, lambda);

			double threshold = fabs(xidotmax) / M;

			if (S[i] > 2 * threshold) {
				Reg = 0;
			} else {
				Reg = SmoothTransition(S[i], 1.6 * xidotmax / M, 0, 1.5 * xidotmax * xidotmax / (4 * M * M));
			}

			RegAlpha = /*1 - alpha[i];*/SmoothTransition(alpha[i], 1, 0, 1.5 * xidotmax * xidotmax / (4 * M * M));

			*prod = (*prod) * S[i];

			if (Reg != 0) {
				*flag += 1;
			}

			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "S[%d] = %lf - alpha[%d] = %lf -  Salpha[%d] = %lf", i, S[i], i, alpha[i], i, Salpha[i]);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Reg[%d] = %lf", i, Reg);
			S[i] = S[i] / (S[i] * S[i] + Reg + RegAlpha);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Sinv[%d] = %lf", i, S[i]);

		}

	}

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++)
			temp[n * i + j] = V[n * j + i] * S[j];
	}

	// Jpinv = V*S*U'
	GT_MultMatrix(U, m, n, temp, n, n, JPInv);

}

void RegPinvNullSpaceOriented(const double *J, int m, int n, double *JPInv, double M, double xidotmax,
		const double* alpha, const double* G, const double* Q, double* prod, int* flag) {
	//int tempswap;
	static double SVD[3 * MaxMatrixDim];
	static double V[MaxMatrixDim];
	static double S[MaxMatrixDim];
	static double U[MaxMatrixDim];
	//static double temp[CMAT_MAX_MATRIX_DIM];

	static double Vg[MaxMatrixDim];
	static double Sg[MaxMatrixDim];
	static double Ug[MaxMatrixDim];
	static double SVDg[3 * MaxMatrixDim];

	//int	i, j;
	int i;
	double Reg;
	double RegAlpha;
	*flag = 0;

	memset(S, 0, MaxMatrixDim);

	SVD_NumericalRecipes(U, m, n, S, V, SVD);

	SVD_NumericalRecipes(Ug, m, n, Sg, Vg, SVDg);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVDEpsilon) {
			*prod = (*prod) * S[i];
			S[i] = 0;
			*flag += 1;
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "S[%d] = %lf - alpha[%d] = %lf -  Salpha[%d] = %lf", i, S[i], i, alpha[i], i, Salpha[i]);
		} else {
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Threshold[%d] = %lf", i, threshold);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Lambda[%d] = %lf", i, lambda);

			double threshold = fabs(xidotmax) / M;

			if (S[i] > 2 * threshold) {
				Reg = 0;
			} else {
				Reg = SmoothTransition(S[i], 1.6 * xidotmax / M, 0, 1.5 * xidotmax * xidotmax / (4 * M * M));
			}

			RegAlpha = /*1 - alpha[i];*/SmoothTransition(alpha[i], 1, 0, 1.5 * xidotmax * xidotmax / (4 * M * M));

			*prod = (*prod) * S[i];

			if (Reg != 0) {
				*flag += 1;
			}

			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "S[%d] = %lf - alpha[%d] = %lf -  Salpha[%d] = %lf", i, S[i], i, alpha[i], i, Salpha[i]);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Reg[%d] = %lf", i, Reg);
			S[i] = S[i] / (S[i] * S[i] + Reg + RegAlpha);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Sinv[%d] = %lf", i, S[i]);

		}

	}

	// Jpinv = V*S*U'

}

void RegPinvSmooth(const double *J, int m, int n, double *JPInv, double threshold, double lambda, double* prod,
		int* flag) {
	int tempswap;
	static double SVD[3 * MaxMatrixDim];
	static double V[MaxMatrixDim];
	static double S[MaxMatrixDim];
	static double U[MaxMatrixDim];
	static double temp[MaxMatrixDim];

	int i, j;
	double Reg;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, MaxMatrixDim);

	// computing J = U*S*V'
	// however this function returns
	// V in the parameter U
	// S in the parameter S
	// U in the parameter V
	SVD_NumericalRecipes(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVDEpsilon) {
			*prod = (*prod) * S[i];
			S[i] = 0;
			//flag += 1;
			*flag += 1;
		} else {
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Threshold[%d] = %lf", i, threshold);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Lambda[%d] = %lf", i, lambda);

			if (S[i] > threshold) {
				Reg = 0;
			} else {
				Reg = SmoothTransition(S[i], threshold, 0, lambda);
			}

			*prod = (*prod) * S[i];

			if (Reg != 0) {
				*flag += 1;
			}

			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "S[%d] = %lf", i, S[i]);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Reg[%d] = %lf", i, Reg);
			S[i] = S[i] / (S[i] * S[i] + Reg);
			//KAL::DebugConsole::Write(LOG_LEVEL_INFO, "SVD",  "Sinv[%d] = %lf", i, S[i]);

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
