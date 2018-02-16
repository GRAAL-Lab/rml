

#include "cmat_svd.h"

namespace CMATtest {

//  Return  sqrt(a*a+b*b) (used by GT_svdcmp)
double GT_pythag(double a, double b) {
	return sqrt(a * a + b * b);
}

double SmoothFunction(double x, double beta, double lambda) {
	return lambda / 2 * (1 + tanh(1 / (1 - x / beta) - beta / x));
}

double SmoothTransition(double x, double beta, double xmin, double lambda) {
	double h;
	if ((x <= xmin))
		h = lambda;
	else if ((x > xmin) && (x < (xmin + beta)))
		h = SmoothFunction(beta + xmin - x, beta, lambda);
	else
		h = 0;

	return h;
}

//  Computes the SVD decomposition of a matrix a[m*n]	(a = U*S*V')
//	The matrix V (NOT the transpose V') replaces "a" on output.
//  The diagonal matrix of singular values S is output as a vector w[1..n].
//	The matrix U is output as V_[n*n].

void GT_svdcmp(double *a, int m, int n, double *w, double *v, double *rv1) {
	int flag, i, its, j, jj, k, l, nm;
	double anorm, c, f, g, h, s, scale, x, y, z;

	g = 0.0;
	scale = 0.0;
	anorm = 0.0;

	for (i = 0; i < n; i++) {
		l = i + 1;
		rv1[i] = scale * g;
		g = 0.0;
		s = 0.0;
		scale = 0.0;

		if (i < m) {
			for (k = i; k < m; k++) {
				scale += fabs(a[k + m * i]);
			}
			if (scale) {
				for (k = i; k < m; k++) {
					a[k + m * i] /= scale;
					s += a[k + m * i] * a[k + m * i];
				}
				f = a[i + m * i];
				g = -MATRIXLIB_SIGN(sqrt(s), f);
				h = f * g - s;
				a[i + m * i] = f - g;

				for (j = l; j < n; j++) {
					for (s = 0.0, k = i; k < m; k++) {
						s += a[k + m * i] * a[k + m * j];
					}
					f = s / h;
					for (k = i; k < m; k++) {
						a[k + m * j] += f * a[k + m * i];
					}
				}
				for (k = i; k < m; k++) {
					a[k + m * i] *= scale;
				}
			}
		}

		w[i] = scale * g;
		g = 0.0;
		s = 0.0;
		scale = 0.0;

		if (i < m && i != (n - 1)) {
			for (k = l; k < n; k++) {
				scale += fabs(a[i + m * k]);
			}
			if (scale) {
				for (k = l; k < n; k++) {
					a[i + m * k] /= scale;
					s += a[i + m * k] * a[i + m * k];
				}
				f = a[i + m * l];
				g = -MATRIXLIB_SIGN(sqrt(s), f);
				h = f * g - s;
				a[i + m * l] = f - g;

				for (k = l; k < n; k++) {
					rv1[k] = a[i + m * k] / h;
				}
				for (j = l; j < m; j++) {
					for (s = 0.0, k = l; k < n; k++) {
						s += a[j + m * k] * a[i + m * k];
					}
					for (k = l; k < n; k++) {
						a[j + m * k] += s * rv1[k];
					}
				}
				for (k = l; k < n; k++) {
					a[i + m * k] *= scale;
				}
			}
		}
		anorm = MATRIXLIB_MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
	}

	for (i = n - 1; i >= 0; i--) {
		if (i < n - 1) {
			if (g) {
				for (j = l; j < n; j++) {
					v[j + n * i] = (a[i + m * j] / a[i + m * l]) / g;
				}
				for (j = l; j < n; j++) {
					for (s = 0.0, k = l; k < n; k++) {
						s += a[i + m * k] * v[k + n * j];
					}
					for (k = l; k < n; k++) {
						v[k + n * j] += s * v[k + n * i];
					}
				}
			}
			for (j = l; j < n; j++) {
				v[i + n * j] = 0.0;
				v[j + n * i] = 0.0;
			}
		}
		v[i + n * i] = 1.0;
		g = rv1[i];
		l = i;
	}

	for (i = MATRIXLIB_MIN(m, n) - 1; i >= 0; i--) {
		l = i + 1;
		g = w[i];

		for (j = l; j < n; j++) {
			a[i + m * j] = 0.0;
		}

		if (g) {
			g = 1.0 / g;

			for (j = l; j < n; j++) {
				for (s = 0.0, k = l; k < m; k++) {
					s += a[k + m * i] * a[k + m * j];
				}
				f = (s / a[i + m * i]) * g;
				for (k = i; k < m; k++) {
					a[k + m * j] += f * a[k + m * i];
				}
			}

			for (j = i; j < m; j++) {
				a[j + m * i] *= g;
			}
		} else {
			for (j = i; j < m; j++) {
				a[j + m * i] = 0.0;
			}
		}
		++a[i + m * i];
	}

	for (k = n - 1; k >= 0; k--) {
		for (its = 1; its <= 30; its++) {
			flag = 1;
			for (l = k; l >= 0; l--) {
				nm = l - 1;
				if ((double) (fabs(rv1[l]) + anorm) == anorm) {
					flag = 0;
					break;
				}
				if ((double) (fabs(w[nm]) + anorm) == anorm) {
					break;
				}
			}
			if (flag) {
				c = 0.0;
				s = 1.0;

				for (i = l; i <= k; i++) {
					f = s * rv1[i];
					rv1[i] = c * rv1[i];
					if ((double) (fabs(f) + anorm) == anorm) {
						break;
					}
					g = w[i];
					h = GT_pythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g * h;
					s = -f * h;
					for (j = 0; j < m; j++) {
						y = a[j + m * nm];
						z = a[j + m * i];
						a[j + m * nm] = y * c + z * s;
						a[j + m * i] = z * c - y * s;
					}
				}
			}

			z = w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j = 0; j < n; j++) {
						v[j + n * k] = -v[j + n * k];
					}
				}
				break;
			}

			if (its == 30) {
				//printf("No convergence in 30 GT_svdcmp iterations");
#if LOG_LEVEL >= LOG_LEVEL_WARNING
				//KAL::DebugConsole::Write(LOG_LEVEL_WARNING, "GT_svdcmp", "No convergence in 30 GT_svdcmp iterations");
#endif
			}
			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
			g = GT_pythag(f, 1.0);
			f = ((x - z) * (x + z) + h * ((y / (f + MATRIXLIB_SIGN(g, f))) - h)) / x;
			c = 1.0;
			s = 1.0;

			for (j = l; j <= nm; j++) {
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = GT_pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;

				for (jj = 0; jj < n; jj++) {
					x = v[jj + n * j];
					z = v[jj + n * i];
					v[jj + n * j] = x * c + z * s;
					v[jj + n * i] = z * c - x * s;
				}
				z = GT_pythag(f, h);
				w[j] = z;
				if (z) {
					z = 1.0 / z;
					c = f * z;
					s = h * z;
				}
				f = c * g + s * y;
				x = c * y - s * g;

				for (jj = 0; jj < m; jj++) {
					y = a[jj + m * j];
					z = a[jj + m * i];
					a[jj + m * j] = y * c + z * s;
					a[jj + m * i] = z * c - y * s;
				}
			}

			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
}

void SVD(const CMAT::Matrix& A, CMAT::Matrix& U, CMAT::Matrix& S, CMAT::Matrix& V) {
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double w[CMAT_MAX_MATRIX_DIM];

	//static double wordered[CMAT_MAX_MATRIX_DIM];
	static int index[CMAT_MAX_MATRIX_DIM];

	static CMAT::Matrix Utmp;
	static CMAT::Matrix Vtmp;

	//Utmp = A;
	static double U_temp[CMAT_MAX_MATRIX_DIM], V_temp[CMAT_MAX_MATRIX_DIM];
	A.CopyTo(U_temp);

	GT_svdcmp(U_temp, A.GetNumRows(), A.GetNumColumns(), w, V_temp, SVD);

	Utmp.CopyFrom(U_temp);
	Vtmp.CopyFrom(V_temp);

	for (int i = 0; i < CMAT_MAX_MATRIX_DIM; i++)
		index[i] = i + 1;

	double max = 0;
	int index_max;
	for (int i = 0; i < A.GetNumColumns(); i++) {
		max = w[i];
		index_max = i;
		for (int j = i + 1; j < A.GetNumColumns(); j++) {
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

	Vtmp.SetDimensions(A.GetNumColumns(), A.GetNumColumns());
	Utmp.SetDimensions(A.GetNumRows(), std::max(A.GetNumRows(), A.GetNumColumns()));

	//Vtmp.PrintToDebugConsole("Vtmp");
	U = Utmp.GetSubMatrix(1, index[0], A.GetNumRows(), index[0]);
	V = Vtmp.GetSubMatrix(1, index[0], A.GetNumColumns(), index[0]);
	//Utmp.PrintToDebugConsole("Utmp");

	for (int i = 1; i < A.GetNumColumns(); i++) {
		//KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "A1", "%d", i);
		//U = U.RightJuxtapose(Utmp.GetSubMatrix(1, index[i], A.GetNumRows(), index[i]));
		V = V.RightJuxtapose(Vtmp.GetSubMatrix(1, index[i], A.GetNumColumns(), index[i]));
	}

	for (int i = 1; i < A.GetNumRows(); i++) {
		//KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "A2", "%d", i);
		U = U.RightJuxtapose(Utmp.GetSubMatrix(1, index[i], A.GetNumRows(), index[i]));
		//V = V.RightJuxtapose(Vtmp.GetSubMatrix(1, index[i], A.GetNumColumns(), index[i]));
	}

	U = U.GetSubMatrix(1, 1, A.GetNumRows(), A.GetNumRows());

	S.SetDimensions(A.GetNumRows(), A.GetNumColumns());
	S.Reset();
	S.SetDiag(w);

	S = S.GetSubMatrix(1, 1, A.GetNumRows(), A.GetNumColumns());
}

//  Raised cosine function  (out = lambda*cos[(in/th)*(PI/2)])
double GT_RaisedCos(double in, double th, double lambda) {
	if (in < 0)
		in = -in;

	if (0 <= in && in <= th)
		return lambda * (0.5 * cos((in / th) * PI) + 0.5);
	else
		return 0;

}

//  Product of two matrices
short GT_MultMatrix(const double* A, int m, int n, const double* B, char k, char p, double* output) {

	int i;
	int j;
	int l;

	if (n != k) {
		return -1;
	}

	for (i = 0; i < m * p; i++) {
		output[i] = 0;
	}

	for (l = 0; l < p; l++) {
		for (i = 0; i < m; i++) {
			for (j = 0; j < n; j++) {
				output[i + l * m] += A[i + m * j] * B[j + l * n];
			}
		}
	}
	return 1;
}

//  Transpose a matrix (NOTE: the input and the output matrices must be different!!!)
void GT_TransMatrix(const double* A, int m, int n, double* output) {

	int i;
	int j;

	for (i = 0; i < m; i++) {
		for (j = 0; j < n; j++) {
			output[j + n * i] = A[i + m * j];
		}
	}
}

//  Computes the SVD-based regularized matrix pseudoinversion (a = U*S*V')
void GT_RegPinv(const double *J, int m, int n, double *JPInv, double treshold, double lambda, double* prod, int* flag) {
	int tempswap;
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double V[CMAT_MAX_MATRIX_DIM];
	static double S[CMAT_MAX_MATRIX_DIM];
	static double U[CMAT_MAX_MATRIX_DIM];
	static double temp[CMAT_MAX_MATRIX_DIM];

	int i, j;
	double Reg;
	//short flag = 0;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, CMAT_MAX_MATRIX_DIM);

	GT_svdcmp(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		double singularValue = fabs(S[i]);
		if (singularValue <= SVD_EPS) {
			if (singularValue < 0) {
				KAL::DebugConsole::Write(LOG_LEVEL_WARNING, "GT_svdcmp", "DAFUQ S[%d] < 0 -> S = %lf", i, S[i]);
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

void RegPinvEnrico(const double *J, int m, int n, double *JPInv, double M, double xidotmax, double* prod, int* flag) {
	int tempswap;
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double V[CMAT_MAX_MATRIX_DIM];
	static double S[CMAT_MAX_MATRIX_DIM];
	static double U[CMAT_MAX_MATRIX_DIM];
	static double temp[CMAT_MAX_MATRIX_DIM];

	int i, j;
	double Reg;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, CMAT_MAX_MATRIX_DIM);

	// computing J = U*S*V'
	// however this function returns
	// V in the parameter U
	// S in the parameter S
	// U in the parameter V
	GT_svdcmp(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVD_EPS) {
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
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double V[CMAT_MAX_MATRIX_DIM];
	static double S[CMAT_MAX_MATRIX_DIM];
	static double U[CMAT_MAX_MATRIX_DIM];
	static double temp[CMAT_MAX_MATRIX_DIM];

	int i, j;
	double Reg;
	double RegAlpha;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, CMAT_MAX_MATRIX_DIM);

	// computing J = U*S*V'
	// however this function returns
	// V in the parameter U
	// S in the parameter S
	// U in the parameter V
	GT_svdcmp(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVD_EPS) {
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
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double V[CMAT_MAX_MATRIX_DIM];
	static double S[CMAT_MAX_MATRIX_DIM];
	static double U[CMAT_MAX_MATRIX_DIM];
	//static double temp[CMAT_MAX_MATRIX_DIM];

	static double Vg[CMAT_MAX_MATRIX_DIM];
	static double Sg[CMAT_MAX_MATRIX_DIM];
	static double Ug[CMAT_MAX_MATRIX_DIM];
	static double SVDg[3 * CMAT_MAX_MATRIX_DIM];

	//int	i, j;
	int i;
	double Reg;
	double RegAlpha;
	*flag = 0;

	memset(S, 0, CMAT_MAX_MATRIX_DIM);

	GT_svdcmp(U, m, n, S, V, SVD);

	GT_svdcmp(Ug, m, n, Sg, Vg, SVDg);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVD_EPS) {
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
	static double SVD[3 * CMAT_MAX_MATRIX_DIM];
	static double V[CMAT_MAX_MATRIX_DIM];
	static double S[CMAT_MAX_MATRIX_DIM];
	static double U[CMAT_MAX_MATRIX_DIM];
	static double temp[CMAT_MAX_MATRIX_DIM];

	int i, j;
	double Reg;
	*flag = 0;

	GT_TransMatrix(J, m, n, U);

	tempswap = n;
	n = m;
	m = tempswap;

	memset(S, 0, CMAT_MAX_MATRIX_DIM);

	// computing J = U*S*V'
	// however this function returns
	// V in the parameter U
	// S in the parameter S
	// U in the parameter V
	GT_svdcmp(U, m, n, S, V, SVD);

	*prod = 1.0;

	for (i = 0; i < n; i++) {
		if (S[i] <= SVD_EPS) {
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

