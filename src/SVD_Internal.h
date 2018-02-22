/*
 * SVD_Internal.h
 *
 *  Created on: Feb 19, 2018
 *      Author: fraw
 */



#ifndef SRC_SVD_INTERNAL_H_
#define SRC_SVD_INTERNAL_H_

namespace rml {

const double RPY2VectEpsilon = 0.000000001;
const unsigned long int MaxMatrixDim = 1300;           ///< Max matrices dimension
const double SVDEpsilon = 0.00000001;

/**
 * @internal for internal use only
 *
 * @return  sqrt(a*a+b*b) (@see CMAT::GT_svdcmp)
 */
double GT_pythag(double a, double b);

/**
 * @internal for internal use only
 *
 * @brief Raised cosine function  (out = lambda*cos[(in/th)*(PI/2)]).
 */
double GT_RaisedCos(double in, double th, double lambda);
/**
 * @internal for internal use only
 *
 * @brief Product of two matrices
 */
short GT_MultMatrix(const double* A, int m, int n, const double* B, char k, char p, double* OUT);
/**
 * @internal for internal use only
 *
 * @brief Transpose a matrix
 *
 * @note the input and the output matrices must be different!!!
 */
void GT_TransMatrix(const double* A, int m, int n, double* OUT);

/**
 * @internal for internal use only
 */
double SmoothTransition(double x, double beta, double xmin, double lambda);
/**
 * @internal for internal use only
 */
double SmoothFunction(double x, double beta, double lambda);

}

#endif /* SRC_SVD_INTERNAL_H_ */
