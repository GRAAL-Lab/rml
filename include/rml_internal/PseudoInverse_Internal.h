/*
 * PseudoInverse_Internal.h
 *
 *  Created on: Feb 19, 2018
 *      Author: fraw
 */

#ifndef SRC_PSEUDOINVERSE_INTERNAL_H_
#define SRC_PSEUDOINVERSE_INTERNAL_H_

namespace rml {

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}


/**
 * @internal for internal use only
 */
void RegPinvEnrico(const double *J, int m, int n, double *JPInv, double M, double xidotmax, double* prod, int* flag);
/**
 * @internal for internal use only
 */
void RegPinvAlpha(const double *J, int m, int n, double *JPInv, double M, double xidotmax, const double* alpha, double* prod, int* flag);
/**
 * @internal for internal use only
 */
void RegPinvSmooth(const double *J, int m, int n, double *JPInv, double threshold, double lambda, double* prod, int* flag);
/**
 * @internal for internal use only
 */
void RegPinvNullSpaceOriented(const double *J, int m, int n, double *JPInv, double M, double xidotmax, const double* alpha, const double* G, const double* Q, double* prod, int* flag);

}


#endif /* SRC_PSEUDOINVERSE_INTERNAL_H_ */
