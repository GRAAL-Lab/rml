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

}


#endif /* SRC_PSEUDOINVERSE_INTERNAL_H_ */
