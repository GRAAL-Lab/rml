/*
 * MatrixBaseAddons.h
 *
 *  Created on: Feb 16, 2018
 *      Author: fw
 */

//#ifndef INCLUDE_RML_MATRIXBASEADDONS_H_
//#define INCLUDE_RML_MATRIXBASEADDONS_H_
//
//
//
//
//
//#endif /* INCLUDE_RML_MATRIXBASEADDONS_H_ */

inline Scalar at(uint i, uint j) const { return this->operator()(i,j); }
inline Scalar& at(uint i, uint j) { return this->operator()(i,j); }
inline Scalar at(uint i) const { return this->operator[](i); }
inline Scalar& at(uint i) { return this->operator[](i); }

//template<typename MatT>
//inline void RightJuxtapose(const MatT inmat){
//	PlainObject res = derived();
//	res.resize(res.rows() + inmat.rows(), res.cols() + inmat.cols());
//	res.block(0, this->cols(), inmat.rows(), inmat.cols()) = inmat;
//
//	//return res;
//}
