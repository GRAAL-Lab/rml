/*
 * MatrixBaseAddons.h
 *
 *  Created on: Feb 16, 2018
 *      Author: fw
 */

#ifndef INCLUDE_RML_MATRIXBASEADDONS_H_
#define INCLUDE_RML_MATRIXBASEADDONS_H_

inline Scalar at(uint i, uint j) const { return this->operator()(i,j); }
inline Scalar& at(uint i, uint j) { return this->operator()(i,j); }
inline Scalar at(uint i) const { return this->operator[](i); }
inline Scalar& at(uint i) { return this->operator[](i); }


inline Matrix<Scalar, 6, 6> GetRigidBodyMatrix() const
{
	eigen_assert(derived().rows() == 3 && derived().cols() == 1);
	Matrix<Scalar, 3, 3> t_hat;
	t_hat << 0,    -derived()(2),  derived()(1),
			derived()(2),    0,  -derived()(0),
			-derived()(1),  derived()(0),    0;
	Matrix<Scalar, 6, 6> S;
	S.block(0,0,3,3) = S.block(3,3,3,3) = Eigen::Matrix<Scalar, 3, 3>::Identity();
	S.block(0,3,3,3) = Eigen::Matrix<Scalar, 3, 3>::Zero();
	S.block(3,0,3,3) = -1.0 * t_hat;
	return S;
}

#endif /* INCLUDE_RML_MATRIXBASEADDONS_H_ */



//template<typename MatT>
//inline void RightJuxtapose(const MatT inmat){
//	PlainObject res = derived();
//	res.resize(res.rows() + inmat.rows(), res.cols() + inmat.cols());
//	res.block(0, this->cols(), inmat.rows(), inmat.cols()) = inmat;
//
//	//return res;
//}
