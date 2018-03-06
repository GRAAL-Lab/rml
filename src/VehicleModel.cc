/*
 * ctrl_vehiclemodel.cpp
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

#include <iostream>
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include "rml/VehicleModel.h"


using std::cout;
using std::endl;

namespace rml {

VehicleModel::VehicleModel() {
	modelInitialized_ = false;
}

//VehicleModel::VehicleModel(const VehicleModel& other) :
//				modelInitialized_(other.modelInitialized_),
//				fbkPosition_(other.fbkPosition_), wTv_(other.wTv_), vTb_(other.vTb_), I3_(other.I3_){
//}
//
//VehicleModel& VehicleModel::operator=(VehicleModel other) {
//	swap(*this, other);
//	return *this;
//}

VehicleModel::~VehicleModel() {
}

void VehicleModel::SetFeedbackPosition(const Eigen::Vector6d& fbkPos) {
	fbkPosition_ = fbkPos;

	//EvaluatewTv();
}

void VehicleModel::SetFeedbackVelocity(const Eigen::Vector6d& fbkVel) {
	fbkVelocity_ = fbkVel;
}

const Eigen::Vector6d& VehicleModel::GetCartesianVelocity() {
		Eigen::TransfMatrix wTv;
		wTv = fbkPosition_.ToTransfMatrix();
	    cartVelocity_ = wTv.GetRotMatrix().Transpose().GetCartesianRotationMatrix() * fbkVelocity_;
	    return cartVelocity_;
}

void VehicleModel::SetJacobian(Eigen::Matrix6d vehicleJacobian) {

	vJv_ = vehicleJacobian;
	modelInitialized_ = true;
}

void swap(rml::VehicleModel& first, rml::VehicleModel& second) {

	using std::swap;
	swap(first.fbkPosition_, second.fbkPosition_);
	swap(first.fbkVelocity_, second.fbkVelocity_);
	swap(first.cartVelocity_, second.cartVelocity_);
	swap(first.wTv_, second.wTv_);
	swap(first.I3_, second.I3_);
}

}


