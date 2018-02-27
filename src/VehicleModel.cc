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
	vehicleDOFs_ = 0;
}

VehicleModel::VehicleModel(const VehicleModel& other) :
				vehicleDOFs_ (0), q_(other.q_), wTv_(other.wTv_), vTb_(other.vTb_), I3_(other.I3_), Zeros_(other.Zeros_) {

	if(vehicleDOFs_ != 0){
		h_ = other.h_;
	}
}

VehicleModel& VehicleModel::operator=(VehicleModel other) {
	swap(*this, other);
	return *this;
}

VehicleModel::~VehicleModel() {
}

void VehicleModel::SetPosition(const Eigen::Vector6d& q) {
	q_ = q;

	EvaluatewTv();
}

void VehicleModel::SetVelocity(const Eigen::Vector6d& qdot) {
	qdot_ = qdot;
}

void VehicleModel::SetBaseDOFs(int armJoints) {
	vehicleDOFs_ = armJoints;

	h_.resize(vehicleDOFs_);

	q_ = Eigen::VectorXd::Zero(vehicleDOFs_);
	qdot_ = Eigen::VectorXd::Zero(vehicleDOFs_);
}

void VehicleModel::InitMatrix() {

	Zeros_ = Eigen::MatrixXd::Zero(3,3);
}

void VehicleModel::EvaluatewTv() {

	std::cout << "VehicleModel::EvaluatewTv - " <<
			"Using empty call EvaluatewTv(), missing specialization" << std::endl;
}

/*void VehicleModel::Get6DPosition(CMAT::Vect6& vehiclePos) {
    ortos::DebugConsole::Write(ortos::LogLevel::error, "VehicleModel::Get6DPositionVector",
            "Using empty call Get6DVelocityVector(), missing specialization");
}

void VehicleModel::Get6DVelocity(CMAT::Vect6& vehicleVel) {
	ortos::DebugConsole::Write(ortos::LogLevel::error, "VehicleModel::Get6DVelocityVector",
			"Using empty call Get6DVelocityVector(), missing specialization");
}*/

void swap(rml::VehicleModel& first, rml::VehicleModel& second) {

	using std::swap;
	swap(first.q_, second.q_);
	swap(first.qdot_, second.qdot_);
	swap(first.wTv_, second.wTv_);
	swap(first.vTb_, second.vTb_);
	swap(first.h_, second.h_);
	swap(first.I3_, second.I3_);
	swap(first.Zeros_, second.Zeros_);
}

}


