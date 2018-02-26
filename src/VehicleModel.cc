/*
 * ctrl_vehiclemodel.cpp
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

//#include "ctrl.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <utility>
#include <vector>
#include <iterator>
#include <climits>
#include <stdlib.h>
#include "rml/VehicleModel.h"


using std::cout;
using std::endl;

namespace rml {

VehicleModel::VehicleModel() {
	vehicleDOFs_ = 0;
	h_ = NULL;
	arrayQ_ = NULL;
}

VehicleModel::VehicleModel(const VehicleModel& other) :
				vehicleDOFs_ (0), q_(other.q_), wTv_(other.wTv_), vTb_(other.vTb_), I3_(other.I3_), Zeros_(other.Zeros_) {

	/*
	 * If the arm model we are copying is not initialised we have to initialise all the pointers to NULL since
	 * we don't know yet the size of the containers.
	 */
	if (vehicleDOFs_ == 0) {
		h_ = NULL;
		arrayQ_ = NULL;
	}

	/*
	 * While if the arm has been already initialised we can copy all the necessary containers.
	 */
	else {
		h_ = new CMAT::Vect6[vehicleDOFs_];
		arrayQ_ = new double[vehicleDOFs_];

		for (int i = 0; i < vehicleDOFs_; i++) {
			h_[i] = other.h_[i];
		}

	}
}

VehicleModel& VehicleModel::operator=(VehicleModel other) {
	swap(*this, other);
	return *this;
}

VehicleModel* VehicleModel::clone() const {
	return new VehicleModel(*this);
}

VehicleModel::~VehicleModel() {
	if (vehicleDOFs_ != 0) {
		delete[] h_;
		delete[] arrayQ_;
	}
}

/*int BaseModel::SelectModelRepresentation(int representation) {

}*/

void VehicleModel::SetPosition(const CMAT::Matrix& q) {
	q_ = q;
}

void VehicleModel::SetVelocity(const CMAT::Matrix& qdot) {
	qdot_ = qdot;
}

void VehicleModel::SetBaseDOFs(int armJoints) {
	vehicleDOFs_ = armJoints;

	h_ = new CMAT::Vect6[vehicleDOFs_];
	arrayQ_ = new double[vehicleDOFs_];

	q_ = CMAT::Matrix::Zeros(vehicleDOFs_, 1);
	qdot_ = CMAT::Matrix::Zeros(vehicleDOFs_, 1);
}

void VehicleModel::InitMatrix() {
	I3_ = CMAT::Matrix::Eye(3);
	Zeros_ = CMAT::Matrix::Zeros(3);

    vTb_ = CMAT::Matrix::Eye(4);
    wTv_ = CMAT::Matrix::Eye(4);
}

void VehicleModel::EvaluatewTv(CMAT::TransfMatrix& wTv) {

	ortos::DebugConsole::Write(ortos::LogLevel::error, "VehicleModel::EvaluatewTv",
			"Using empty call EvaluatewTv(), missing specialization");
}

void VehicleModel::Get6DPosition(CMAT::Vect6& vehiclePos) {
    ortos::DebugConsole::Write(ortos::LogLevel::error, "VehicleModel::Get6DPositionVector",
            "Using empty call Get6DVelocityVector(), missing specialization");
}

void VehicleModel::Get6DVelocity(CMAT::Vect6& vehicleVel) {
	ortos::DebugConsole::Write(ortos::LogLevel::error, "VehicleModel::Get6DVelocityVector",
			"Using empty call Get6DVelocityVector(), missing specialization");
}

void swap(rml::VehicleModel& first, rml::VehicleModel& second) {

	using std::swap;
	swap(first.q_, second.q_);
	swap(first.qdot_, second.qdot_);
	swap(first.wTv_, second.wTv_);
	swap(first.vTb_, second.vTb_);
	swap(first.h_, second.h_);
	swap(first.arrayQ_, second.arrayQ_);
	swap(first.I3_, second.I3_);
	swap(first.Zeros_, second.Zeros_);
}

}


