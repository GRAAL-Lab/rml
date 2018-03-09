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
#include <vector>
#include <algorithm>

#include "rml/VehicleModel.h"
#include "rml/Functions.h"

using std::cout;
using std::endl;

namespace rml {

VehicleModel::VehicleModel() {
	modelInitialized_ = false;
}

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

void VehicleModel::AddRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat)
{
	attachedBodyFrames_.insert(std::make_pair(ID, TMat));
}

Eigen::TransfMatrix VehicleModel::GetAttachedBodyTransf(std::string& ID)
{
	return attachedBodyFrames_.at(ID);
}

Eigen::TransfMatrix VehicleModel::GetCurrentAttachedBodyTransf(std::string& ID)
{
	Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID);
	return GetwTv() * TMat;
}

Eigen::MatrixXd VehicleModel::GetAttachedBodyJacobian(std::string& ID)
{
	Eigen::TransfMatrix RBMat = attachedBodyFrames_.at(ID);
	return GetRigidBodyMatrix(RBMat.GetTransl()) * GetvJv();
}

}


