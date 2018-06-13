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

#include "VehicleModel.h"
#include "Functions.h"

using std::cout;
using std::endl;

namespace rml {

VehicleModel::VehicleModel(const std::string id): id_(id) {
	modelInitialized_ = false;
	fbkPosition_.setZero();
	velocityOnVehicle_.setZero();
	accelerationOnVehicle_.setZero();
    controlRef_.setZero();
    transformation_.insert(std::make_pair(id_,fbkPosition_.ToTransfMatrix()));
}

VehicleModel::~VehicleModel() {
}

void VehicleModel::SetPositionOnInertial(const Eigen::Vector6d& fbkPos) {
	fbkPosition_ = fbkPos; 
    transformation_.erase(transformation_.begin(), transformation_.end());
    jacobians_.erase(jacobians_.begin(), jacobians_.end());
    transformation_.insert(std::make_pair(id_,fbkPosition_.ToTransfMatrix()));
    jacobians_.insert(std::make_pair(id_,vJv_));
    //updating rigid frame transformation matrix
    for (std::unordered_map<std::string, Eigen::TransfMatrix>::iterator iter = attachedBodyFrames_.begin();
         iter != attachedBodyFrames_.end();
         ++iter)
    {
        std::string id=iter->first;
        transformation_.insert(std::make_pair(id,GetAttachedBodyTransf(id)));
        jacobians_.insert(std::make_pair(id,GetAttachedBodyJacobian(id)));
    }
}

void VehicleModel::SetVelocityOnVehicle(const Eigen::Vector6d& velocityOnVehicle) {
	velocityOnVehicle_ = velocityOnVehicle;
}

void VehicleModel::SetAccelerationOnVehicle(const Eigen::Vector6d& accOnVehicle) {
	accelerationOnVehicle_ = accOnVehicle;
}

/*const Eigen::Vector6d& VehicleModel::GetCartesianVelocity() {
		Eigen::TransfMatrix wTv;
		wTv = fbkPosition_.ToTransfMatrix();
	    cartVelocity_ = wTv.GetRotMatrix().Transpose().GetCartesianRotationMatrix() * fbkVelocity_;
	    return cartVelocity_;
}

const Eigen::Vector6d& VehicleModel::GetCartesianAcceleration() {
		Eigen::TransfMatrix wTv;
		wTv = fbkPosition_.ToTransfMatrix();
	    cartAcceleration_ = wTv.GetRotMatrix().Transpose().GetCartesianRotationMatrix() * fbkAcceleration_;
	    return cartVelocity_;
}*/

void VehicleModel::SetJacobian(Eigen::Matrix6d vehicleJacobian) {

	vJv_ = vehicleJacobian;
    jacobians_.insert(std::make_pair(id_,vJv_));
	modelInitialized_ = true;
}

void VehicleModel::AddRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat)
{
    attachedBodyFrames_.insert(std::make_pair(id_+ID, TMat));
    transformation_.insert(std::make_pair(id_+ID,GetCurrentAttachedBodyTransf(id_+ID)));
    jacobians_.insert(std::make_pair(id_+ID,GetAttachedBodyJacobian(id_+ID)));
}

Eigen::TransfMatrix VehicleModel::GetAttachedBodyTransf(std::string& ID)
{
	return attachedBodyFrames_.at(ID);
}

Eigen::TransfMatrix VehicleModel::GetCurrentAttachedBodyTransf(const std::string ID)
{
	Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID);
    return transformation_.at(id_) * TMat;
    //TODO carlotta
    //return GetwTv() * TMat;
}

Eigen::MatrixXd VehicleModel::GetAttachedBodyJacobian(const std::string ID)
{
	Eigen::TransfMatrix RBMat = attachedBodyFrames_.at(ID);
    return GetRigidBodyMatrix(RBMat.GetTransl())*jacobians_.at(id_);
    //TODO carlotta
    //return GetRigidBodyMatrix(RBMat.GetTransl()) * GetvJv();
}

Eigen::TransfMatrix VehicleModel::GetTransfMatrix(const std::string ID)
{
   return transformation_.at(ID);
}

Eigen::MatrixXd VehicleModel::GetJacobian(const std::string ID)
{
    return jacobians_.at(ID);
}
}


