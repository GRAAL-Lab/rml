/*
 * ctrl_vehiclemodel.cpp
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#include <algorithm>
#include <iostream>
#include <vector>

#include "RMLDefines.h"
#include "RMLExceptions.h"
#include "Functions.h"
#include "VehicleModel.h"

using std::cout;
using std::endl;

namespace rml {

VehicleModel::VehicleModel(const std::string id)
    : modelInitialized_(false)
    , isMapInitialized_(false)
    , id_(id)
{
    fbkPosition_.setZero();
    velocityOnVehicle_.setZero();
    accelerationOnVehicle_.setZero();
    controlRef_.setZero();
    transformation_.insert(std::make_pair(id_, fbkPosition_.ToTransfMatrix()));
}

VehicleModel::~VehicleModel()
{
}

void VehicleModel::SetPositionOnInertial(const Eigen::Vector6d& fbkPos)
{
    fbkPosition_ = fbkPos;
    if (!isMapInitialized_) {
        // If the Jacobian has been changed we need to rebuild the transformation
        // map --> Question is, will it ever change? Does it make sense to erase
        // everytime?

        transformation_.erase(transformation_.begin(), transformation_.end());
        jacobians_.erase(jacobians_.begin(), jacobians_.end());
        transformation_.insert(std::make_pair(id_, fbkPosition_.ToTransfMatrix()));
        jacobians_.insert(std::make_pair(id_, vJv_));
        // Updating rigid frame transformation matrix
        for (std::unordered_map<std::string, Eigen::TransfMatrix>::iterator iter = attachedBodyFrames_.begin();
             iter != attachedBodyFrames_.end();
             ++iter) {
            std::string id = iter->first;
            transformation_.insert(std::make_pair(id, GetAttachedBodyTransf(id)));
            jacobians_.insert(std::make_pair(id, GetAttachedBodyJacobian(id)));
        }
        isMapInitialized_ = true;
    } else {
        transformation_.find(id_)->second = fbkPosition_.ToTransfMatrix();
        jacobians_.find(id_)->second = vJv_;
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, Eigen::TransfMatrix>::iterator iter = attachedBodyFrames_.begin();
             iter != attachedBodyFrames_.end();
             ++iter) {
            std::string id = iter->first;
            transformation_.find(id)->second = GetAttachedBodyTransf(id);
            jacobians_.find(id)->second = GetAttachedBodyJacobian(id);
        }
    }
}

void VehicleModel::SetVelocityOnVehicle(const Eigen::Vector6d& velocityOnVehicle)
{
    velocityOnVehicle_ = velocityOnVehicle;
}

void VehicleModel::SetAccelerationOnVehicle(const Eigen::Vector6d& accOnVehicle)
{
    accelerationOnVehicle_ = accOnVehicle;
}

void VehicleModel::SetJacobian(Eigen::Matrix6d vehicleJacobian)
{
    vJv_ = vehicleJacobian;
    jacobians_.insert(std::make_pair(id_, vJv_));
    modelInitialized_ = true;
    isMapInitialized_ = false;
}

void VehicleModel::AddRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat)
{
    attachedBodyFrames_.insert(std::make_pair(id_ + ID, TMat));
    transformation_.insert(std::make_pair(id_ + FrameID::Body + ID, GetCurrentAttachedBodyTransf(id_ + ID)));
    jacobians_.insert(std::make_pair(id_ + FrameID::Body + ID, GetAttachedBodyJacobian(id_ + ID)));
}

Eigen::TransfMatrix VehicleModel::GetAttachedBodyTransf(const std::string& ID) throw(std::exception)
{
    if (attachedBodyFrames_.find(ID) == attachedBodyFrames_.end()) {
        VehicleModelWrongLabelException vehicleModelWrongLabel;
        vehicleModelWrongLabel.SetID("GetAttachedBodyTransf");
        throw(vehicleModelWrongLabel);
    }
    return attachedBodyFrames_.at(ID);
}

Eigen::TransfMatrix VehicleModel::GetCurrentAttachedBodyTransf(const std::string ID)
{
    Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID);
    return transformation_.at(id_) * TMat;
}

Eigen::MatrixXd VehicleModel::GetAttachedBodyJacobian(const std::string ID)
{
    Eigen::TransfMatrix RBMat = attachedBodyFrames_.at(ID);
    return GetRigidBodyMatrix(RBMat.GetTransl()) * jacobians_.at(id_);
}

Eigen::TransfMatrix VehicleModel::GetTransfMatrix(const std::string ID) throw(std::exception)
{
    if (transformation_.find(ID) == transformation_.end()) {
        VehicleModelWrongLabelException vehicleModelWrongLabel;
        vehicleModelWrongLabel.SetID("GetTransfMatrix");
        throw(vehicleModelWrongLabel);
    }
    return transformation_.at(ID);
}

Eigen::MatrixXd VehicleModel::GetJacobian(const std::string ID) throw(std::exception)
{
    if (jacobians_.find(ID) == jacobians_.end()) {
        VehicleModelWrongLabelException vehicleModelWrongLabel;
        vehicleModelWrongLabel.SetID("GetJacobian");
        throw(vehicleModelWrongLabel);
    }
    return jacobians_.at(ID);
}

const Eigen::TransfMatrix VehicleModel::GetwTv()
{
    return fbkPosition_.ToTransfMatrix();
}

const Eigen::Matrix6d& VehicleModel::GetvJv() const throw(std::exception)
{
    if (!modelInitialized_) {
        throw(VehicleModelNotInitializedException());
    }
    return vJv_;
}

bool VehicleModel::IsModelInitialized() const
{
    return modelInitialized_;
}

const Eigen::Vector6d& VehicleModel::GetControlVector() const
{
    return controlRef_;
}

void VehicleModel::SetControlVector(const Eigen::Vector6d& controlRef)
{
    controlRef_ = controlRef;
}

void VehicleModel::SetID(std::string id)
{
    id_ = id;
}

std::string VehicleModel::GetID()
{
    return id_;
}

const Eigen::Vector6d& VehicleModel::GetPositionOnInertial()
{
    return fbkPosition_;
}

const Eigen::Vector6d& VehicleModel::GetVelocityOnVehicle()
{
    return velocityOnVehicle_;
}

const Eigen::Vector6d& VehicleModel::GetAccelerationOnVehicle()
{
    return accelerationOnVehicle_;
}
}
