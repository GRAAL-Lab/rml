/*
 * ctrl_vehiclemodel.cpp
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#include <algorithm>
#include <iostream>
#include <vector>

#include "Functions.h"
#include "RMLDefines.h"
#include "RMLExceptions.h"
#include "VehicleModel.h"

using std::cout;
using std::endl;

namespace rml {

VehicleModel::VehicleModel(const std::string id)
    : modelInitialized_(false)
    , isMapInitialized_(false)
{
    fbkPosition_.setZero();
    velocityOnVehicle_.setZero();
    accelerationOnVehicle_.setZero();
    controlRef_.setZero();
    transformation_.insert(std::make_pair(id_, fbkPosition_.ToTransfMatrix()));
    std::size_t underscorepos = id.find_first_of("_");
    if (underscorepos == std::string::npos) {
        id_ = std::move(id);
    } else {
        LabelSyntaxException labelException;
        labelException.SetHow("VehicleModel() constructor: Underscores '_' not allowed in ID");
        throw(labelException);
    }
}

VehicleModel::~VehicleModel()
{
}

void VehicleModel::PositionOnInertial(Eigen::Vector6d fbkPos)
{
    fbkPosition_ = std::move(fbkPos);
    if (!isMapInitialized_) {
        // If the Jacobian has been changed we need to rebuild the transformation
        // map --> Question is, will it ever change? Does it make sense to erase
        // everytime?

        transformation_.erase(transformation_.begin(), transformation_.end());
        jacobians_.erase(jacobians_.begin(), jacobians_.end());
        transformation_.insert(std::make_pair(id_, fbkPosition_.ToTransfMatrix()));
        jacobians_.insert(std::make_pair(id_, vJv_));
        // Updating rigid frame transformation matrix
        for (std::unordered_map<std::string, Eigen::TransfMatrix>::iterator iter = rigidBodyFrames_.begin(); iter != rigidBodyFrames_.end(); ++iter) {
            std::string id = iter->first;
            transformation_.insert(std::make_pair(id, fbkPosition_.ToTransfMatrix() * iter->second));
            jacobians_.insert(std::make_pair(id, GetRigidBodyMatrix(iter->second.Transl()) * jacobians_.at(id_)));
        }
        isMapInitialized_ = true;
    } else {
        transformation_.find(id_)->second = fbkPosition_.ToTransfMatrix();
        jacobians_.find(id_)->second = vJv_;
        //updating rigid frame transformation matrix
        for (std::unordered_map<std::string, Eigen::TransfMatrix>::iterator iter = rigidBodyFrames_.begin(); iter != rigidBodyFrames_.end(); ++iter) {
            std::string id = iter->first;
            transformation_.find(id)->second = fbkPosition_.ToTransfMatrix() * iter->second;
            jacobians_.find(id)->second = GetRigidBodyMatrix(iter->second.Transl()) * jacobians_.at(id_);
        }
    }
}

void VehicleModel::Jacobian(Eigen::Matrix6d vehicleJacobian)
{
    vJv_ = std::move(vehicleJacobian);
    jacobians_.insert(std::make_pair(id_, vJv_));
    modelInitialized_ = true;
    isMapInitialized_ = false;
}

void VehicleModel::SetRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat)
{
    std::string idRigidFrame = id_ + FrameID::Body + ID;
    // Check if rigid body is already present
    if (rigidBodyFrames_.find(idRigidFrame) != rigidBodyFrames_.end()) {
        // Check if the associated joint is the same otherwise throw exception

        rigidBodyFrames_.at(idRigidFrame) = TMat;
        transformation_.at(idRigidFrame) = transformation_.at(id_) * TMat; //EvaluateRigidBodyTransf(idRigidFrame);
        jacobians_.at(idRigidFrame) = GetRigidBodyMatrix(TMat.Transl()) * jacobians_.at(id_);
    } else {
        rigidBodyFrames_.insert(std::make_pair(idRigidFrame, TMat));
        transformation_.insert(std::make_pair(idRigidFrame, transformation_.at(id_) * TMat));
        jacobians_.insert(std::make_pair(idRigidFrame, GetRigidBodyMatrix(TMat.Transl()) * jacobians_.at(id_)));
    }
}

Eigen::TransfMatrix VehicleModel::GetTransformation(const std::string& frameID) noexcept(false)
{

    if (transformation_.find(frameID) == transformation_.end()) {
        WrongFrameException vehicleModelWrongLabel;
        std::string how = "[VEHICLE MODEL] The frame does not exist " + frameID;
        vehicleModelWrongLabel.SetHow(how);
        throw(vehicleModelWrongLabel);
    }
    return transformation_.at(frameID);
}

Eigen::TransfMatrix VehicleModel::GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k)
{
    return GetTransformation(frameID_j).transpose() * GetTransformation(frameID_k);
}

Eigen::MatrixXd VehicleModel::Jacobian(const std::string& ID) noexcept(false)
{
    if (jacobians_.find(ID) == jacobians_.end()) {
        WrongFrameException vehicleModelWrongLabel;
        std::string how = "[VEHICLE MODEL] The frame does not exist " + ID;
        vehicleModelWrongLabel.SetHow(how);
        throw(vehicleModelWrongLabel);
    }
    return jacobians_.at(ID);
}
}
