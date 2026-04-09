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
    inertialF_T_vehicleF_.setIdentity();
    velocity_.setZero();
    acceleration_.setZero();
    controlRef_.setZero();

    std::size_t underscorepos = id.find_first_of("_");
    if (underscorepos == std::string::npos) {
        id_ = std::move(id);
        transformation_.insert(std::make_pair(id_, inertialF_T_vehicleF_));

    } else {
        LabelSyntaxException labelException;
        labelException.SetHow("VehicleModel() constructor: Underscores '_' not allowed in ID");
        throw(labelException);
    }
}

VehicleModel::~VehicleModel()
{
}

void VehicleModel::PositionOnInertialFrame(const Eigen::TransformationMatrix& inertialF_T_vehicleF)
{
    inertialF_T_vehicleF_ = inertialF_T_vehicleF;

    if (!isMapInitialized_) {
        // If the Jacobian has been changed we need to rebuild the transformation
        // map --> Question is, will it ever change? Does it make sense to erase
        // everytime?
        transformation_.erase(transformation_.begin(), transformation_.end());
        jacobians_.erase(jacobians_.begin(), jacobians_.end());

        transformation_.insert(std::make_pair(id_, inertialF_T_vehicleF_));
        jacobians_.insert(std::make_pair(id_, vJv_));

        // Updating rigid frame transformation matrix
        for (std::unordered_map<std::string, Eigen::TransformationMatrix>::iterator iter = rigidBodyFrames_.begin(); iter != rigidBodyFrames_.end(); ++iter) {
            std::string id = iter->first;
            transformation_.insert(std::make_pair(id, inertialF_T_vehicleF_ * iter->second));
            jacobians_.insert(std::make_pair(id, RigidBodyMatrix(iter->second.TranslationVector()) * jacobians_.at(id_)));
        }
        isMapInitialized_ = true;
    } else {
        transformation_.find(id_)->second = inertialF_T_vehicleF_;
        jacobians_.find(id_)->second = vJv_;

        // Updating rigid frame transformation matrix
        for (std::unordered_map<std::string, Eigen::TransformationMatrix>::iterator iter = rigidBodyFrames_.begin(); iter != rigidBodyFrames_.end(); ++iter) {
            std::string id = iter->first;
            transformation_.find(id)->second = inertialF_T_vehicleF_ * iter->second;
            jacobians_.find(id)->second = RigidBodyMatrix(iter->second.TranslationVector()) * jacobians_.at(id_);
        }
    }
}

void VehicleModel::Jacobian(Eigen::Matrix6d J)
{
    vJv_ = std::move(J);
    jacobians_.insert(std::make_pair(id_, vJv_));
    modelInitialized_ = true;
    isMapInitialized_ = false;
}

void VehicleModel::AttachRigidBodyFrame(const std::string& frameID, const Eigen::TransformationMatrix& vehicleF_T_frameID)
{
    std::string rigidBodyFrameID = id_ + "_" + frameID;

    // Check if rigid body is already present
    if (rigidBodyFrames_.find(rigidBodyFrameID) != rigidBodyFrames_.end()) {
        rigidBodyFrames_.at(rigidBodyFrameID) = vehicleF_T_frameID;
        transformation_.at(rigidBodyFrameID) = transformation_.at(id_) * vehicleF_T_frameID; //EvaluateRigidBodyTransf(idRigidFrame);
        jacobians_.at(rigidBodyFrameID) = RigidBodyMatrix(vehicleF_T_frameID.TranslationVector()) * jacobians_.at(id_);
    } else {
        rigidBodyFrames_.insert(std::make_pair(rigidBodyFrameID, vehicleF_T_frameID));
        transformation_.insert(std::make_pair(rigidBodyFrameID, transformation_.at(id_) * vehicleF_T_frameID));
        jacobians_.insert(std::make_pair(rigidBodyFrameID, RigidBodyMatrix(vehicleF_T_frameID.TranslationVector()) * jacobians_.at(id_)));
    }
}

Eigen::TransformationMatrix VehicleModel::TransformationMatrix(const std::string& frameID) noexcept(false)
{
    if (transformation_.find(frameID) == transformation_.end()) {
        WrongFrameException vehicleModelWrongLabel;
        std::string how = "[VEHICLE MODEL] The frame does not exist: " + frameID;
        vehicleModelWrongLabel.SetHow(how);
        throw(vehicleModelWrongLabel);
    }
    return transformation_.at(frameID);
}

Eigen::TransformationMatrix VehicleModel::TransformationMatrix(const std::string& frameID_j, const std::string& frameID_k)
{
    return TransformationMatrix(frameID_j).transpose() * TransformationMatrix(frameID_k);
}

Eigen::MatrixXd VehicleModel::Jacobian(const std::string& ID) noexcept(false)
{
    if (jacobians_.find(ID) == jacobians_.end()) {
        WrongFrameException vehicleModelWrongLabel;
        std::string how = "[VEHICLE MODEL] The frame does not exist: " + ID;
        vehicleModelWrongLabel.SetHow(how);
        throw(vehicleModelWrongLabel);
    }
    return jacobians_.at(ID);
}

}
