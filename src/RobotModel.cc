/*
 * RobotModel.cc
 *
 *  Created on: Feb 27, 2018
 *      Author: fraw
 */

#include "rml_internal/Futils.h"

#include "Functions.h"
#include "MatrixOperations.h"
#include "RobotModel.h"

namespace rml {

RobotModel::RobotModel(Eigen::TransfMatrix robotFrame, std::string frameID)
    : robotFrameID_(frameID)
    , robotFrame_(robotFrame)
    , DoF_(0)
    , isVehicle_(false)
{
    vehicle_ = std::make_shared<rml::VehicleModel>(rml::VehicleModel(frameID));
    vehicle_->SetJacobian(Eigen::MatrixXd::Zero(6, 6));
    vehicle_->SetPositionOnInertial(robotFrame.GetRPYXYZ());
}

RobotModel::RobotModel(Eigen::TransfMatrix robotFrame, std::string frameID, Eigen::MatrixXd JRobotFrame)
    : robotFrameID_(frameID)
    , robotFrame_(robotFrame)
    , isVehicle_(true)
{
    vehicle_ = std::make_shared<rml::VehicleModel>(rml::VehicleModel(frameID));
    vehicle_->SetJacobian(JRobotFrame);
    robotFrameID_ = frameID;
    vehicle_->SetPositionOnInertial(robotFrame.GetRPYXYZ());
    DoF_ = 6;
}

RobotModel::~RobotModel()
{
    // TODO Auto-generated destructor stub
}

int RobotModel::GetTotalDOFs()
{
    return DoF_;
}

bool RobotModel::LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& robotframeToArm) throw(ExceptionWithHow)
{
    if (arm->IsModelInitialized()) {
        if (CheckArm(arm->GetID())) {
            std::string how;
            how = "Already existing arm: " + arm->GetID() + " in the RobotModel!";
            RobotModelArmException conflictingArmModelException;
            conflictingArmModelException.SetHow(how);
            throw(conflictingArmModelException);
        }
        armsModel_.insert(std::make_pair(arm->GetID(), arm));
        robotframeToArm_.insert(std::make_pair(arm->GetID(), robotframeToArm));
        DoF_ += arm->GetNumJoints();
        return true;

    } else {
        std::string how;
        how = "ArmModel: " + arm->GetID() + " is NOT initialized";
        RobotModelArmException notInitializedArmModelExceptions;
        notInitializedArmModelExceptions.SetHow(how);
        throw(notInitializedArmModelExceptions);
    }
}

std::string RobotModel::GetRobotFrameID()
{
    return robotFrameID_;
}

void RobotModel::SetRobotFramePosition(Eigen::TransfMatrix robotFrame)
{
    vehicle_->SetPositionOnInertial(robotFrame.GetRPYXYZ());
    robotFrame_ = robotFrame;
}

void RobotModel::SetRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat, const std::string frameToAttachID) throw(ExceptionWithHow)
{
    std::size_t partIDIndex = frameToAttachID.find_first_of("_");
    std::string partID = frameToAttachID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;

    if (partIDIndex == std::string::npos && frameToAttachID != robotFrameID_) {
        std::string how;
        how = "wrong string format: " + frameToAttachID;
        RobotModelWrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    } else if (frameToAttachID == robotFrameID_ || partID == robotFrameID_) {
        vehicle_->SetRigidBodyFrame(ID, TMat);
    } else if (CheckArm(partID)) {
        armsModel_.at(partID)->SetRigidBodyFrame(ID, frameToAttachID, TMat);
    } else {
        std::string how;
        how = "Asking a not existing part: " + partID;
        RobotModelWrongFrameException robotModelNotExistingPartExc;
        robotModelNotExistingPartExc.SetHow(how);
        throw(robotModelNotExistingPartExc);
    }
}

bool RobotModel::CheckArm(const std::string armID) const
{
    if (armsModel_.find(armID) != armsModel_.end()) {
        return true;
    } else {
        return false;
    }
}

bool RobotModel::CheckVehicle() const
{
    return isVehicle_;
}

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForFrame(const std::string& frameID) const throw(ExceptionWithHow)
{
    Eigen::MatrixXd bJt, rJt;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (CheckArm(partID)) {
        bJt = armsModel_.at(partID)->GetJacobian(frameID);
        Eigen::RotMatrix rRb = robotframeToArm_.at(partID).GetRotMatrix();
        rJt = rRb.GetCartesianRotationMatrix() * bJt;
        return rJt;
    } else {
        std::string how;
        how = "Not existing arm: " + partID;
        RobotModelArmException notExistingArmException;
        notExistingArmException.SetHow(how);
        throw(notExistingArmException);
    }
}

Eigen::Matrix6d RobotModel::GetIsolatedVehicleJacobianForFrame(const std::string& frameID) const
{
    Eigen::Matrix6d vJv;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (isVehicle_) {
        if (CheckArm(partID)) {
            vJv = vehicle_->GetJacobian(robotFrameID_);
            Eigen::TransfMatrix bTj = armsModel_.at(partID)->GetTransformation(frameID);
            Eigen::TransfMatrix vTj = robotframeToArm_.at(partID) * bTj;
            vJv = GetRigidBodyMatrix(vTj.GetTransl()) * vJv;
        }
    }
    return vJv;
}

void RobotModel::SetRobotControl(const Eigen::VectorXd& y) throw(std::exception)
{
    int startIndex = 0;
    if (y.size() == DoF_) {
        if (isVehicle_) {
            vehicle_->SetControlVector(y.block(0, 0, 6, 1));
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->SetControlVector(y.block(startIndex, 0, iter->second->GetNumJoints(), 1));
            startIndex = startIndex + iter->second->GetNumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

Eigen::VectorXd RobotModel::GetRobotControl(std::string partID) throw(ExceptionWithHow)
{
    Eigen::VectorXd y;

    if (partID == robotFrameID_ && isVehicle_) {
        return vehicle_->GetControlVector();
    } else if (CheckArm(partID)) {
        return (armsModel_.at(partID)->GetControlVector());
    } else {
        std::string how;
        how = "Asking a not existing part: " + partID;
        RobotModelWrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::TransfMatrix RobotModel::GetTransformation(const std::string& frameID) throw(ExceptionWithHow)
{

    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;
    if (partIDIndex == std::string::npos && frameID != robotFrameID_) {
        std::string how;
        how = "wrong string format: " + frameID;
        RobotModelWrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    if (frameID == robotFrameID_ || partID == robotFrameID_) {

        return vehicle_->GetTransformation(frameID);
    } else if (CheckArm(partID)) {
        T = armsModel_.at(partID)->GetTransformation(frameID);
        T = robotFrame_ * robotframeToArm_.at(partID) * T;
        return T;
    }

    std::string how;
    how = "Asking a not existing part: " + partID;
    RobotModelWrongFrameException robotModelNotExistingPartExc;
    robotModelNotExistingPartExc.SetHow(how);
    throw(robotModelNotExistingPartExc);
}

Eigen::TransfMatrix RobotModel::GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k)
{
    Eigen::TransfMatrix out;
    Eigen::TransfMatrix wTj, wTk;
    std::string partID_a = frameID_j.substr(0, frameID_j.find_first_of("_"));
    std::string partID_b = frameID_k.substr(0, frameID_k.find_first_of("_"));

    wTj = GetTransformation(frameID_j);
    wTk = GetTransformation(frameID_k);

    out = wTj.transpose() * wTk;
    return out;
}

Eigen::MatrixXd RobotModel::GetCartesianJacobian(const std::string& frameID) throw(ExceptionWithHow)
{

    std::string modelID = frameID.substr(0, frameID.find_first_of("_"));
    Eigen::MatrixXd totJac, tempJ;
    if (frameID == robotFrameID_ || modelID == robotFrameID_) {
        if (isVehicle_) {
            totJac = RightJuxtapose(totJac, vehicle_->GetJacobian(frameID));
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(6, DoF_ - 6));
        } else {
            totJac = Eigen::MatrixXd::Zero(6, DoF_);
        }
        return totJac;
    } else if (CheckArm(modelID)) {
        if (isVehicle_) {
            totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForFrame(frameID));
        }
        for (auto iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == modelID) {
                //in get isolated arm jacobian the vTb is taken into account
                tempJ = GetIsolatedArmJacobianForFrame(frameID);
            } else {
                tempJ = Eigen::MatrixXd::Zero(6, armsModel_.at(modelID)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
        return totJac;
    } else if (frameID.find_first_of("_") == std::string::npos && frameID != robotFrameID_) {
        std::string how;
        how = "Wrong format frame id: " + frameID;
        RobotModelWrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    std::string how;
    how = "Asking a not existing part: " + modelID;
    RobotModelWrongFrameException robotModelNotExistingPartException;
    robotModelNotExistingPartException.SetHow(how);
    throw(robotModelNotExistingPartException);
}

Eigen::MatrixXd RobotModel::GetJointSpaceJacobian(const std::string& armID) throw(ExceptionWithHow)

{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm(armID)) {
        int taskSize = armsModel_.at(armID)->GetNumJoints();
        if (vehicle_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(taskSize, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == armID) {
                tempJ = Eigen::MatrixXd::Identity(taskSize, taskSize);
            } else {
                tempJ = Eigen::MatrixXd::Zero(taskSize, armsModel_.at(armID)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
        return totJac;
    }
    std::string how;
    how = "Asking a not existing arm model : " + armID;
    RobotModelArmException notExistingArmException;
    notExistingArmException.SetHow(how);
    throw(notExistingArmException);
}
Eigen::MatrixXd RobotModel::GetManipulabilityJacobian(const std::string& frameID) throw(ExceptionWithHow)
{

    Eigen::MatrixXd totJac, tempJ;
    std::string armID = frameID.substr(0, frameID.find_first_of("_"));
    if (frameID.find_first_of("_") == std::string::npos) {
        std::string how;
        how = "Wrong format frame id: " + frameID;
        RobotModelWrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    if (CheckArm(armID)) {
        if (isVehicle_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(1, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {

            if (iter->first == armID) {
                tempJ = iter->second->GetManipulabilityJacobian(frameID);

            } else {
                tempJ = Eigen::MatrixXd::Zero(1, armsModel_.at(iter->first)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
        return totJac;
    }
    std::string how;
    how = "Asking frame for a no existing arm: " + armID;
    RobotModelWrongFrameException notExistingPartExc;
    notExistingPartExc.SetHow(how);
    throw(notExistingPartExc);
}

double RobotModel::GetManipulability(const std::string& frameID) throw(ExceptionWithHow)
{
    double out;
    std::string armID = frameID.substr(0, frameID.find_first_of("_"));
    if (frameID.find_first_of("_") == std::string::npos) {
        std::string how;
        how = "Wrong format frame id: " + frameID;
        RobotModelWrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    if (CheckArm(armID)) {
        out = armsModel_.at(armID)->GetManipulability(frameID);
        return out;
    }
    RobotModelArmException notExistingArm;
    std::string how;
    how = "Asking a not existing arm: " + armID;
    notExistingArm.SetHow(how);
    throw(notExistingArm);
}

const std::shared_ptr<ArmModel> RobotModel::GetArm(std::string ID) const throw(ExceptionWithHow)
{
    if (CheckArm(ID)) {

        return armsModel_.at(ID);
    }
    std::string how;
    how = "Asking not existing arm: " + ID;
    RobotModelArmException notExistingArmException;
    notExistingArmException.SetHow(how);
    throw(notExistingArmException);
}

Eigen::VectorXd RobotModel::GetSystemPositionVector()
{
    Eigen::VectorXd pos;
    if (isVehicle_) {
        pos = UnderJuxtapose(pos, vehicle_->GetPositionOnInertial());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        pos = UnderJuxtapose(pos, iter->second->GetJointsPosition());
    }
    return pos;
}

Eigen::VectorXd RobotModel::GetSystemVelocityVector()
{
    Eigen::VectorXd vel;
    if (isVehicle_) {
        vel = UnderJuxtapose(vel, vehicle_->GetVelocityOnVehicle());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        vel = UnderJuxtapose(vel, iter->second->GetJointsVelocity());
    }
    return vel;
}

void RobotModel::SetSystemPositionVector(Eigen::VectorXd position) throw(std::exception)
{
    int startIndex = 0;
    if (position.size() == DoF_) {
        if (isVehicle_) {
            vehicle_->SetPositionOnInertial(position.block(0, 0, 6, 1));
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->SetJointsPosition(position.block(startIndex, 0, iter->second->GetNumJoints(), 1));
            startIndex = startIndex + iter->second->GetNumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

void RobotModel::SetSystemVelocityVector(Eigen::VectorXd velocity) throw(std::exception)
{

    int startIndex = 0;
    if (velocity.size() == DoF_) {
        if (isVehicle_) {
            vehicle_->SetVelocityOnVehicle(velocity.block(0, 0, 6, 1));
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->SetJointsVelocity(velocity.block(startIndex, 0, iter->second->GetNumJoints(), 1));
            startIndex = startIndex + iter->second->GetNumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

Eigen::VectorXd RobotModel::GetSystemAccelerationVector()
{
    Eigen::VectorXd acc;
    if (isVehicle_) {
        acc = UnderJuxtapose(acc, vehicle_->GetAccelerationOnVehicle());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        acc = UnderJuxtapose(acc, iter->second->GetJointsAcceleration());
    }
    return acc;
}

void RobotModel::SetSystemAccelerationVector(Eigen::VectorXd acceleration) throw(std::exception)
{

    int startIndex = 0;
    if (acceleration.size() == DoF_) {
        if (isVehicle_) {
            vehicle_->SetAccelerationOnVehicle(acceleration.block(0, 0, 6, 1));
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->SetJointsAcceleration(acceleration.block(startIndex, 0, iter->second->GetNumJoints(), 1));
            startIndex = startIndex + iter->second->GetNumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

void RobotModel::SetPositionVector(std::string partID, Eigen::VectorXd position) throw(ExceptionWithHow)
{

    if (partID == robotFrameID_ & isVehicle_) {
        vehicle_->SetPositionOnInertial(position);
    } else if (CheckArm(partID)) {
        armsModel_.find(partID)->second->SetJointsPosition(position);
    } else {
        std::string how;
        how = "Asking a not existing part: " + partID;
        RobotModelWrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::VectorXd RobotModel::GetPositionVector(std::string partID) throw(ExceptionWithHow)
{
    Eigen::VectorXd out;

    if (partID == robotFrameID_ & isVehicle_) {
        out = vehicle_->GetPositionOnInertial();
        return out;
    } else if (CheckArm(partID)) {
        out = armsModel_.find(partID)->second->GetJointsPosition();
        return out;
    }
    std::string how;
    how = "Asking a not existing part: " + partID;
    RobotModelWrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}

void RobotModel::SetVelocityVector(std::string partID, Eigen::VectorXd velocity) throw(ExceptionWithHow)
{
    if (partID == robotFrameID_ & isVehicle_) {
        vehicle_->SetVelocityOnVehicle(velocity);
    } else if (CheckArm(partID)) {
        armsModel_.find(partID)->second->SetJointsVelocity(velocity);
    } else {
        std::string how;
        how = "Asking a not existing part: " + partID;
        RobotModelWrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::VectorXd RobotModel::GetVelocityVector(std::string partID) throw(ExceptionWithHow)
{
    Eigen::VectorXd out;

    if (partID == robotFrameID_ & isVehicle_) {
        out = vehicle_->GetVelocityOnVehicle();
        return out;
    } else if (CheckArm(partID)) {
        out = armsModel_.find(partID)->second->GetJointsVelocity();
        return out;
    }
    std::string how;
    how = "Asking a not existing part: " + partID;
    RobotModelWrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}

void RobotModel::SetAccelerationVector(std::string partID, Eigen::VectorXd acceleration) throw(ExceptionWithHow)
{
    if (partID == robotFrameID_ & isVehicle_) {
        vehicle_->SetAccelerationOnVehicle(acceleration);
    } else if (CheckArm(partID)) {
        armsModel_.find(partID)->second->SetJointsAcceleration(acceleration);
    } else {
        std::string how;
        how = "Asking a not existing part: " + partID;
        RobotModelWrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::VectorXd RobotModel::GetAccelerationVector(std::string partID) throw(ExceptionWithHow)
{
    Eigen::VectorXd out;

    if (partID == robotFrameID_ & isVehicle_) {
        out = vehicle_->GetAccelerationOnVehicle();
        return out;
    } else if (CheckArm(partID)) {
        out = armsModel_.find(partID)->second->GetJointsAcceleration();
        return out;
    }
    std::string how;
    how = "Asking a not existing part: " + partID;
    RobotModelWrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}
}

/* namespace rml */

/*const std::shared_ptr<VehicleModel> RobotModel::GetVehicle() const throw(ExceptionWithHow)
{
    if (isVehicle_) {
        return vehicle_;
    }
    std::string how;
    how = "The vehicle does not exist";
    RobotModelVehicleException notExistingVehicleExc;
    notExistingVehicleExc.SetHow(how);
    throw(notExistingVehicleExc);
}*/
