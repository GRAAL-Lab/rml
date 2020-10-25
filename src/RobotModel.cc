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

RobotModel::RobotModel(Eigen::TransformationMatrix inertialF_T_bodyF, std::string bodyFrameID)
    : bodyFrameID_ { std::move(bodyFrameID) }
    , inertialF_T_bodyF_ { std::move(inertialF_T_bodyF) }
    , DoF_ { 0 }
    , isMobileRobot_ { false }
{
    robotBase_ = std::make_shared<rml::VehicleModel>(rml::VehicleModel(bodyFrameID_));
    robotBase_->Jacobian(Eigen::MatrixXd::Zero(6, 6));
    robotBase_->PositionOnInertialFrame(inertialF_T_bodyF);
}

RobotModel::RobotModel(Eigen::TransformationMatrix inertialF_T_bodyF, std::string bodyFrameID, Eigen::MatrixXd bodyF_JBodyFrame)
    : bodyFrameID_ { std::move(bodyFrameID) }
    , inertialF_T_bodyF_ { std::move(inertialF_T_bodyF) }
    , isMobileRobot_ { true }
{
    robotBase_ = std::make_shared<rml::VehicleModel>(rml::VehicleModel(bodyFrameID_));
    robotBase_->Jacobian(bodyF_JBodyFrame);
    robotBase_->PositionOnInertialFrame(inertialF_T_bodyF_);
    DoF_ = 6;
}

RobotModel::~RobotModel()
{
}

bool RobotModel::LoadArm(const std::shared_ptr<ArmModel>& arm, const Eigen::TransformationMatrix& bodyframeToArm) noexcept(false)
{
    if (arm->IsModelInitialized()) {
        if (CheckArm(arm->ID())) {
            std::string how;
            how = "Already existing arm: " + arm->ID() + " in the RobotModel!";
            RobotModelArmException conflictingArmModelException;
            conflictingArmModelException.SetHow(how);
            throw(conflictingArmModelException);
        }
        armsModel_.insert(std::make_pair(arm->ID(), arm));

        bodyFrameToArm_.insert(std::make_pair(arm->ID(), bodyframeToArm));

        DoF_ += arm->NumJoints();

        robotBase_->AttachRigidBodyFrame(arm->ID(), bodyframeToArm);

        return true;

    } else {
        std::string how;
        how = "ArmModel: " + arm->ID() + " is NOT initialized";
        RobotModelArmException notInitializedArmModelExceptions;
        notInitializedArmModelExceptions.SetHow(how);
        throw(notInitializedArmModelExceptions);
    }
}

void RobotModel::PositionOnInertialFrame(Eigen::TransformationMatrix inertialF_T_bodyF)
{
    inertialF_T_bodyF_ = std::move(inertialF_T_bodyF);
    robotBase_->PositionOnInertialFrame(inertialF_T_bodyF_);
}

void RobotModel::AttachRigidBodyFrame(const std::string& frameID, const std::string& frameToAttachID, const Eigen::TransformationMatrix& frameToAttachID_T_frameID) noexcept(false)
{
    std::size_t partIDIndex = frameToAttachID.find_first_of("_");
    std::string partID = frameToAttachID.substr(0, partIDIndex);
    Eigen::TransformationMatrix T;

    if (partIDIndex == std::string::npos && frameToAttachID != bodyFrameID_) {
        std::string how;
        how = "wrong string format: " + frameToAttachID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    } else if (frameToAttachID == bodyFrameID_ || partID == bodyFrameID_) {
        robotBase_->AttachRigidBodyFrame(frameID, frameToAttachID_T_frameID);
    } else if (CheckArm(partID)) {
        armsModel_.at(partID)->AttachRigidBodyFrame(frameID, frameToAttachID, frameToAttachID_T_frameID);
    } else {
        std::string how;
        how = "[ROBOT MODEL] Asking a not existing part: " + partID;
        WrongFrameException robotModelNotExistingPartExc;
        robotModelNotExistingPartExc.SetHow(how);
        throw(robotModelNotExistingPartExc);
    }
}

bool RobotModel::CheckArm(const std::string& armID) const
{
    if (armsModel_.find(armID) != armsModel_.end()) {
        return true;
    } else {
        return false;
    }
}

Eigen::MatrixXd RobotModel::ArmJacobian(const std::string& frameID) const noexcept(false)
{
    Eigen::MatrixXd bJt, rJt;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (CheckArm(partID)) {
        bJt = armsModel_.at(partID)->Jacobian(frameID);
        Eigen::RotationMatrix rRb = bodyFrameToArm_.at(partID).RotationMatrix();
        rJt = rRb.CartesianRotationMatrix() * bJt;
        return rJt;
    } else {
        std::string how;
        how = "Not existing arm: " + partID;
        RobotModelArmException notExistingArmException;
        notExistingArmException.SetHow(how);
        throw(notExistingArmException);
    }
}

Eigen::Matrix6d RobotModel::BaseJacobian(const std::string& frameID) const
{
    Eigen::Matrix6d vJv;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (isMobileRobot_) {
        if (CheckArm(partID)) {
            vJv = robotBase_->Jacobian(bodyFrameID_);
            Eigen::TransformationMatrix bTj = armsModel_.at(partID)->TransformationMatrix(frameID);
            Eigen::TransformationMatrix vTj = bodyFrameToArm_.at(partID) * bTj;
            vJv = RigidBodyMatrix(vTj.TranslationVector()) * vJv;
        }
    }
    return vJv;
}

void RobotModel::ControlVector(const Eigen::VectorXd& y) noexcept(false)
{
    unsigned int startIndex = 0;
    if (y.size() == DoF_) {
        if (isMobileRobot_) {
            robotBase_->ControlVector() = y.block(0, 0, 6, 1);
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end(); ++iter) {
            iter->second->ControlVector(y.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

Eigen::VectorXd RobotModel::ControlVector(const std::string& partID) noexcept(false)
{
    Eigen::VectorXd y;

    if (partID == bodyFrameID_ && isMobileRobot_) {
        return robotBase_->ControlVector();
    } else if (CheckArm(partID)) {
        return (armsModel_.at(partID)->ControlVector());
    } else {
        std::string how;
        how = "[ROBOT MODEL] Asking a not existing part: " + partID;
        WrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::TransformationMatrix RobotModel::TransformationMatrix(const std::string& frameID) noexcept(false)
{

    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (frameID == bodyFrameID_ || partID == bodyFrameID_) {
        return robotBase_->TransformationMatrix(frameID);
    } else if (CheckArm(partID)) {
        return inertialF_T_bodyF_ * bodyFrameToArm_.at(partID) * armsModel_.at(partID)->TransformationMatrix(frameID);
    } else if (frameID == rml::FrameID::WorldFrame) {
        return Eigen::TransformationMatrix::Identity();
    }

    if (partIDIndex == std::string::npos && frameID != bodyFrameID_) {
        std::string how;
        how = "[ROBOT MODEL] Wrong string format: " + frameID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }

    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotModelNotExistingPartExc;
    robotModelNotExistingPartExc.SetHow(how);
    throw(robotModelNotExistingPartExc);
}

Eigen::TransformationMatrix RobotModel::TransformationMatrix(const std::string& frameID_j, const std::string& frameID_k)
{
    if (frameID_j == frameID_k) {
        return Eigen::TransformationMatrix::Identity();
    } else if (frameID_j == rml::FrameID::WorldFrame) {
        return TransformationMatrix(frameID_k);
    } else if (frameID_k == rml::FrameID::WorldFrame) {
        return TransformationMatrix(frameID_j).inverse();
    } else {
        return TransformationMatrix(frameID_j).inverse() * TransformationMatrix(frameID_k);
    }
}

Eigen::MatrixXd RobotModel::CartesianJacobian(const std::string& frameID) noexcept(false)
{
    std::string modelID = frameID.substr(0, frameID.find_first_of("_"));
    Eigen::MatrixXd totJac, tempJ;
    if (frameID == bodyFrameID_ || modelID == bodyFrameID_) {
        if (isMobileRobot_) {
            totJac = RightJuxtapose(totJac, robotBase_->Jacobian(frameID));
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(6, DoF_ - 6));
        } else {
            totJac = Eigen::MatrixXd::Zero(6, DoF_);
        }
        return totJac;
    } else if (CheckArm(modelID)) {
        if (isMobileRobot_) {
            totJac = RightJuxtapose(totJac, BaseJacobian(frameID));
        }
        for (auto iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == modelID) {
                //in get isolated arm jacobian the vTb is taken into account
                tempJ = ArmJacobian(frameID);
            } else {
                tempJ = Eigen::MatrixXd::Zero(6, armsModel_.at(modelID)->NumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
        return totJac;
    } else if (frameID.find_first_of("_") == std::string::npos && frameID != bodyFrameID_) {
        std::string how;
        how = "[ROBOT MODEL] Wrong format frame id: " + frameID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + modelID;
    WrongFrameException robotModelNotExistingPartException;
    robotModelNotExistingPartException.SetHow(how);
    throw(robotModelNotExistingPartException);
}

Eigen::MatrixXd RobotModel::JointSpaceJacobian(const std::string& armID) noexcept(false)
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm(armID)) {
        unsigned int taskSize = armsModel_.at(armID)->NumJoints();
        if (robotBase_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(taskSize, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == armID) {
                tempJ = Eigen::MatrixXd::Identity(taskSize, taskSize);
            } else {
                tempJ = Eigen::MatrixXd::Zero(taskSize, armsModel_.at(armID)->NumJoints());
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
Eigen::MatrixXd RobotModel::ManipulabilityJacobian(const std::string& frameID) noexcept(false)
{
    Eigen::MatrixXd totJac, tempJ;
    std::string armID = frameID.substr(0, frameID.find_first_of("_"));
    if (frameID.find_first_of("_") == std::string::npos) {
        std::string how;
        how = "[ROBOT MODEL] Wrong format frame id: " + frameID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    if (CheckArm(armID)) {
        if (isMobileRobot_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(1, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {

            if (iter->first == armID) {
                tempJ = iter->second->ManipulabilityJacobian(frameID);

            } else {
                tempJ = Eigen::MatrixXd::Zero(1, armsModel_.at(iter->first)->NumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
        return totJac;
    }
    std::string how;
    how = "[ROBOT MODEL] Asking frame for a no existing arm: " + armID;
    WrongFrameException notExistingPartExc;
    notExistingPartExc.SetHow(how);
    throw(notExistingPartExc);
}

double RobotModel::Manipulability(const std::string& frameID) noexcept(false)
{
    std::string armID = frameID.substr(0, frameID.find_first_of("_"));
    if (frameID.find_first_of("_") == std::string::npos) {
        std::string how;
        how = "[ROBOT MODEL] Wrong format frame id: " + frameID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    if (CheckArm(armID)) {
        return armsModel_.at(armID)->Manipulability(frameID);
    }
    RobotModelArmException notExistingArm;
    std::string how;
    how = "Asking a not existing arm: " + armID;
    notExistingArm.SetHow(how);
    throw(notExistingArm);
}

const std::shared_ptr<ArmModel>& RobotModel::Arm(const std::string& ID) const noexcept(false)
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

Eigen::VectorXd RobotModel::PositionVector()
{
    Eigen::VectorXd pos;
    if (isMobileRobot_) {
        pos = UnderJuxtapose(pos, robotBase_->PositionOnInertialFrame().ToVector());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        pos = UnderJuxtapose(pos, iter->second->JointsPosition());
    }
    return pos;
}

void RobotModel::PositionVector(Eigen::VectorXd& position) noexcept(false)
{
    unsigned int startIndex = 0;
    if (position.size() == DoF_) {
        if (isMobileRobot_) {
            Eigen::TransformationMatrix inertialF_T_vehicleF;
            inertialF_T_vehicleF.TranslationVector(position.head(3));
            inertialF_T_vehicleF.RotationMatrix(rml::EulerRPY(position.segment(3, 3)).ToRotationMatrix());
            robotBase_->PositionOnInertialFrame(inertialF_T_vehicleF);
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end(); ++iter) {
            iter->second->JointsPosition(position.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

Eigen::VectorXd RobotModel::VelocityVector()
{
    Eigen::VectorXd vel;
    if (isMobileRobot_) {
        vel = UnderJuxtapose(vel, robotBase_->VelocityVector());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end(); ++iter) {
        vel = UnderJuxtapose(vel, iter->second->JointsVelocity());
    }
    return vel;
}

void RobotModel::VelocityVector(const Eigen::VectorXd& velocity) noexcept(false)
{
    unsigned int startIndex = 0;
    if (velocity.size() == DoF_) {
        if (isMobileRobot_) {
            robotBase_->VelocityVector() = velocity.head(6);
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end(); ++iter) {
            iter->second->JointsVelocity(velocity.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

Eigen::VectorXd RobotModel::AccelerationVector()
{
    Eigen::VectorXd acc;
    if (isMobileRobot_) {
        acc = UnderJuxtapose(acc, robotBase_->AccelerationVector());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end(); ++iter) {
        acc = UnderJuxtapose(acc, iter->second->JointsAcceleration());
    }
    return acc;
}

void RobotModel::AccelerationVector(const Eigen::VectorXd& acceleration) noexcept(false)
{
    unsigned int startIndex = 0;
    if (acceleration.size() == DoF_) {
        if (isMobileRobot_) {
            robotBase_->AccelerationVector() = acceleration.head(6);
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end(); ++iter) {
            iter->second->JointsAcceleration(acceleration.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

void RobotModel::PositionVector(const std::string& partID, const Eigen::VectorXd& position) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        Eigen::TransformationMatrix inertialF_T_vehicleF;
        inertialF_T_vehicleF.TranslationVector(position.head(3));
        inertialF_T_vehicleF.RotationMatrix(rml::EulerRPY(position.tail(3)).ToRotationMatrix());
        robotBase_->PositionOnInertialFrame(inertialF_T_vehicleF);
    } else if (CheckArm(partID)) {
        armsModel_.find(partID)->second->JointsPosition(position);
    } else {
        std::string how;
        how = "[ROBOT MODEL] Asking a not existing part: " + partID;
        WrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::VectorXd RobotModel::PositionVector(const std::string& partID) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        return robotBase_->PositionOnInertialFrame().ToVector();
    } else if (CheckArm(partID)) {
        return armsModel_.find(partID)->second->JointsPosition();
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}

void RobotModel::VelocityVector(const std::string& partID, const Eigen::VectorXd& velocity) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        robotBase_->VelocityVector() = velocity;
    } else if (CheckArm(partID)) {
        armsModel_.find(partID)->second->JointsVelocity(velocity);
    } else {
        std::string how;
        how = "[ROBOT MODEL] Asking a not existing part: " + partID;
        WrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::VectorXd RobotModel::VelocityVector(const std::string& partID) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        return robotBase_->VelocityVector();
    } else if (CheckArm(partID)) {
        return armsModel_.find(partID)->second->JointsVelocity();
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}

void RobotModel::AccelerationVector(const std::string& partID, const Eigen::VectorXd& acceleration) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        robotBase_->AccelerationVector() = acceleration;
    } else if (CheckArm(partID)) {
        armsModel_.find(partID)->second->JointsAcceleration(acceleration);
    } else {
        std::string how;
        how = "[ROBOT MODEL] Asking a not existing part: " + partID;
        WrongFrameException robotNotExistingPartException;
        robotNotExistingPartException.SetHow(how);
        throw(robotNotExistingPartException);
    }
}

Eigen::VectorXd RobotModel::AccelerationVector(const std::string& partID) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        return robotBase_->AccelerationVector();
    } else if (CheckArm(partID)) {
        return armsModel_.find(partID)->second->JointsAcceleration();
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}
}
