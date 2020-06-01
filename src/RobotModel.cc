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

RobotModel::RobotModel(Eigen::TransfMatrix bodyFrame, std::string frameID)
    : bodyFrameID_(frameID)
    , bodyFrame_(bodyFrame)
    , DoF_(0)
    , isMobileRobot_(false)
{
    robotBase_ = std::make_shared<rml::VehicleModel>(rml::VehicleModel(frameID));
    robotBase_->Jacobian(Eigen::MatrixXd::Zero(6, 6));
    robotBase_->PositionOnInertial(bodyFrame.XYZ_RPY());
}

RobotModel::RobotModel(Eigen::TransfMatrix bodyFrame, std::string frameID, Eigen::MatrixXd JRobotFrame)
    : bodyFrameID_(frameID)
    , bodyFrame_(bodyFrame)
    , isMobileRobot_(true)
{
    robotBase_ = std::make_shared<rml::VehicleModel>(rml::VehicleModel(frameID));
    robotBase_->Jacobian(JRobotFrame);
    bodyFrameID_ = frameID;
    robotBase_->PositionOnInertial(bodyFrame.XYZ_RPY());
    DoF_ = 6;
}

RobotModel::~RobotModel()
{
    // TODO Auto-generated destructor stub
}

bool RobotModel::LoadArm(const std::shared_ptr<ArmModel>& arm, const Eigen::TransfMatrix& bodyframeToArm) noexcept(false)
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
        robotBase_->SetRigidBodyFrame(arm->ID(), bodyframeToArm);
        return true;

    } else {
        std::string how;
        how = "ArmModel: " + arm->ID() + " is NOT initialized";
        RobotModelArmException notInitializedArmModelExceptions;
        notInitializedArmModelExceptions.SetHow(how);
        throw(notInitializedArmModelExceptions);
    }
}

void RobotModel::SetBodyFramePosition(Eigen::TransfMatrix bodyFrame)
{
    bodyFrame_ = std::move(bodyFrame);
    robotBase_->PositionOnInertial(bodyFrame_.XYZ_RPY());
}

void RobotModel::SetAttachedRigidBodyFrame(const std::string& frameID, const Eigen::TransfMatrix& TMat, const std::string& frameToAttachID) noexcept(false)
{
    std::size_t partIDIndex = frameToAttachID.find_first_of("_");
    std::string partID = frameToAttachID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;

    if (partIDIndex == std::string::npos && frameToAttachID != bodyFrameID_) {
        std::string how;
        how = "wrong string format: " + frameToAttachID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    } else if (frameToAttachID == bodyFrameID_ || partID == bodyFrameID_) {
        robotBase_->SetRigidBodyFrame(frameID, TMat);
    } else if (CheckArm(partID)) {
        armsModel_.at(partID)->SetRigidBodyFrame(frameID, frameToAttachID, TMat);
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

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForFrame(const std::string& frameID) const noexcept(false)
{
    Eigen::MatrixXd bJt, rJt;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (CheckArm(partID)) {
        bJt = armsModel_.at(partID)->Jacobian(frameID);
        Eigen::RotMatrix rRb = bodyFrameToArm_.at(partID).RotationMatrix();
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

Eigen::Matrix6d RobotModel::GetIsolatedRobotFrameJacobianForFrame(const std::string& frameID) const
{
    Eigen::Matrix6d vJv;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (isMobileRobot_) {
        if (CheckArm(partID)) {
            vJv = robotBase_->Jacobian(bodyFrameID_);
            Eigen::TransfMatrix bTj = armsModel_.at(partID)->GetTransformation(frameID);
            Eigen::TransfMatrix vTj = bodyFrameToArm_.at(partID) * bTj;
            vJv = GetRigidBodyMatrix(vTj.Transl()) * vJv;
        }
    }
    return vJv;
}

void RobotModel::SetRobotControl(const Eigen::VectorXd& y) noexcept(false)
{
    int startIndex = 0;
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

Eigen::VectorXd RobotModel::GetRobotControl(const std::string& partID) noexcept(false)
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

Eigen::TransfMatrix RobotModel::GetTransformation(const std::string& frameID) noexcept(false)
{

    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);
    Eigen::TransfMatrix T = Eigen::TransfMatrix::Identity();

    if (partIDIndex == std::string::npos && frameID != bodyFrameID_) {
        std::string how;
        how = "[ROBOT MODEL] Wrong string format: " + frameID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }

    if (frameID == bodyFrameID_ || partID == bodyFrameID_) {
        return robotBase_->GetTransformation(frameID);
    } else if (CheckArm(partID)) {
        T = armsModel_.at(partID)->GetTransformation(frameID);
        T = bodyFrame_ * bodyFrameToArm_.at(partID) * T;
        return T;
    } else if (frameID == rml::FrameID::WorldFrame) {
        return T;
    }

    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotModelNotExistingPartExc;
    robotModelNotExistingPartExc.SetHow(how);
    throw(robotModelNotExistingPartExc);
}

Eigen::TransfMatrix RobotModel::GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k)
{
    Eigen::TransfMatrix out;
    if (frameID_j == frameID_k) {
        out.setIdentity();
    } else if (frameID_j == rml::FrameID::WorldFrame) {
        out = GetTransformation(frameID_k);

    } else if (frameID_k == rml::FrameID::WorldFrame) {
        out = GetTransformation(frameID_j).inverse();
    } else {
        Eigen::TransfMatrix wTj, wTk;
        std::string partID_a = frameID_j.substr(0, frameID_j.find_first_of("_"));
        std::string partID_b = frameID_k.substr(0, frameID_k.find_first_of("_"));

        wTj = GetTransformation(frameID_j);
        wTk = GetTransformation(frameID_k);

        out = wTj.inverse() * wTk;
    }
    return out;
}

Eigen::MatrixXd RobotModel::GetCartesianJacobian(const std::string& frameID) noexcept(false)
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
            totJac = RightJuxtapose(totJac, GetIsolatedRobotFrameJacobianForFrame(frameID));
        }
        for (auto iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == modelID) {
                //in get isolated arm jacobian the vTb is taken into account
                tempJ = GetIsolatedArmJacobianForFrame(frameID);
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

Eigen::MatrixXd RobotModel::GetJointSpaceJacobian(const std::string& armID) noexcept(false)
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm(armID)) {
        int taskSize = armsModel_.at(armID)->NumJoints();
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
Eigen::MatrixXd RobotModel::GetManipulabilityJacobian(const std::string& frameID) noexcept(false)
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

double RobotModel::GetManipulability(const std::string& frameID) noexcept(false)
{
    double out;
    std::string armID = frameID.substr(0, frameID.find_first_of("_"));
    if (frameID.find_first_of("_") == std::string::npos) {
        std::string how;
        how = "[ROBOT MODEL] Wrong format frame id: " + frameID;
        WrongFrameException robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetHow(how);
        throw(robotModelWrongFrameFormat);
    }
    if (CheckArm(armID)) {
        out = armsModel_.at(armID)->Manipulability(frameID);
        return out;
    }
    RobotModelArmException notExistingArm;
    std::string how;
    how = "Asking a not existing arm: " + armID;
    notExistingArm.SetHow(how);
    throw(notExistingArm);
}

const std::shared_ptr<ArmModel>& RobotModel::GetArm(const std::string& ID) const noexcept(false)
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
    if (isMobileRobot_) {
        pos = UnderJuxtapose(pos, robotBase_->PositionOnInertial());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        pos = UnderJuxtapose(pos, iter->second->JointsPosition());
    }
    return pos;
}

Eigen::VectorXd RobotModel::GetSystemVelocityVector()
{
    Eigen::VectorXd vel;
    if (isMobileRobot_) {
        vel = UnderJuxtapose(vel, robotBase_->VelocityOnVehicle());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        vel = UnderJuxtapose(vel, iter->second->JointsVelocity());
    }
    return vel;
}

void RobotModel::SetSystemPositionVector(Eigen::VectorXd& position) noexcept(false)
{
    int startIndex = 0;
    if (position.size() == DoF_) {
        if (isMobileRobot_) {
            robotBase_->PositionOnInertial(position.block(0, 0, 6, 1));
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->JointsPosition(position.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

void RobotModel::SetSystemVelocityVector(const Eigen::VectorXd& velocity) noexcept(false)
{
    int startIndex = 0;
    if (velocity.size() == DoF_) {
        if (isMobileRobot_) {
            robotBase_->VelocityOnVehicle() = velocity.block(0, 0, 6, 1);
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->JointsVelocity(velocity.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

Eigen::VectorXd RobotModel::GetSystemAccelerationVector()
{
    Eigen::VectorXd acc;
    if (isMobileRobot_) {
        acc = UnderJuxtapose(acc, robotBase_->AccelerationOnVehicle());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        acc = UnderJuxtapose(acc, iter->second->JointsAcceleration());
    }
    return acc;
}

void RobotModel::SetSystemAccelerationVector(const Eigen::VectorXd& acceleration) noexcept(false)
{

    int startIndex = 0;
    if (acceleration.size() == DoF_) {
        if (isMobileRobot_) {
            robotBase_->AccelerationOnVehicle() = acceleration.block(0, 0, 6, 1);
            startIndex = 6;
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel>>::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            iter->second->JointsAcceleration(acceleration.block(startIndex, 0, iter->second->NumJoints(), 1));
            startIndex = startIndex + iter->second->NumJoints();
        }
    } else {
        throw(RobotModelWrongControlSizeVectorException());
    }
}

void RobotModel::SetPositionVector(const std::string& partID, const Eigen::VectorXd& position) noexcept(false)
{

    if (partID == bodyFrameID_ & isMobileRobot_) {
        robotBase_->PositionOnInertial(position);
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

Eigen::VectorXd RobotModel::GetPositionVector(const std::string& partID) noexcept(false)
{
    Eigen::VectorXd out;

    if (partID == bodyFrameID_ & isMobileRobot_) {
        out = robotBase_->PositionOnInertial();
        return out;
    } else if (CheckArm(partID)) {
        out = armsModel_.find(partID)->second->JointsPosition();
        return out;
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}

void RobotModel::SetVelocityVector(const std::string& partID, const Eigen::VectorXd& velocity) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        robotBase_->VelocityOnVehicle() = velocity;
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

Eigen::VectorXd RobotModel::GetVelocityVector(const std::string& partID) noexcept(false)
{
    Eigen::VectorXd out;

    if (partID == bodyFrameID_ & isMobileRobot_) {
        out = robotBase_->VelocityOnVehicle();
        return out;
    } else if (CheckArm(partID)) {
        out = armsModel_.find(partID)->second->JointsVelocity();
        return out;
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}

void RobotModel::SetAccelerationVector(const std::string& partID, const Eigen::VectorXd& acceleration) noexcept(false)
{
    if (partID == bodyFrameID_ & isMobileRobot_) {
        robotBase_->AccelerationOnVehicle() = acceleration;
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

Eigen::VectorXd RobotModel::GetAccelerationVector(const std::string& partID) noexcept(false)
{
    Eigen::VectorXd out;

    if (partID == bodyFrameID_ & isMobileRobot_) {
        out = robotBase_->AccelerationOnVehicle();
        return out;
    } else if (CheckArm(partID)) {
        out = armsModel_.find(partID)->second->JointsAcceleration();
        return out;
    }
    std::string how;
    how = "[ROBOT MODEL] Asking a not existing part: " + partID;
    WrongFrameException robotNotExistingPartException;
    robotNotExistingPartException.SetHow(how);
    throw(robotNotExistingPartException);
}
}
