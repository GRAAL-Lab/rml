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
    //vehicleFrameID_ = frameID;
    vehicle_->SetPositionOnInertial(robotFrame.GetRPYXYZ());
}

RobotModel::~RobotModel()
{
    // TODO Auto-generated destructor stub
}

int RobotModel::GetTotalDOFs()
{
    int totDOFs(0);
    if (isVehicle_) {
        totDOFs += 6;
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        totDOFs += iter->second->GetNumJoints();
    }
    return totDOFs;
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

void RobotModel::SetRigidBodyFrameToRobotFrame(const std::string ID, const Eigen::TransfMatrix TMat)
{

    vehicle_->SetRigidBodyFrame(ID, TMat);
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

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForFrame(const std::string& frameID) const throw(std::exception)
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

void RobotModel::SetRobotControl(const Eigen::VectorXd& y) throw(std::exception)
{
    int startIndex = 0;
    if (y.size() == GetTotalDOFs()) {
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
//TODO
Eigen::VectorXd RobotModel::GetRobotControl(std::string partID) throw(std::exception)
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

Eigen::TransfMatrix RobotModel::GetTransformation(const std::string& frameID) throw(std::exception)
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

Eigen::MatrixXd RobotModel::GetCartesianJacobian(const std::string& frameID) throw(std::exception)
{

    std::string modelID = frameID.substr(0, frameID.find_first_of("_"));
    Eigen::MatrixXd totJac, tempJ;
    if (frameID == robotFrameID_ || modelID == robotFrameID_) {
        if (isVehicle_) {
            totJac = RightJuxtapose(totJac, vehicle_->GetJacobian(frameID));
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(6, GetTotalDOFs() - 6));
        } else {
            totJac = Eigen::MatrixXd::Zero(6, GetTotalDOFs());
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

Eigen::MatrixXd RobotModel::GetJointSpaceJacobian(const std::string& armID) throw(std::exception)

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
Eigen::MatrixXd RobotModel::GetManipulabilityJacobian(const std::string& frameID) throw(std::exception)
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

double RobotModel::GetManipulability(const std::string& frameID) throw(std::exception)
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

const std::shared_ptr<ArmModel> RobotModel::GetArm(std::string ID) const throw(std::exception)
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

const std::shared_ptr<VehicleModel> RobotModel::GetVehicle() const throw(std::exception)
{
    if (isVehicle_) {
        return vehicle_;
    }
    std::string how;
    how = "The vehicle does not exist";
    RobotModelVehicleException notExistingVehicleExc;
    notExistingVehicleExc.SetHow(how);
    throw(notExistingVehicleExc);
}
}

/*bool RobotModel::LoadVehicle(const std::shared_ptr<VehicleModel> vehicle) throw(ExceptionWithHow)
{
    if (vehicle->IsModelInitialized()) {
        if (vehicle_) {
            std::string how;
            how = "Vehicle Model already loaded with id " + robotFrameID_;
            RobotModelVehicleException vehicleModelAlreadyLoadedException;
            vehicleModelAlreadyLoadedException.SetHow(how);
            throw(vehicleModelAlreadyLoadedException);
        } else {
            vehicle_ = vehicle;
            robotFrameID_ = vehicle->GetID();
            vehicleFrameID_ = vehicle->GetID();
            return true;
        }
    } else {
        std::string how;
        how = "Vehicle Model " + vehicle->GetID() + " not initialized";
        RobotModelVehicleException vehicleModelNotInitializedException;
        vehicleModelNotInitializedException.SetHow(how);
        throw(vehicleModelNotInitializedException);
    }
}
*/
/* namespace rml */
