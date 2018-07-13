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

RobotModel::RobotModel()
{
}

RobotModel::~RobotModel()
{
    // TODO Auto-generated destructor stub
}

int RobotModel::GetTotalDOFs()
{
    int totDOFs(0);
    if (vehicle_) {
        totDOFs += 6;
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        totDOFs += iter->second->GetNumJoints();
    }
    return totDOFs;
}

bool RobotModel::LoadVehicle(const std::shared_ptr<VehicleModel> vehicle) throw(std::exception)
{
    if (vehicle->IsModelInitialized()&& !vehicle_) {
        vehicle_ = vehicle;
        vehicleID_ = vehicle->GetID();
        return true;
    } else {
        throw(RobotModelNotInitializedVehicleModelException());
        return false;
    }
}

bool RobotModel::LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb) throw(std::exception)
{
    if (arm->IsModelInitialized()) {
        for (auto& iter : armsModel_) {
            if (iter.first == arm->GetID()) {
                RobotModelConflictingArmModelIDException conflictingArmModelException;
                conflictingArmModelException.SetID(arm->GetID());
                throw(conflictingArmModelException);
                return false;
            }
        }
        armsModel_.insert(std::make_pair(arm->GetID(), arm));
        vehicleToBase_.insert(std::make_pair(arm->GetID(), vTb));
        return (true);
    } else {
        throw(RobotModelNotInitializedArmModelException());
        return false;
    }
}

bool RobotModel::CheckArm(const std::string armID) const throw(std::exception)
{
    // if give exception cannot be used for actually checkind if the robot is present.

    if (armsModel_.find(armID) != armsModel_.end()) {

        return true;
    } else {
        RobotModelArmException robotModelArmException;
        robotModelArmException.SetID(armID);
        throw(robotModelArmException);
        return false;
    }
}

bool RobotModel::CheckVehicle() const throw(std::exception)
{
    if (vehicle_) {
        return true;
    } else {
        throw RobotModelVehicleException();
    }
}

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForFrame(const std::string& frameID) const
{
    Eigen::MatrixXd bJt;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (CheckArm(partID)) {
        bJt = armsModel_.at(partID)->GetJacobian(frameID);
        Eigen::RotMatrix vRb = vehicleToBase_.at(partID).GetRotMatrix();
        bJt = vRb.GetCartesianRotationMatrix() * bJt;
    }
    return bJt;
}

Eigen::Matrix6d RobotModel::GetIsolatedVehicleJacobianForFrame(const std::string& frameID) const
{
    Eigen::Matrix6d vJv;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (vehicle_) {
        if (CheckArm(partID)) {
            vJv = vehicle_->GetJacobian(vehicleID_);
            Eigen::TransfMatrix bTj = armsModel_.at(partID)->GetTransformation(frameID);
            Eigen::TransfMatrix vTj = vehicleToBase_.at(partID) * bTj;

            vJv = GetRigidBodyMatrix(vTj.GetTransl()) * vJv;
        }
    }
    return vJv;
}

Eigen::VectorXd RobotModel::GetSystemPositionVector()
{
    Eigen::VectorXd pos;
    if (vehicle_) {
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
    if (vehicle_) {
        vel = UnderJuxtapose(vel, vehicle_->GetVelocityOnVehicle());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        vel = UnderJuxtapose(vel, iter->second->GetJointsVelocity());
    }
    return vel;
}

void RobotModel::SetRobotControl(const Eigen::VectorXd& y)
{
    int startIndex = 0;
    if (vehicle_) {
        vehicle_->SetControlVector(y.block(0, 0, 6, 1));
        startIndex = 6;
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        iter->second->SetControlVector(y.block(startIndex, 0, iter->second->GetNumJoints(), 1));
        startIndex = startIndex + iter->second->GetNumJoints();
    }
}

Eigen::VectorXd RobotModel::GetRobotControl(std::string partID)
{
    Eigen::VectorXd y;
    //TODO IF VEHICLE
    if (partID == vehicleID_) {
        return vehicle_->GetControlVector();
    } else if (CheckArm(partID)) {
        return (armsModel_.at(partID)->GetControlVector());
    }

    //TODO EXCEPTION
}

Eigen::TransfMatrix RobotModel::GetTransformation(const std::string& frameID)
{

    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;

    if (partID == vehicleID_ || frameID == vehicleID_) {
        return vehicle_->GetTransformation(frameID);
    }
    else if (CheckArm(partID)) {
        //check arm
        T = armsModel_.at(partID)->GetTransformation(frameID);
        if (vehicle_) {
            T = vehicle_->GetTransformation(vehicleID_) * vehicleToBase_.at(partID) * T;
        } else {
            //if there is no vehicle, the inertial frame wrt all is expressed is the vehicle to base.
            T = vehicleToBase_.at(partID) * T;
        }
    }

    return T;
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

Eigen::MatrixXd RobotModel::GetCartesianJacobian(const std::string& frameID, JacobianObserver jacobianObserver)
{

    std::string modelID = frameID.substr(0, frameID.find_first_of("_"));
    Eigen::MatrixXd totJac, tempJ;

    if ((frameID == vehicleID_ || modelID == vehicleID_)) {
        if (jacobianObserver == InertialFrame) {
            totJac = RightJuxtapose(totJac, vehicle_->GetJacobian(frameID));
        } else if (jacobianObserver == VehicleFrame) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(6, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            tempJ = Eigen::MatrixXd::Zero(6, iter->second->GetNumJoints());
            totJac = RightJuxtapose(totJac, tempJ);
        }
    }

    else if (CheckArm(modelID)) {
        if (vehicle_) {
            if (jacobianObserver == InertialFrame) {
                totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForFrame(frameID));

            } else if (jacobianObserver == VehicleFrame) {

                totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(6, 6));
            }
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
    }
    return totJac;
}

Eigen::MatrixXd RobotModel::GetJointSpaceJacobian(const std::string& armID)

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
    }
    return totJac;
}
Eigen::MatrixXd RobotModel::GetManipulabilityJacobian(const std::string& frameID)
{

    Eigen::MatrixXd totJac, tempJ;
    std::string armID= frameID.substr(0,frameID.find_first_of("_"));
    if (CheckArm(armID)) {
        if (vehicle_) {

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
    }

    return totJac;
}

const std::shared_ptr<ArmModel> RobotModel::GetArm(std::string ID) const
{
    CheckArm(ID);

    return armsModel_.at(ID);
}

const std::shared_ptr<VehicleModel> RobotModel::GetVehicle() const
{
    CheckVehicle();
    return vehicle_;
}
}

/* namespace rml */
