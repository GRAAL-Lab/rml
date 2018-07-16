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
    if (vehicle->IsModelInitialized()) {
        if (vehicle_) {
            throw(RobotModelVehicleModelAlreadyLoadedException());
        } else {
            vehicle_ = vehicle;
            vehicleID_ = vehicle->GetID();
            return true;
        }
    } else {
        throw(RobotModelNotInitializedVehicleModelException());
        return false;
    }
}

bool RobotModel::LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb) throw(std::exception)
{
    if (arm->IsModelInitialized()) {
        if (CheckArm(arm->GetID())) {
            RobotModelConflictingArmModelIDException conflictingArmModelException;
            conflictingArmModelException.SetWho(arm->GetID());
            throw(conflictingArmModelException);
            return false;
        }
        armsModel_.insert(std::make_pair(arm->GetID(), arm));
        vehicleToBase_.insert(std::make_pair(arm->GetID(), vTb));
        return (true);

    } else {
        throw(RobotModelNotInitializedArmModelException());
        return false;
    }
}

bool RobotModel::CheckArm(const std::string armID) const
{
    // if give exception cannot be used for actually checkind if the robot is present.

    if (armsModel_.find(armID) != armsModel_.end()) {
        return true;
    } else {
        //RobotModelArmException robotModelArmException;
        //robotModelArmException.SetWho(armID);
        //throw(robotModelArmException);
        return false;
    }
}

bool RobotModel::CheckVehicle() const
{
    if (vehicle_) {
        return true;
    } else {
        //throw RobotModelVehicleException();
        return false;
    }
}

//TODO throw exception in frame one
Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForFrame(const std::string& frameID) const throw(std::exception)
{
    Eigen::MatrixXd bJt;
    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);

    if (CheckArm(partID)) {
        bJt = armsModel_.at(partID)->GetJacobian(frameID);
        Eigen::RotMatrix vRb = vehicleToBase_.at(partID).GetRotMatrix();
        bJt = vRb.GetCartesianRotationMatrix() * bJt;
        return bJt;
    } else {
        throw(RobotModelArmException());
    }
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

void RobotModel::SetRobotControl(const Eigen::VectorXd& y) throw(std::exception)
{
    int startIndex = 0;
    if (y.size() == GetTotalDOFs()) {
        if (vehicle_) {
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

Eigen::VectorXd RobotModel::GetRobotControl(std::string partID) throw(std::exception)
{
    Eigen::VectorXd y;

    if (partID == vehicleID_ && vehicle_) {
        return vehicle_->GetControlVector();
    } else if (CheckArm(partID)) {
        return (armsModel_.at(partID)->GetControlVector());
    } else {
        RobotModelNotExistingPartException robotNotExistingPartException;
        robotNotExistingPartException.SetWhere("GetRobotControl");
        robotNotExistingPartException.SetWho(partID);
        throw(robotNotExistingPartException);
    }
}

Eigen::TransfMatrix RobotModel::GetTransformation(const std::string& frameID) throw(std::exception)
{

    std::size_t partIDIndex = frameID.find_first_of("_");
    std::string partID = frameID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;
    if(partIDIndex==std::string::npos && frameID!=vehicleID_)
    {
        RobotModelWrongFrameFormat robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetWhere("GetTransformation");
        robotModelWrongFrameFormat.SetWho(frameID);
        throw (robotModelWrongFrameFormat);
    }


    if (partID == vehicleID_ || frameID == vehicleID_) {
        return vehicle_->GetTransformation(frameID);
    } else if (CheckArm(partID)) {
        //check arm
        T = armsModel_.at(partID)->GetTransformation(frameID);
        if (vehicle_) {
            T = vehicle_->GetTransformation(vehicleID_) * vehicleToBase_.at(partID) * T;
        } else {
            //if there is no vehicle, the inertial frame wrt all is expressed is the vehicle to base.
            T = vehicleToBase_.at(partID) * T;
        }
        return T;
    }

    RobotModelNotExistingPartException robotModelNotExistingPartExc;
    robotModelNotExistingPartExc.SetWho(partID);
    robotModelNotExistingPartExc.SetWhere("GetTransformation");
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

Eigen::MatrixXd RobotModel::GetCartesianJacobian(const std::string& frameID, JacobianObserver jacobianObserver) throw(std::exception)
{

    std::string modelID = frameID.substr(0, frameID.find_first_of("_"));
    Eigen::MatrixXd totJac, tempJ;
    if(frameID.find_first_of("_")==std::string::npos && frameID!=vehicleID_ )
    {
        RobotModelWrongFrameFormat robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetWhere("GetCartesianJacobian");
        robotModelWrongFrameFormat.SetWho(frameID);
        throw(robotModelWrongFrameFormat);
    }

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
         return totJac;
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
        return totJac;
    }
    RobotModelNotExistingPartException robotModelNotExistingPartException;
    robotModelNotExistingPartException.SetWhere("GetCartesianJacobian");
    robotModelNotExistingPartException.SetWho(modelID);
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
    RobotModelNotExistingPartException notExistingPartException;
    notExistingPartException.SetWhere("GetJointSpaceJacobian");
    notExistingPartException.SetWho(armID);
    throw(notExistingPartException);
}
Eigen::MatrixXd RobotModel::GetManipulabilityJacobian(const std::string& frameID) throw(std::exception)
{

    Eigen::MatrixXd totJac, tempJ;
    std::string armID = frameID.substr(0, frameID.find_first_of("_"));
    if(frameID.find_first_of("_")==std::string::npos)
    {
        RobotModelWrongFrameFormat robotModelWrongFrameFormat;
        robotModelWrongFrameFormat.SetWhere("GetCartesianJacobian");
        robotModelWrongFrameFormat.SetWho(frameID);
    }
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
        return totJac;
    }
    RobotModelNotExistingPartException notExistingPartExc;
    notExistingPartExc.SetWhere("GetManipulabilityJacobian");
    notExistingPartExc.SetWho(armID);
    throw(notExistingPartExc);
}

const std::shared_ptr<ArmModel> RobotModel::GetArm(std::string ID) const throw(std::exception)
{
    if (CheckArm(ID)) {

        return armsModel_.at(ID);
    }
    RobotModelNotExistingPartException notExistingPartExc;
    notExistingPartExc.SetWhere("GetArm");
    notExistingPartExc.SetWho(ID);
}

const std::shared_ptr<VehicleModel> RobotModel::GetVehicle() const throw(std::exception)
{
    if (CheckVehicle()) {
        return vehicle_;
    }
    RobotModelNotExistingPartException notExistingPartExc;
    notExistingPartExc.SetWhere("GetVehicle");
    notExistingPartExc.SetWho("Vehicle");
}
}

/* namespace rml */
