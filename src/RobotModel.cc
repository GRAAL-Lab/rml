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
    jacobianMethodsMap_.insert(std::make_pair("Frame", JacobianType::Arm));
    //jacobianMethodsMap_.insert(std::make_pair("Identity", 2));
    //jacobianMethodsMap_.insert(std::make_pair("Manipulability", 3));
    jacobianMethodsMap_.insert(std::make_pair("Vehicle", JacobianType::Vehicle));
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
        vehicle_ = vehicle;
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

bool RobotModel::CheckArm(std::string armID) const throw(std::exception)
{
    if (armsModel_.find(armID) != armsModel_.end()) {
        return true;
    } else {
        RobotModelArmException robotModelArmException;
        robotModelArmException.SetID(armID);
        throw(robotModelArmException);
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

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForFrame(std::string ID) const
{
    Eigen::MatrixXd bJt;
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);

    if (CheckArm(partID)) {
        bJt = armsModel_.at(partID)->GetJacobian(ID);
        if (vehicle_) {
            Eigen::RotMatrix vRb = vehicleToBase_.at(partID).GetRotMatrix();
            bJt = vRb.GetCartesianRotationMatrix() * bJt;
        }
    }
    return bJt;
}

Eigen::Matrix6d RobotModel::GetIsolatedVehicleJacobianForFrame(std::string ID) const
{
    Eigen::Matrix6d vJv;
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    std::cout << "GetIsolatedVehicleJacobianForFrame::ID=" << ID << std::endl;
    std::cout << "GetIsolatedVehicleJacobianForFrame::partID=" << partID << std::endl;

    if (CheckVehicle()) {
        if (CheckArm(partID)) {
            vJv = vehicle_->GetvJv();
            Eigen::TransfMatrix bTj = armsModel_.at(partID)->GetTransformationMatrix(ID);
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
    if (partID == vehicle_->GetID()) {
        return vehicle_->GetControlVector();
    } else if (CheckArm(partID)) {
        return (armsModel_.at(partID)->GetControlVector());
    }
}

Eigen::TransfMatrix RobotModel::GetTransformation(std::string transformationID)
{

    std::size_t partIDIndex = transformationID.find_first_of("_");
    std::string partID = transformationID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;

    if(vehicle_){
    if (partID == vehicle_->GetID() || transformationID == vehicle_->GetID()) {
        return vehicle_->GetTransfMatrix(transformationID);
    }
    }else if (CheckArm(partID)) {
        //check arm
        T = armsModel_.at(partID)->GetTransformationMatrix(transformationID);
        if (vehicle_) {
            T = vehicle_->GetwTv() * vehicleToBase_.at(partID) * T;
        } else {
            //if there is no vehicle, the inertial frame wrt all is expressed is the vehicle to base.
            T = vehicleToBase_.at(partID) * T;
        }
    }

    return T;
}

Eigen::MatrixXd RobotModel::GetJacobian_Frame(std::string frameID, JacobianObserver jacobianObserver)
{

    std::string modelID = frameID.substr(0, frameID.find_first_of("_"));

    Eigen::MatrixXd totJac, tempJ;
    if (vehicle_) {
        if ((frameID == vehicle_->GetID() || modelID == vehicle_->GetID()) && vehicle_) {
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
    } else if (CheckArm(modelID)) {

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

Eigen::MatrixXd RobotModel::GetJacobian_JointSpace(std::string armID)

{

    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm(armID)) {
        int taskSize = armsModel_.at(armID)->GetNumJoints();
        std::cout << "taskSize " << taskSize << std::endl;
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
Eigen::MatrixXd RobotModel::GetJacobian_Manipulability(std::string armID)
{

    Eigen::MatrixXd totJac, tempJ;

    if (CheckArm(armID)) {
        if (vehicle_) {

            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(1, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {

            if (iter->first == armID) {
                tempJ = iter->second->GetJacobian(armID + FrameID::Manipulability);

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
