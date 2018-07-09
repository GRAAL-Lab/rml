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
        throw RobotModelArmException();
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

Eigen::VectorXd RobotModel::ExtractVehicleSlice(const Eigen::VectorXd& y) const
{
    Eigen::VectorXd slice;
    if (CheckVehicle()) {
        slice = y.block(0, 0, 6, 1);
    }
    return slice;
}

Eigen::VectorXd RobotModel::ExtractArmSlice(const Eigen::VectorXd& y, std::string armID)
{
    int startIndex(0);
    if (vehicle_)
        startIndex = 6;

    if (CheckArm(armID)) {
        std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin();
        while (iter->first != armID) {
            startIndex = startIndex + iter->second->GetNumJoints();
            iter++;
        }
    }
    return y.block(startIndex, 0, armsModel_.at(armID)->GetNumJoints(), 1);
}

void RobotModel::SetRobotControl(const Eigen::VectorXd& y)
{
    if (vehicle_) {
        vehicle_->SetControlVector(ExtractVehicleSlice(y));
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        iter->second->SetControlVector(ExtractArmSlice(y, iter->first));
    }
}

Eigen::VectorXd RobotModel::GetRobotControl()
{
    Eigen::VectorXd y;
    if (vehicle_) {
        y = UnderJuxtapose(y, vehicle_->GetControlVector());
    }
    for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
         ++iter) {
        y = UnderJuxtapose(y, iter->second->GetControlVector());
    }
    return y;
}

Eigen::TransfMatrix RobotModel::GetTransformation(std::string transformationID)
{

    std::size_t partIDIndex = transformationID.find_first_of("_");
    std::string partID = transformationID.substr(0, partIDIndex);
    Eigen::TransfMatrix T;

    if (partID == vehicle_->GetID()) {
        return vehicle_->GetTransfMatrix(transformationID);
    } else if (CheckArm(partID)) {
        //check arm
        T = armsModel_.at(partID)->GetTransformationMatrix(transformationID);
        if (vehicle_) {
            T = vehicle_->GetwTv() * vehicleToBase_.at(partID) * T;
        }
    }

    return T;
}

Eigen::MatrixXd RobotModel::GetJacobian(std::string ID) throw(std::exception)
{
    /**
     * POLICY FOR JACOBIANS (all projected on the vehicle and measured wrt to the inertial frame )
     * "Frame"    + "_" + vehicleID + "_" + arm ID + "_" + "Joint" + "_" + N° Joint : vJj projected on the vehicle .
     * "Frame"    + "_" + arm ID + "_" + "Joint" + "_" + N° Joint : Jacobian joint frame (includes vehicle contribution) projected on the vehicle .
     * "Frame"    + "_" + vehicleID + "_" + arm ID + "_" + "Tool": vJt projected on the vehicle .
     * "Frame"    + "_" + arm ID + "_" + "Tool" :  Jacobian joint frame (includes vehicle contribution) projected on the vehicle.
     * "Identity" + "_" + arm ID + "_" :  Identity Jacobian .
     * "Frame"    + "_" + arm ID + "_" + "Body" + "_" + Frame ID : Jacobian Rigid Body attached to the arm.
     * "Vehicle"  + "_" + vehicleID + "_" + "Body" + "_" + Frame ID : Jacobian Rigid Body attached to the vehicle i.e. no arm contribution.
     * "Vehicle"  + "_" + vehicle ID : Vehicle Jacobian projected on the vehicle
     */


    std::string partID = ID.substr(0, ID.find_first_of("_"));
    std::string temp = ID.substr(ID.find_first_of("_") + 1);
    std::cout << "ID=" << ID << std::endl;
    std::cout << "partID=" << partID << std::endl;
    std::cout << "temp=" << temp << std::endl;

    Eigen::MatrixXd out;

    if (jacobianMethodsMap_.find(partID) != jacobianMethodsMap_.end()) {
        JacobianType ret = jacobianMethodsMap_.at(partID);
        switch (ret) {
        case JacobianType::Arm:
            out = GetJacobian_Frame(temp);
            break;
        case JacobianType::Vehicle:
            out = GetJacobian_Vehicle(temp);
            break;
        }

    } else {
        throw RobotModelWrongLabelException();
    }

    return out;
}

Eigen::MatrixXd RobotModel::GetJacobian_Frame(std::string ID)
{

    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    std::string temp = ID.substr(partIDIndex + 1);
    std::cout << "GetJacobian_Frame::partID=" << partID << std::endl;
    std::cout << "GetJacobian_Frame::temp=" << temp << std::endl;


    Eigen::MatrixXd totJac, tempJ;
    if (partID == vehicle_->GetID() && vehicle_) {
        totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForFrame(temp));
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            tempJ = Eigen::MatrixXd::Zero(6, iter->second->GetNumJoints());
            totJac = RightJuxtapose(totJac, tempJ);
        }
    } else if (CheckArm(partID)) {

        if (vehicle_) {
            totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForFrame(ID));
        }
        for (auto iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == partID) {
                tempJ = GetIsolatedArmJacobianForFrame(ID);
            } else {
                tempJ = Eigen::MatrixXd::Zero(6, armsModel_.at(partID)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
    }
    return totJac;
}

Eigen::MatrixXd RobotModel::GetJacobian_Identity(std::string ID)

{
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    Eigen::MatrixXd totJac, tempJ;

    if (CheckArm(partID)) {
        int taskSize = armsModel_.at(partID)->GetNumJoints();
        if (vehicle_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(taskSize, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == partID) {
                tempJ = Eigen::MatrixXd::Identity(taskSize, taskSize);
            } else {
                tempJ = Eigen::MatrixXd::Zero(taskSize, armsModel_.at(partID)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
    }
    return totJac;
}

Eigen::MatrixXd RobotModel::GetJacobian_Manipulability(std::string ID)
{
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    Eigen::MatrixXd totJac, tempJ;

    if (CheckArm(partID)) {
        if (vehicle_) {

            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(1, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            if (iter->first == partID) {

                armsModel_.at(partID)->EvaluateManipulability(tempJ);

            } else {
                tempJ = Eigen::MatrixXd::Zero(1, armsModel_.at(iter->first)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }
    }

    return totJac;
}

Eigen::MatrixXd RobotModel::GetJacobian_Vehicle(std::string ID)
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckVehicle()) {
        if (CheckVehicle()) {
            totJac = RightJuxtapose(totJac, vehicle_->GetJacobian(ID));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            tempJ = Eigen::MatrixXd::Zero(6, iter->second->GetNumJoints());
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
// //////////////////       Public      ////////////////// //

//TODO use GetJacobian
/*
Eigen::MatrixXd RobotModel::GetArmJacobian_JointFrame(int armIndex, int jointIndex)
{
    Eigen::MatrixXd totJac, tempJ;
    if (vehicle_) {
        totJac = RightJuxtapose(totJac, Eigen::Matrix6d::Zero());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        if (i == armIndex)
            tempJ = GetIsolatedArmJacobianForJoint(i, jointIndex);
        else
            tempJ = Eigen::MatrixXd::Zero(6, arms_.at(armIndex)->GetNumJoints());
        totJac = RightJuxtapose(totJac, tempJ);
    }
    return totJac;
}
*/
/*
Eigen::MatrixXd RobotModel::GetVehicleJacobian_JointFrame(int armIndex, int jointIndex)
{
    Eigen::MatrixXd totJac, tempJ;
    if (vehicle_) {
        totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForJoint(armIndex, jointIndex));
    }
    for (int i = 0; i < arms_.size(); ++i) {
        tempJ = Eigen::MatrixXd::Zero(6, arms_.at(armIndex)->GetNumJoints());
        totJac = RightJuxtapose(totJac, tempJ);
    }
    return totJac;
}
*/
/*
Eigen::MatrixXd RobotModel::GetArmJacobian_ToolFrame(int armIndex)
{
    Eigen::MatrixXd totJac, tempJ;
    if (vehicle_) {
        totJac = RightJuxtapose(totJac, Eigen::Matrix6d::Zero());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        if (i == armIndex)
            tempJ = GetIsolatedArmJacobianTF();
        else
            tempJ = Eigen::MatrixXd::Zero(6, arms_.at(armIndex)->GetNumJoints());
        totJac = RightJuxtapose(totJac, tempJ);
    }
    return totJac;
}
*/
/*
Eigen::MatrixXd RobotModel::GetVehicleJacobian_ToolFrame(int armIndex)
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckVehicle()) {
        totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianEE(armIndex));
    }
    for (int i = 0; i < arms_.size(); ++i) {
        tempJ = Eigen::MatrixXd::Zero(6, arms_.at(armIndex)->GetNumJoints());
        totJac = RightJuxtapose(totJac, tempJ);
    }
    return totJac;
}
*/
/*
Eigen::MatrixXd RobotModel::GetArmJacobian_Identity(int armIndex)
{
    //TODO
    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm("TODO")) {
        int taskSize = arms_.at(armIndex)->GetNumJoints();
        if (vehicle_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(taskSize, 6));
        }
        for (int i = 0; i < arms_.size(); ++i) {
            if (i == armIndex)
                tempJ = Eigen::MatrixXd::Identity(taskSize, taskSize);
            else
                tempJ = Eigen::MatrixXd::Zero(taskSize, arms_.at(i)->GetNumJoints());
            totJac = RightJuxtapose(totJac, tempJ);
        }
    }
    return totJac;
}
*/
/*
Eigen::MatrixXd RobotModel::GetArmJacobian_Manipulability(int armIndex, double& mu)
{
    Eigen::MatrixXd totJac, tempJ;
    //TODO
    if (CheckArm("TODO")) {
        if (vehicle_) {
            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(1, 6));
        }
        for (int i = 0; i < arms_.size(); ++i) {
            if (i == armIndex)
                arms_.at(i)->EvaluateManipulability(tempJ, mu);
            else
                tempJ = Eigen::MatrixXd::Zero(1, arms_.at(i)->GetNumJoints());
            totJac = RightJuxtapose(totJac, tempJ);
        }
    }
    return totJac;
}
*/
/*
Eigen::MatrixXd RobotModel::GetVehicleJacobian()
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckVehicle()) {
        totJac = RightJuxtapose(totJac, vehicle_->GetvJv());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        tempJ = Eigen::MatrixXd::Zero(6, arms_.at(i)->GetNumJoints());
        totJac = RightJuxtapose(totJac, tempJ);
    }
    return totJac;
}
*/
/*
Eigen::TransfMatrix RobotModel::GetTransfMatrix_VehicleToArmBase(int armIndex) const
{
    Eigen::TransfMatrix vTb;
    //TODO
    if (CheckVehicle() && CheckArm("TODO")) {
        vTb = vehicleTbase_.at(armIndex);
    }
    return vTb;
}
*/
/*
Eigen::TransfMatrix RobotModel::GetTransfMatrix_JointFrame(int armIndex, int jointIndex) const
{
    Eigen::TransfMatrix wTj;
    if (vehicle_) {
        wTj = vehicle_->GetwTv();
    }
    //TODO
    if (CheckArm("TODO")) {
        wTj = wTj * vehicleTbase_.at(armIndex) * arms_.at(armIndex)->GetBase2JointTransf(jointIndex);
    }
    return wTj;
}
*/
/*
Eigen::TransfMatrix RobotModel::GetTransfMatrix_ToolFrame(int armIndex) const
{
    Eigen::TransfMatrix wTt;
    if (vehicle_) {
        wTt = vehicle_->GetwTv();
    }
    //TODO
    if (CheckArm("TODO")) {
        wTt = wTt * vehicleTbase_.at(armIndex) * arms_.at(armIndex)->GetBase2ToolTransf();
    }
    return wTt;
}
*/
