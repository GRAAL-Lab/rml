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
    jacobianMethodsMap_.insert(std::make_pair("Frame", 1));
    jacobianMethodsMap_.insert(std::make_pair("Identity", 2));
    jacobianMethodsMap_.insert(std::make_pair("Manipulability", 3));
    jacobianMethodsMap_.insert(std::make_pair("Vehicle", 4));
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

bool RobotModel::LoadVehicle(const std::shared_ptr<VehicleModel> vehicle)
{
    if (vehicle->IsModelInitialized()) {
        vehicle_ = vehicle;
        return true;
    } else {
        std::cout << tc::redL << "Error: Loaded a NOT initialised VehicleModel" << tc::none << std::endl;
        return false;
    }
}

bool RobotModel::LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb)
{
    if (arm->IsModelInitialized()) {
        armsModel_.insert(std::make_pair(arm->GetID(), arm));
        vehicleToBase_.insert(std::make_pair(arm->GetID(), vTb));
        return (true);
    } else {
        std::cout << tc::redL << "Error: Loaded a NOT initialised ArmModel" << tc::none << std::endl;
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
    //TODO
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
    if (partID == vehicle_->GetID()) {
        return vehicle_->GetTransfMatrix(transformationID);
    } else if (CheckArm(partID)) {
        //check arm
        Eigen::TransfMatrix T = armsModel_.at(partID)->GetTransformationMatrix(transformationID);
        if (vehicle_) {
            return vehicle_->GetwTv() * vehicleToBase_.at(partID) * T;
        } else
            return T;
    }
}

Eigen::MatrixXd RobotModel::GetJacobian(std::string ID)
{
    /**
     * POLICY FOR JACOBIANS (all projected on the vehicle and measured wrt to the inertial frame )
     * "Frame" + "_" + vehicleID + "_" + arm ID + "_" + "Joint" + "_" + N° Joint : vJj projected on the vehicle .
     * "Frame" + "_" + arm ID + "_" + "Joint" + "_" + N° Joint : Jacobian joint frame (includes vehicle contribution) projected on the vehicle .
     * "Frame" + "_" + vehicleID + "_" + arm ID + "_" + "Tool": vJt projected on the vehicle .
     * "Frame" + "_" + arm ID + "_" + "Tool" :  Jacobian joint frame (includes vehicle contribution) projected on the vehicle.
     * "Identity" + "_" + arm ID + "_" :  Identity Jacobian .
     * "Frame" + "_" + arm ID + "_" + "Body" + "_" + Frame ID : Jacobian Rigid Body attached to the arm.
     * "Vehicle" + "_" + vehicleID + "_" + "Body" + "_" + Frame ID : Jacobian Rigid Body attached to the vehicle i.e. no arm contribution.
     * "Vehicle"+"_"+vehicle ID : Vehicle Jacobian projected on the vehicle
     *
     *
     */
    std::string partID = ID.substr(0, ID.find_first_of("_"));
    std::string temp = ID.substr(ID.find_first_of("_") + 1);

    Eigen::MatrixXd out;

    switch (jacobianMethodsMap_.at(partID)) {
    case 1:
        out = GetJacobian_Frame(temp);
        break;
    case 2:
        out = GetJacobian_Identity(temp);
        break;
    case 3:
        out = GetJacobian_Manipulability(temp);
        break;
    case 4:
        out = GetJacobian_Vehicle(temp);
        break;
    default:
        std::cout << "WRONG LABEL !!" << std::endl;
        break;
    }
    return out;
}

Eigen::MatrixXd RobotModel::GetJacobian_Frame(std::string ID)
{

    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    Eigen::MatrixXd totJac, tempJ;
    if (partID == vehicle_->GetID() && vehicle_) {
        totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForFrame(ID.substr(partIDIndex + 1)));
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
             ++iter) {
            tempJ = Eigen::MatrixXd::Zero(6, iter->second->GetNumJoints());
            totJac = RightJuxtapose(totJac, tempJ);
        }
    } else if (CheckArm(partID)) {

        if (vehicle_) {
            totJac = RightJuxtapose(totJac, GetIsolatedVehicleJacobianForFrame(ID));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = armsModel_.begin(); iter != armsModel_.end();
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

                double mu;
                armsModel_.at(partID)->EvaluateManipulability(tempJ, mu);
                armsModel_.at(partID)->SetManipulability(mu);

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
        return totJac;
    }
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
