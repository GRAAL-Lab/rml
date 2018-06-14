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
    std::cout << "inside the constructor" << std::endl;
    jacobianMethodsMap_.insert(std::make_pair("Joint", GetJacobian_JointFrame));
    jacobianMethodsMap_.insert(std::make_pair("Tool", GetJacobian_ToolFrame));
    jacobianMethodsMap_.insert(std::make_pair("Identity", GetJacobian_Identity));
    jacobianMethodsMap_.insert(std::make_pair("Manipulability", GetJacobian_Manipulability));

    // TODO Auto-generated constructor stub
}

RobotModel::~RobotModel()
{
    // TODO Auto-generated destructor stub
    /*
     * ArmID_Joint_5;
    Eigen::MatrixXd RobotModel::GetArmJacobian_JointFrame(std::string ArmID)
    {

    }
    VehicleID_Joint_5
    Eigen::MatrixXd RobotModel::GetVehicleJacobian_JointFrame(std::string ArmID)
    {

    }
    ArmID_
    Eigen::MatrixXd GetArmJacobian_ToolFrame(std::string ArmID){}
    VehicleID_Tool
Eigen::MatrixXd GetVehicleJacobian_ToolFrame(std::string ArmID){}
    ArmID_identity
Eigen::MatrixXd GetArmJacobian_Identity(std::string ArmID){}
*/
    //std::map<std::string, Eigen::MatrixXd (*)()> jacobianMethodsMap_;
}

int RobotModel::GetTotalDOFs()
{
    int totDOFs(0);
    if (vehicle_) {
        totDOFs += 6;
    }
    for (int i = 0; i < arms_.size(); ++i) {
        totDOFs += arms_.at(i)->GetNumJoints();
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

int RobotModel::LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb)
{
    if (arm->IsModelInitialized()) {
        arms_.push_back(arm);
        //		JArm_.push_back(Eigen::MatrixXd(6, arm.GetNumJoints()));
        //		JVeh_.push_back(Eigen::Matrix6d());
        vehicleTbase_.push_back(vTb);
        armsModel_.insert(std::make_pair(arm->GetID(), arm));
        vehicleToBase_.insert(std::make_pair(arm->GetID(), vTb));
        return (arms_.size() - 1);
    } else {
        std::cout << tc::redL << "Error: Loaded a NOT initialised ArmModel" << tc::none << std::endl;
        return -1;
    }
}

bool RobotModel::CheckArm(int armIndex) const throw(std::exception)
{
    if (armIndex < arms_.size()) {
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

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianTF(int armIndex) const
{
    Eigen::MatrixXd bJt;
    if (CheckArm(armIndex)) {
        std::cout << "inside isolated arm jacobian"  << std::endl;
        /// The robot model actually returns the jacobian of the end-effector  w.r.t. the base of the robot
        bJt = arms_.at(armIndex)->GetBase2ToolJacobian();
        if (vehicle_) {
            Eigen::RotMatrix vRb = vehicleTbase_.at(armIndex).GetRotMatrix();
            bJt = vRb.GetCartesianRotationMatrix() * bJt;
        }
    }
    return bJt;
}

Eigen::Matrix6d RobotModel::GetIsolatedVehicleJacobianEE(int armIndex) const
{
    Eigen::Matrix6d vJv;
    if (CheckVehicle()) {
        if (CheckArm(armIndex)) {
            vJv = vehicle_->GetvJv();
            Eigen::TransfMatrix bTt = arms_.at(armIndex)->GetBase2ToolTransf();
            Eigen::TransfMatrix vTt = vehicleTbase_.at(armIndex) * bTt;
            vJv = GetRigidBodyMatrix(vTt.GetTransl()) * vJv;
        }
    }
    return vJv;
}

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianForJoint(int armIndex, int jointIndex) const
{
    Eigen::MatrixXd bJj;
    if (CheckArm(armIndex)) {
        /// The robot model actually returns the jacobian of the end-effector  w.r.t. the base of the robot
        /// The robot model actually returns the transformation of the end-effector  w.r.t. the base of the robot
        bJj = arms_.at(armIndex)->EvaluateBase2JointJacobian(jointIndex);
        if (vehicle_) {
            Eigen::RotMatrix vRb = vehicleTbase_.at(armIndex).GetRotMatrix();
            bJj = vRb.GetCartesianRotationMatrix() * bJj;
        }
    }
    return bJj;
}

Eigen::Matrix6d RobotModel::GetIsolatedVehicleJacobianForJoint(int armIndex, int jointIndex) const
{
    Eigen::Matrix6d vJv;
    if (CheckVehicle()) {
        if (CheckArm(armIndex)) {
            vJv = vehicle_->GetvJv();
            Eigen::TransfMatrix bTj = arms_.at(armIndex)->GetBase2JointTransf(jointIndex);
            Eigen::TransfMatrix vTj = vehicleTbase_.at(armIndex) * bTj;
            vJv = GetRigidBodyMatrix(vTj.GetTransl()) * vJv;
        }
    }
    return vJv;
}

// //////////////////       Public      ////////////////// //

//TODO use GetJacobian
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

Eigen::MatrixXd RobotModel::GetArmJacobian_ToolFrame(int armIndex)
{
    Eigen::MatrixXd totJac, tempJ;
    if (vehicle_) {
        totJac = RightJuxtapose(totJac, Eigen::Matrix6d::Zero());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        if (i == armIndex)
            tempJ = GetIsolatedArmJacobianTF(i);
        else
            tempJ = Eigen::MatrixXd::Zero(6, arms_.at(armIndex)->GetNumJoints());
        totJac = RightJuxtapose(totJac, tempJ);
    }
    return totJac;
}

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

Eigen::MatrixXd RobotModel::GetArmJacobian_Identity(int armIndex)
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm(armIndex)) {
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

Eigen::MatrixXd RobotModel::GetArmJacobian_Manipulability(int armIndex, double& mu)
{
    Eigen::MatrixXd totJac, tempJ;
    if (CheckArm(armIndex)) {
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

Eigen::TransfMatrix RobotModel::GetTransfMatrix_VehicleToArmBase(int armIndex) const
{
    Eigen::TransfMatrix vTb;
    if (CheckVehicle() && CheckArm(armIndex)) {
        vTb = vehicleTbase_.at(armIndex);
    }
    return vTb;
}

Eigen::TransfMatrix RobotModel::GetTransfMatrix_JointFrame(int armIndex, int jointIndex) const
{
    Eigen::TransfMatrix wTj;
    if (vehicle_) {
        wTj = vehicle_->GetwTv();
    }
    if (CheckArm(armIndex)) {
        wTj = wTj * vehicleTbase_.at(armIndex) * arms_.at(armIndex)->GetBase2JointTransf(jointIndex);
    }
    return wTj;
}

Eigen::TransfMatrix RobotModel::GetTransfMatrix_ToolFrame(int armIndex) const
{
    Eigen::TransfMatrix wTt;
    if (vehicle_) {
        wTt = vehicle_->GetwTv();
    }
    if (CheckArm(armIndex)) {
        wTt = wTt * vehicleTbase_.at(armIndex) * arms_.at(armIndex)->GetBase2ToolTransf();
    }
    return wTt;
}

Eigen::VectorXd RobotModel::GetSystemPositionVector() const
{
    Eigen::VectorXd pos;
    if (vehicle_) {
        pos = UnderJuxtapose(pos, vehicle_->GetPositionOnInertial());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        pos = UnderJuxtapose(pos, arms_.at(i)->GetJointsPosition());
    }
    return pos;
}

Eigen::VectorXd RobotModel::GetSystemVelocityVector() const
{
    Eigen::VectorXd vel;
    if (vehicle_) {
        vel = UnderJuxtapose(vel, vehicle_->GetVelocityOnVehicle());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        vel = UnderJuxtapose(vel, arms_.at(i)->GetJointsVelocity());
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

Eigen::VectorXd RobotModel::ExtractArmSlice(const Eigen::VectorXd& y, int armIndex) const
{
    int startIndex(0);
    if (vehicle_)
        startIndex = 6;
    if (CheckArm(armIndex)) {
        for (int i = 0; i < armIndex; ++i) {
            startIndex = startIndex + arms_.at(i)->GetNumJoints();
        }
    }
    return y.block(startIndex, 0, arms_.at(armIndex)->GetNumJoints(), 1);
}

void RobotModel::SetRobotControl(const Eigen::VectorXd& y) const
{
    if (vehicle_) {
        vehicle_->SetControlVector(ExtractVehicleSlice(y));
    }
    for (int i = 0; i < arms_.size(); ++i) {
        arms_.at(i)->SetControlVector(ExtractArmSlice(y, i));
    }
}

Eigen::VectorXd RobotModel::GetRobotControl() const
{
    Eigen::VectorXd y;
    if (vehicle_) {
        y = UnderJuxtapose(y, vehicle_->GetControlVector());
    }
    for (int i = 0; i < arms_.size(); ++i) {
        y = UnderJuxtapose(y, arms_.at(i)->GetControlVector());
    }
    return y;
}

Eigen::TransfMatrix RobotModel::GetTransformation(std::string transformationID)
{

    std::size_t partIDIndex = transformationID.find_first_of("_");
    std::string partID = transformationID.substr(0, partIDIndex);
    if (partID == vehicle_->GetID()) {
        return vehicle_->GetTransfMatrix(transformationID);
    } else {
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
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    std::string temp = ID.substr(partIDIndex + 1, ID.size());
    std::size_t tempIndex = temp.find_first_of("_");
    std::string jacobianID = temp.substr(0, tempIndex);
    return jacobianMethodsMap_.at(jacobianID)(ID, *this);
}

Eigen::MatrixXd RobotModel::GetJacobian_JointFrame(std::string ID, rml::RobotModel robot)
{
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    Eigen::MatrixXd totJac, tempJ;
    if (partID == robot.vehicle_->GetID()) {
                //TODO because it depends also on the arm.
    } else {




        if (robot.vehicle_) {
            totJac = RightJuxtapose(totJac, Eigen::Matrix6d::Zero());
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = robot.armsModel_.begin(); iter != robot.armsModel_.end();
             ++iter) {
            if(iter->first == partID)
            {   //todo instead of i string;
                int i = 0 ;
                tempJ = robot.GetIsolatedArmJacobianForJoint(i,3);
            }
            else
            {
                tempJ = Eigen::MatrixXd::Zero(6, robot.armsModel_.at(partID)->GetNumJoints());
            }
            totJac = RightJuxtapose(totJac, tempJ);
        }

    }
    return totJac;

}

Eigen::MatrixXd RobotModel::GetJacobian_ToolFrame(std::string ID, rml::RobotModel robot)
{
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    Eigen::MatrixXd totJac, tempJ;
    if (partID == robot.vehicle_->GetID()) {
                //TODO because it depends also on the arm.
    } else {
        if (robot.vehicle_) {

            totJac = RightJuxtapose(totJac, Eigen::Matrix6d::Zero());
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = robot.armsModel_.begin(); iter != robot.armsModel_.end();
             ++iter) {
            if(iter->first == partID)
            {   int i =0;
                 tempJ = robot.GetIsolatedArmJacobianTF(i);
            }
            else
            {
                tempJ = Eigen::MatrixXd::Zero(6, robot.armsModel_.at(partID)->GetNumJoints());

            }
            totJac = RightJuxtapose(totJac, tempJ);
        }

    }
    return totJac;
}

Eigen::MatrixXd RobotModel::GetJacobian_Identity(std::string ID, rml::RobotModel robot)

{
    std::size_t partIDIndex = ID.find_first_of("_");
    std::string partID = ID.substr(0, partIDIndex);
    Eigen::MatrixXd totJac, tempJ;
    int taskSize = robot.armsModel_.at(partID)->GetNumJoints();
    if (partID == robot.vehicle_->GetID()) {
                //TODO because it depends also on the arm.
    } else {
        if (robot.vehicle_) {

            totJac = RightJuxtapose(totJac, Eigen::MatrixXd::Zero(taskSize, 6));
        }
        for (std::map<std::string, std::shared_ptr<rml::ArmModel> >::iterator iter = robot.armsModel_.begin(); iter != robot.armsModel_.end();
             ++iter) {
            if(iter->first == partID)
            {   int i =0;
                 tempJ = Eigen::MatrixXd::Identity(taskSize, taskSize);
            }
            else
            {
                tempJ = Eigen::MatrixXd::Zero(taskSize, robot.armsModel_.at(partID)->GetNumJoints());

            }
            totJac = RightJuxtapose(totJac, tempJ);
        }

    }
    return totJac;
}

Eigen::MatrixXd RobotModel::GetJacobian_Manipulability(std::string ID, rml::RobotModel robot)
{

    std::cout << "get jacobian manipolability frame =" << ID << std::endl;
    return Eigen::Matrix3d::Identity();
}
} /* namespace rml */
