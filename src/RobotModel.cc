/*
 * RobotModel.cc
 *
 *  Created on: Feb 27, 2018
 *      Author: fraw
 */

#include "rml/RobotModel.h"
#include "rml_internal/Futils.h"
#include "rml/Functions.h"
#include "rml/MatrixOperations.h"

namespace rml
{

RobotModel::RobotModel()
{
	// TODO Auto-generated constructor stub

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
		return (arms_.size() - 1);
	} else {
		std::cout << tc::redL << "Error: Loaded a NOT initialised ArmModel" << tc::none << std::endl;
		return -1;
	}

}

bool RobotModel::CheckArm(int armIndex) const throw (std::exception)
{
	if (armIndex < arms_.size()) {
		return true;
	} else {
		throw RobotModelArmException();
	}
}

bool RobotModel::CheckVehicle() const throw (std::exception)
{
	if (vehicle_) {
		return true;
	} else {
		//std::cout << tc::redL << "Error: Arm index is out of range" << tc::none << std::endl;
		throw RobotModelVehicleException();
	}
}

Eigen::MatrixXd RobotModel::GetIsolatedArmJacobianTF(int armIndex) const
{
	Eigen::MatrixXd bJt;
	if (CheckArm(armIndex)) {
		/// The robot model actually returns the jacobian of the end-effector  w.r.t. the base of the robot
		bJt = arms_.at(armIndex)->GetbJt();
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
			Eigen::TransfMatrix bTt = arms_.at(armIndex)->GetbTt();
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
		bJj = arms_.at(armIndex)->GetBase2JointJacobian(jointIndex);
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

} /* namespace rml */
