/*
 * RobotModel.h
 *
 *  Created on: Feb 27, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_ROBOTMODEL_H_
#define INCLUDE_RML_ROBOTMODEL_H_

#include <rml/VehicleModel.h>
#include <rml/ArmModel.h>

/**
 * \class RobotModel
 *
 * \ingroup RML
 *
 * \brief This class provides a container for storing multi-arm mobile manipulators,
 * including a series of model related functions.
 *
 * Detailed description TODO.
 *
 *
 * \author (last to touch it) fw
 *
 * \date 2018/02/28 12:06:20
 *
 * Contact: francesco.wanderlingh@dibris.unige.it
 *
 * Created on: Tue Feb 27 10:22:30 2018
 *
 *
 */

#include <vector>

namespace rml {

class RobotModel {

	VehicleModel vehicle_;
	std::vector<ArmModel> arms_;

public:
	RobotModel();
	virtual ~RobotModel();
/*
	void RobotModel::ArmOnVehicleJacobiansEE(CMAT::Matrix& Ja, CMAT::Matrix& Jv)
	{
		/// The robot model actually returns the transformation of the end-effector  w.r.t. the base of the robot
		bTt_ = armModel_->GetbTt();

		/// The robot model actually returns the jacobian of the end-effector  w.r.t. the base of the robot
		bJt_ = armModel_->GetbJt();

		//vJv_ = vehicleModel_->GetvJv();

		CMAT::RotMatrix vRb = vehicleModel_->GetvTb().GetRotMatrix();
		CMAT::TransfMatrix vTt_ = vehicleModel_->GetvTb() * bTt_;

		Ja = vRb.GetCartesianRotationMatrix() * bJt_;
		Jv = vTt_.GetTrasl().GetRigidBodyMatrix() * vehicleModel_->GetvJv();
		//Jv.PrintMtx("Jv");
	}


	void RobotModel::ArmOnVehicleJacobiansForJoint(CMAT::Matrix& Ja, CMAT::Matrix& Jv, int jointIndex)
	{
	    /// The robot model actually returns the transformation of the end-effector  w.r.t. the base of the robot
	    armModel_->EvaluateBase2JointTransf(bTj_, jointIndex);

	    /// The robot model actually returns the jacobian of the end-effector  w.r.t. the base of the robot
	    armModel_->EvaluateBase2JointJacobian(bJj_, jointIndex);

	    //vJv_ = vehicleModel_->GetvJv();

	    CMAT::RotMatrix vRb = vehicleModel_->GetvTb().GetRotMatrix();
	    CMAT::TransfMatrix vTj_ = vehicleModel_->GetvTb() * bTj_;

	    Ja = vRb.GetCartesianRotationMatrix() * bJj_;
	    Jv = vTj_.GetTrasl().GetRigidBodyMatrix() * vehicleModel_->GetvJv();
	    //Jv.PrintMtx("Jv");
	}
*/
	const ArmModel& GetArm(int index) const {
		return arms_.at(index);
	}

	const VehicleModel& GetVehicle() const {
		return vehicle_;
	}
};

} /* namespace rml */

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
