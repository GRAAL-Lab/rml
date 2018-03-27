/*
 * RobotModel.h
 *
 *  Created on: Feb 27, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_ROBOTMODEL_H_
#define INCLUDE_RML_ROBOTMODEL_H_

#include <vector>
#include <memory>

#include "VehicleModel.h"
#include "ArmModel.h"

/**
 * \class RobotModel
 *
 * \ingroup RML
 *
 * \brief This class provides a container for storing multi-arm mobile manipulators,
 * including a series of model related functions.
 *
 * \author (last to touch it) fw
 *
 * \date 2018/02/28 12:06:20
 *
 * Contact: francesco.wanderlingh@dibris.unige.it
 *
 * Created on: Tue Feb 27 10:22:30 2018
 *
 */


namespace rml {

/**
 * @brief Exception to be thrown when the joint index out of bounds
 */
class RobotModelArmException: public std::exception
{
	virtual const char* what() const throw () {
		return "[RobotModel] Error: Arm index out of bounds!!!";
	}
};

/**
 * @brief Exception to be thrown when vehicle is not present
 */
class RobotModelVehicleException: public std::exception
{
	virtual const char* what() const throw () {
		return "[RobotModel] Error: No vehicle!!!";
	}
};

/**
 * @brief Robot Model class, stores vehicle and arms models and evaluates total transformation
 * matrices and Jacobians.
 *
 * @details This class implements a base vehicle model class.
 * The derived class should re-implement the InitMatrix method, to set the geometry of the arm and the
 * EvaluatedJdq method to evaluate the derivative of the Jacobian w.r.t. q that is used in the manipulability Jacobian
 * computation.
 */

class RobotModel {

	std::shared_ptr<VehicleModel> vehicle_;
	std::vector<std::shared_ptr<ArmModel> > arms_;

	Eigen::MatrixXd J_;
	std::vector<Eigen::TransfMatrix> vehicleTbase_;
	std::vector<Eigen::MatrixXd> JArm_;
	std::vector<Eigen::Matrix6d> JVeh_;

	Eigen::MatrixXd GetIsolatedArmJacobianTF(const int armIndex) const;
	Eigen::Matrix6d GetIsolatedVehicleJacobianEE(const int armIndex) const;

	Eigen::MatrixXd GetIsolatedArmJacobianForJoint(int armIndex, int jointIndex) const;
	Eigen::Matrix6d GetIsolatedVehicleJacobianForJoint(int armIndex, int jointIndex) const;

public:
	RobotModel();
	virtual ~RobotModel();

	/**
	 * @brief Returns the number of total degrees of fredoom of the system composed of
	 * a vehicle plus \p n arms.
	 * @return	The total number of DOFs (vehicle + arms)
	 */
	int GetTotalDOFs();

	/**
	 * @brief Loads a vehicle in the robot model
	 *
	 * Loads a vehicle in the robot model, only one vehicle is allowed. The VehicleModel
	 * must be initialized before loading, otherwise is not accepted.
	 *
	 * @param vehicle		The vehicle model
	 * @return				true on success, false otherwise
	 */
	bool LoadVehicle(const std::shared_ptr<VehicleModel> vehicle);

	/**
	 * @brief Loads an arm in the robot model
	 *
	 * Loads a arm in the robot model, there is no limit to the number of arms that can
	 * be loaded. The ArmModel must be initialized before loading, otherwise is not
	 * accepted. A <vehicle-to-base> transformation must be specified, which tells the
	 * position of the arm's base frame with respect to the vehicle frame. If no vehicle
	 * is meant to be loaded, it can be the identity matrix. The returning value is an
	 * ID which identifies the loaded arm within the RobotModel class.
	 *
	 * @param arm		The arm model
	 * @param vTb		The vehicle-to-base trasnformation matrix
	 * @return			The arm ID
	 */
	int LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb);

	/**
	 * Checks that the given armIndex is within the allowed range (i.e. less or equal than
	 * the number of loaded arms).
	 * @param armIndex
	 * @return
	 */
	bool CheckArm(int armIndex) const throw (std::exception);
	bool CheckVehicle() const throw (std::exception);

	Eigen::MatrixXd GetArmJacobian_JointFrame(int armIndex, int jointIndex);
	Eigen::MatrixXd GetVehicleJacobian_JointFrame(int armIndex, int jointIndex);
	Eigen::MatrixXd GetArmJacobian_ToolFrame(int armIndex);
	Eigen::MatrixXd GetVehicleJacobian_ToolFrame(int armIndex);
	Eigen::MatrixXd GetArmJacobian_Identity(int armIndex);
	Eigen::MatrixXd GetArmJacobian_Manipulability(int armIndex, double& mu);
	Eigen::MatrixXd GetVehicleJacobian();

	Eigen::TransfMatrix GetTransfMatrix_VehicleToArmBase(int armIndex) const;
	Eigen::TransfMatrix GetTransfMatrix_JointFrame(int armIndex, int jointIndex) const;
	Eigen::TransfMatrix GetTransfMatrix_ToolFrame(int armIndex) const;

	const std::shared_ptr<ArmModel> GetArm(int index) const {
		return arms_.at(index);
	}

	const std::shared_ptr<VehicleModel> GetVehicle() const {
		return vehicle_;
	}
};

} /* namespace rml */

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
