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
#include <memory>

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

class RobotModel {

	std::shared_ptr<VehicleModel> vehicle_;
	std::vector<std::shared_ptr<ArmModel> > arms_;

	Eigen::MatrixXd J_;
	std::vector<Eigen::TransfMatrix> vehicleTbase_;
	std::vector<Eigen::MatrixXd> JArm_;
	std::vector<Eigen::Matrix6d> JVeh_;

	Eigen::MatrixXd GetIsolatedArmJacobianTF(const int armIndex) const throw (std::exception);
	Eigen::Matrix6d GetIsolatedVehicleJacobianEE(const int armIndex) const throw (std::exception);

	Eigen::MatrixXd GetIsolatedArmJacobianForJoint(int armIndex, int jointIndex) const throw (std::exception);
	Eigen::Matrix6d GetIsolatedVehicleJacobianForJoint(int armIndex, int jointIndex) const throw (std::exception);

public:
	RobotModel();
	virtual ~RobotModel();

	bool LoadVehicle(const std::shared_ptr<VehicleModel> vehicle);
	bool LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb);
	bool CheckArm(int armIndex) const;

	Eigen::MatrixXd GetArmJacobianTF(int armIndex);
	Eigen::MatrixXd GetVehicleJacobianTF(int armIndex);


	const std::shared_ptr<ArmModel> GetArm(int index) const {
		return arms_.at(index);
	}

	const std::shared_ptr<VehicleModel> GetVehicle() const {
		return vehicle_;
	}
};

} /* namespace rml */

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
