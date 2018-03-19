/*
 * ctrl_vehiclemodel.h
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#ifndef __CTRL_VEHICLEMODEL_H__
#define __CTRL_VEHICLEMODEL_H__

#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_map>

#include <rml/Types.h>

namespace rml {

/**
 * @brief Vehicle Model base class
 *
 * @details This class implements a base vehicle model class.
 */
class VehicleModel
{
public:

	/**
	 * @brief Default constructor
	 */
	VehicleModel();

	/**
	 * @brief Default destructor
	 */
	virtual ~VehicleModel();

	/**
	 * @brief Jacobian initialization
	 */
	void SetJacobian(Eigen::Matrix6d vehicleJacobian);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values 
	 * @param[in] fbkPos the base position vector in the form of [y p r x y z]
	 */
	void SetFeedbackPosition(const Eigen::Vector6d& fbkPos);

	/**
	 * @brief Set the base position
	 * The method updates the internal base velocity state. This method should be called before the evaluate methods in order to
	 * have the updated values
	 * @param[in] fbkVel the base velocity vector in the form of [wx wy wz x y z]
	 */
	void SetFeedbackVelocity(const Eigen::Vector6d& fbkVel);

    /**
     * @brief Get the complete 6D position, in the form of [y p r x y z],
     * independently from the DOF of the vehicle.
     *
     * @return      The vehicle position
     */
	const Eigen::Vector6d& GetFeedbackPosition(){
		return fbkPosition_;
	}

	/**
	 * @brief Get the complete 6D velocity, in the form of [wx wy wz x y z],
	 * independently from the Dof of the vehicle.
	 *
	 * @return       The vehicle velocity
	 */
	const Eigen::Vector6d& GetFeedbackVelocity(){
		return fbkVelocity_;
	}

	void SetFeedbackAcceleration(const Eigen::Vector6d& fbkAcc);

	const Eigen::Vector6d& GetFeedbackAcceleration(){
			return fbkAcceleration_;
		}

	const Eigen::Vector6d& GetCartesianVelocity();

	const Eigen::Vector6d& GetCartesianAcceleration();

	void AddRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat);


	Eigen::TransfMatrix GetAttachedBodyTransf(std::string& ID);
	Eigen::TransfMatrix GetCurrentAttachedBodyTransf(std::string& ID);

	Eigen::MatrixXd GetAttachedBodyJacobian(std::string& ID);

	const Eigen::TransfMatrix GetwTv() {
		return fbkPosition_.ToTransfMatrix();
	}

	const Eigen::Matrix6d& GetvJv() const {
		return vJv_;
	}

	bool IsModelInitialized() const {
		return modelInitialized_;
	}

protected:

	bool modelInitialized_;
	std::unordered_map< std::string, Eigen::TransfMatrix > attachedBodyFrames_;
	Eigen::Vector6d fbkPosition_, fbkVelocity_, fbkAcceleration_;
	Eigen::Vector6d cartVelocity_, cartAcceleration_;

	Eigen::Matrix6d vJv_;
	Eigen::TransfMatrix wTv_;
	Eigen::TransfMatrix wTb_;
	Eigen::RotMatrix I3_;

};

}

#endif /* __CTRL_VEHICLEMODEL_H__ */

