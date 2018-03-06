/*
 * ctrl_vehiclemodel.h
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#ifndef __CTRL_VEHICLEMODEL_H__
#define __CTRL_VEHICLEMODEL_H__

#include <vector>
#include <rml/Types.h>

namespace rml {

/**
 * @brief Vehicle Model base class
 *
 * @details This class implements a base vehicle model class.
 * The derived class should re-implement the InitMatrix method, to set the geometry of the arm and the
 * EvaluatedJdq method to evaluate the derivative of the Jacobian w.r.t. q that is used in the manipulability Jacobian
 * computation.
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

//	/**
//	 * @brief Copy constructor
//	 */
//	VehicleModel(const VehicleModel& other);

	/**
	 * @brief Specialized swap() to implement the copy-assignment-operator by reusing the copy-constructor
	 */
	friend void swap(rml::VehicleModel& first, rml::VehicleModel& second);

//	/**
//	 * @brief Copy Assignment Operator
//	 */
//	VehicleModel& operator=(VehicleModel other);


	/**
	 * @brief Jacobian initialization
	 */
	void SetJacobian(Eigen::Matrix6d vehicleJacobian);

//	/**
//	 * @brief Set the number of base DOFs
//	 * This method sets the number of arm joints in the internal state of the class, and allocates all the matrices
//	 * with the given dimensions
//	 * @param[in] baseDOFs the number of base degrees of freedom
//	 */
//	void SetBaseDOFs(int baseDOFs);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values 
	 * @param[in] q the base position vector in the form of [y p r x y z]
	 */
	void SetFeedbackPosition(const Eigen::Vector6d& fbkPos);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values
	 * @param[in] qdot the base position vector in the form of [wx wy wz x y z]
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

	const Eigen::Vector6d& GetCartesianVelocity();



//	int GetDoFs() const {
//		return vehicleDOFs_;
//	}


//	const Eigen::TransfMatrix& GetvTb() const {
//		return vTb_;
//	}

	const Eigen::TransfMatrix& GetwTv() {
		return wTv_;
	}

//	const Eigen::TransfMatrix& GetwTb() {
//		wTb_ = wTv_ * vTb_;
//		return wTb_;
//	}

	const Eigen::Matrix6d& GetvJv() const {
		return vJv_;
	}

	bool IsModelInitialized() const {
		return modelInitialized_;
	}

protected:

	bool modelInitialized_;
	Eigen::Vector6d fbkPosition_, fbkVelocity_, cartVelocity_;

	Eigen::Matrix6d vJv_;
	Eigen::TransfMatrix wTv_;
	//Eigen::TransfMatrix vTb_;
	Eigen::TransfMatrix wTb_;
	//Eigen::TransfMatrix T_;
	Eigen::RotMatrix I3_;

};

}

#endif /* __CTRL_VEHICLEMODEL_H__ */

