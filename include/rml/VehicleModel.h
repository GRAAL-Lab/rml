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

	/**
	 * @brief Copy constructor
	 */
	VehicleModel(const VehicleModel& other);

	/**
	 * @brief Specialized swap() to implement the copy-assignment-operator by reusing the copy-constructor
	 */
	friend void swap(rml::VehicleModel& first, rml::VehicleModel& second);

	/**
	 * @brief Copy Assignment Operator
	 */
	VehicleModel& operator=(VehicleModel other);

	/**
	 * @brief Pure virtual function implemented in the BaseModel_CRTP (Curiously Recurring Template Pattern),
	 * @return a copy of the derived class
	 */
	virtual VehicleModel *clone() const;


	/**
	 * @brief Internal matrices initialization, hard coded in derived class
	 * This method *must* be called before any other, but after the SetArmJoints
	 * It initializes all the matrices to be later used in the evaluation methods
	 */
	virtual void InitMatrix();

	/**
	 * @brief Set the number of base DOFs
	 * This method sets the number of arm joints in the internal state of the class, and allocates all the matrices 
	 * with the given dimensions
	 * @param[in] baseDOFs the number of base degrees of freedom
	 */
	void SetBaseDOFs(int baseDOFs);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values 
	 * @param[in] q the base position vector (must be an armJoints x 1 vector)
	 */
	void SetPosition(const Eigen::MatrixXd& q);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values
	 * @param[in] qdot the base position vector (must be an armJoints x 1 vector)
	 */
	void SetVelocity(const Eigen::MatrixXd& qdot);

//    /**
//     * @brief Get the complete 6D position, in the form of [y p r x y z],
//     * independently from the DOF of the vehicle.
//     *
//     * @param[out] vehiclePos       The vehicle position
//     */
//    virtual void Get6DPosition(Eigen::Vector6d& vehiclePos);
//
//	/*
//	 * @brief Get the complete 6D velocity, in the form of [wx wy wz x y z],
//	 * independently from the Dof of the vehicle.
//	 *
//	 * @param[out] vehicleVel       The vehicle velocity
//	 */
//	virtual void Get6DVelocity(Eigen::Vector6d& vehicleVel);

	int GetDoFs() const {
		return vehicleDOFs_;
	}

	/**
	 * @brief Get the joint position
	 * @return the joint position vector (a baseDOFs x 1 vector)
	 */
	const Eigen::MatrixXd& GetPosition() const {
		return q_;
	}

	const Eigen::MatrixXd& GetVelocity() const {
		return qdot_;
	}

	const Eigen::TransfMatrix& GetvTb() const {
		return vTb_;
	}

	const Eigen::TransfMatrix& GetwTv() {

		return wTv_;
	}

	const Eigen::TransfMatrix& GetwTb() {
		wTb_ = wTv_ * vTb_;
		return wTb_;
	}

	const Eigen::MatrixXd& GetvJv() const {
		return vJv_;
	}

protected:

	/**
	 * @brief Evaluates the vehicle transformation matrix
	 *
	 * @param[out] wTv the vehicle transformation matrix
	 */
	virtual void EvaluatewTv();

	int vehicleDOFs_;
	Eigen::MatrixXd q_, qdot_;

	Eigen::MatrixXd vJv_;
	Eigen::TransfMatrix wTv_;
	Eigen::TransfMatrix vTb_;
	Eigen::TransfMatrix wTb_;
	Eigen::TransfMatrix T_;
	std::vector<Eigen::Vector6d> h_;
	//double* arrayQ_;

	Eigen::RotMatrix I3_;
	Eigen::MatrixXd Zeros_;

};

}

#endif /* __CTRL_VEHICLEMODEL_H__ */

