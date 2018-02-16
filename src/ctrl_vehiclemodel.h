/*
 * ctrl_vehiclemodel.h
 *
 *  Created on: May 16, 2017
 *      Author: francescow
 */

#ifndef __CTRL_VEHICLEMODEL_H__
#define __CTRL_VEHICLEMODEL_H__

#include <vector>
#include <algorithm>	// for std::copy
#include <cmat/cmat.h>

#include "ctrl_defines.h"

namespace CTRL {

/**
 * @brief Vehicle Model base class
 * This class implements a base arm model class
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
	friend void swap(CTRL::VehicleModel& first, CTRL::VehicleModel& second);

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
	void SetBaseDOFs(int armJoints);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values 
	 * @param[in] q the base position vector (must be an armJoints x 1 vector)
	 */
	void SetPosition(const CMAT::Matrix& q);

	/**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values
	 * @param[in] q the base position vector (must be an armJoints x 1 vector)
	 */
	void SetVelocity(const CMAT::Matrix& qdot);

	/**
	 * @brief Evaluates the vehicle transformation matrix
	 * This method returns vehicle transformation matrix
	 *
	 * @param[out] wTv the vehicle transformation matrix
	 */
	virtual void EvaluatewTv(CMAT::TransfMatrix& wTv);

    /**
     * @brief Get the complete 6D position, in the form of [y p r x y z],
     * independently from the DOF of the vehicle.
     *
     * @param[out] vehiclePos       The vehicle position
     */
    virtual void Get6DPosition(CMAT::Vect6& vehiclePos);

	/**
	 * @brief Get the complete 6D velocity, in the form of [wx wy wz x y z],
	 * independently from the Dof of the vehicle.
	 *
	 * @param[out] vehicleVel       The vehicle velocity
	 */
	virtual void Get6DVelocity(CMAT::Vect6& vehicleVel);

	int getDoFs() const {
		return vehicleDOFs_;
	}

	/**
	 * @brief Get the joint position
	 * @param[in] q the joint position vector (a baseDOFs x 1 vector)
	 */
	const CMAT::Matrix& GetPosition() const {
		return q_;
	}

	const CMAT::Matrix& GetVelocity() const {
		return qdot_;
	}

	const CMAT::TransfMatrix& GetvTb() const {
		return vTb_;
	}

	const CMAT::TransfMatrix& GetwTv() {
		EvaluatewTv(wTv_);
		return wTv_;
	}

	const CMAT::TransfMatrix& GetwTb() {
		EvaluatewTv(wTv_);
		wTb_ = wTv_ * vTb_;
		return wTb_;
	}

	const CMAT::Matrix& GetvJv() const {
		return vJv_;
	}

protected:

	int vehicleDOFs_;
	CMAT::Matrix q_, qdot_;

	CMAT::Matrix vJv_;
	CMAT::TransfMatrix wTv_;
	CMAT::TransfMatrix vTb_;
	CMAT::TransfMatrix wTb_;
	CMAT::TransfMatrix T_;
	CMAT::Vect6* h_;
	double* arrayQ_;

	CMAT::Matrix I3_;
	CMAT::Matrix Zeros_;

};

/*
 * NOTE on CRTP C++ Pattern
 * ------------------------
 *
 * CRTP: Curiously recurring template
 *
 * In short, CRTP is when a class A has a base class which is a template specialization for the class A itself. E.g.
 *
 *    template <class T> class X{...};
 *    class A : public X<A> {...};
 *
 * It is curiously recurring, isn't it? :)
 * Now, what does this give you? This actually gives the X template the ability to be a base class for its specializations.
 *
 * This was needed since BaseModel can be either a specialized class for a particular robot (especially where dJdq is defined
 * for calculating the manipulability) or a generic class where the model is loaded runtime.
 */

template <typename Derived>
class VehicleModel_CRTP : public VehicleModel {
public:
    virtual VehicleModel *clone() const {
        return new Derived(static_cast<Derived const&>(*this));
    }
};

}

#endif /* __CTRL_VEHICLEMODEL_H__ */

