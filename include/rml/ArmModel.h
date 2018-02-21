/*
 * ctrl_armmodel.h
 *
 *  Created on: Feb 8, 2018
 *      Author: francescow
 */

#ifndef __ARMMODEL_H__
#define __ARMMODEL_H__

#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "rml/RML.h"

namespace rml {

/**
 * @brief Exception to be thrown when the joint index out of bounds
 */
class ArmModelException: public std::exception
{
	virtual const char* what() const throw () {
		return "[ArmModel] Wrong joint index!";
	}
};

/**
 * @brief Arm Model base class
 * This class implements a base arm model class
 * The derived class should re-implement the InitMatrix method, to set the geometry of the arm and the
 * EvaluatedJdq method to evaluate the derivative of the Jacobian w.r.t. q that is used in the manipulability Jacobian
 * computation.
 */

class ArmModel
{
public:

	/**
	 * @brief Default constructor
	 */
	ArmModel();

	/**
	 * @brief Default destructor
	 */
	virtual ~ArmModel();

	/**
	 * @brief Copy constructor
	 */
	ArmModel(const ArmModel& other);

	/**
	 * @brief Specialized swap() to implement the copy-assignment-operator by reusing the copy-constructor
	 */
	friend void swap(ArmModel& first, ArmModel& second);

	/**
	 * @brief Copy Assignment Operator
	 */
	ArmModel& operator=(ArmModel other);

	/**
	 * @brief Pure virtual function implemented in the ArmModel_CRTP (Curiously Recurring Template Pattern),
	 * @return a copy of the derived class
	 */
	virtual ArmModel *clone() const;

	/**
	 * @brief Internal matrices initialization, hard coded in derived class
	 * This method *must* be called before any other, but after the SetArmJoints
	 * It initializes all the matrices to be later used in the evaluation methods
	 */
	virtual void InitMatrix();

	/**
	 * @brief Internal matrices initialization taken from external files
	 * This method *must* be called before any other, but after the SetArmJoints
	 * It initializes all the matrices to be later used in the evaluation methods
	 * @param[in] matrices_path folder where the n+2 model files are in (wTb0, eTt, biTri[0-numJoints])
	 */
	virtual void InitMatrix(std::string matrices_path);

	/**
	 * @brief Set the number of arm joints
	 * This method sets the number of arm joints in the internal state of the class, and allocates all the matrices 
	 * with the given dimensions
	 * @param[in] armJoints the number of arm joints
	 */
	void SetArmJoints(int armJoints);

	/**
	 * @brief Set the joint position
	 * The method updates the internal joint position state. This method should be called before the evaluate methods in order to 
	 * have the updated values 
	 * @param[in] q the joint position vector (must be an armJoints x 1 vector)
	 */
	void SetJointPosition(const Eigen::VectorXd& q);

	/**
	 * @brief Get the joint position
	 * @param[in] q the joint position vector (an armJoints x 1 vector)
	 */
	const Eigen::VectorXd& GetJointPosition() const;

	/**
	 * @brief Evaluates the Jacobian of the arm
	 * This method returns the Jacobian matrix
	 * @param[out] wJt the Jacobian matrix
	 */
	void EvaluatebJt();

	/**
	 * @brief Evaluates the tool transformation matrix
	 * This method returns tool transformation matrix
	 * @param[out] wJt the tool transformation matrix
	 */
	void EvaluatebTt();

	/**
	 * @brief Evaluates the manipulability measure and its Jacobian
	 * This method returns he manipulability measure and its Jacobian
	 * @param[out] mu the manipulability measure
	 * @param[out] wJt the manipulability measure Jacobian
	 */
	void EvaluateManipulability(Eigen::MatrixXd& Jmu, double& mu);

	/**
	 * @brief Evaluates the transformation matrix (w.r.t. world) of the specified joint
	 *
	 * @param[out] bTelw        Joint transformation matrix
	 * @param[in] jointIndex    Joint index
	 */
	void EvaluateWorld2JointTransf(Eigen::TransfMatrix& wTj, int jointIndex);

	void EvaluateBase2JointTransf(Eigen::TransfMatrix& bTj, int jointIndex);
	/**
	 * @brief Evaluates the jacobian matrix (w.r.t. world) of the specified joint
	 *
	 * @param[out] bJj          Joint jacobian matrix
	 * @param[in] jointIndex    Joint index
	 */
	void EvaluateWorld2JointJacobian(Eigen::MatrixXd& wJj, int jointIndex);

	void EvaluateBase2JointJacobian(Eigen::MatrixXd& bJj, int jointIndex);


	int GetNumJoints() const {
		return numberOfJoints_;
	}

	void SetwTb(const Eigen::TransfMatrix& wTb) {
		wTb0_ = wTb;
	}

	const Eigen::MatrixXd& GetbJt() const {
		return bJt_;
	}


	const Eigen::TransfMatrix& GetbTt() {
		return bTt_;
	}


	const Eigen::TransfMatrix& GeteTt() const {
		return eTt_;
	}

	void SeteTt(const Eigen::TransfMatrix& eTt) {
		eTt_ = eTt;
	}

	const std::vector<Eigen::MatrixXd>& GetdJdq() const {
		return dJdq_;
	}

	const double GetJointMinLimit(int jointIndex) throw (ArmModelException) {
		if(jointIndex < numberOfJoints_)
			return jointLimitsMin_.at(jointIndex);
		else
			throw ArmModelException();
	}

	const double GetJointMaxLimit(int jointIndex){
		if(jointIndex < numberOfJoints_)
			return jointLimitsMAX_.at(jointIndex);
		else
			throw ArmModelException();
	}

protected:

	/**
	 * @brief Loads the model matrices from the files located in file_path
	 * (file names: wTb0, eTt, biTri1, biTri2, biTri3, etc...)
	 *
	 * @param[in] file_path		where all the model matrices files are
	 */
	void ReadModelMatricesFromFile(std::string file_path);

	/**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 *
	 * @param[out] dJdq
	 */
	void EvaluatedJdqNumeric();


	void ForwardDirectGeometry(int jointNumber);
	void BackwardDirectGeometry(int jointNumber, int endEffectorIndex);
	void BackwardDirectGeometryToolFrame(int jointNumber);

	bool hasBeenInitialized_;
	int numberOfJoints_;
	Eigen::VectorXd q_;
	std::vector<Eigen::TransfMatrix> wTei_; 		///< Matrice di Trasformazione dal mondo all'endeffector della BRU i-esima
	std::vector<Eigen::TransfMatrix> biTri_;		///< Matrice di Trasformazione dalla base all'endeffector della BRU i-esima (costante)
	std::vector<Eigen::TransfMatrix> biTei_;		///< biTei = biTri * Tz(qi); Matrice di T dalla base all'ee della BRU i-esima tenuto conto della rotazione del giunto
	Eigen::TransfMatrix wTb0_;						///< Matrice di Trasformazione dal mondo alla base del Robot(costante)
	Eigen::TransfMatrix wTbi_;
	Eigen::TransfMatrix Tz_;
	Eigen::Vector3d w_ki_;
	std::vector<Eigen::Vector6d> h_;
	Eigen::TransfMatrix bTt_;
	Eigen::TransfMatrix eTt_;		///< Matrice di Trasformazione dall'endeffector al tool(costante)
	Eigen::Vector3d w_r_et_;	// vector of the distance between the end effector frame and the tool frame, projected on the world frame

	std::vector<Eigen::MatrixXd> dJdq_;
	Eigen::MatrixXd Jpinv_;
	Eigen::MatrixXd djdqJpinv_;

	Eigen::TransfMatrix wTe_;
	Eigen::MatrixXd bJt_;
	Eigen::RotMatrix I3_;
	Eigen::VectorXd ZeroQ_;
	std::vector< double > jointLimitsMin_;
	std::vector< double > jointLimitsMAX_;
	bool modelReadFromFile_;

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
 * This was needed since ArmModel can be either a specialized class for a particular robot (especially where dJdq is defined
 * for calculating the manipulability) or a generic class where the model is loaded runtime.
 */

//template <typename Derived>
//class ArmModel_CRTP : public ArmModel {
//public:
//    virtual ArmModel *clone() const {
//        return new Derived(static_cast<Derived const&>(*this));
//    }
//};



}

#endif /* __ARMMODEL_H__ */

