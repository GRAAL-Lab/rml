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

#include <rml/Types.h>
#include <rml/RobotLink.h>

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
 *
 * @details This class implements a base arm model class.
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

	void AddLink(Eigen::TransfMatrix, )

	/**
	 * @brief Internal matrices initialization, hard coded in derived class
	 * This method *must* be called before any other, but after the SetArmJoints
	 * It initializes all the matrices to be later used in the evaluation methods
	 */
	//virtual void InitMatrix();

	/**
	 * @brief Internal matrices initialization taken from external files
	 * This method *must* be called before any other, but after the SetArmJoints
	 * It initializes all the matrices to be later used in the evaluation methods
	 * @param[in] matrices_path folder where the n+2 model files are in (wTb0, eTt, biTri[0-numJoints])
	 */
	//virtual void InitMatrix(std::string matrices_path);

	/**
	 * @brief Set the number of arm joints
	 * This method sets the number of arm joints in the internal state of the class, and allocates all the matrices 
	 * with the given dimensions
	 * @param[in] armJoints the number of arm joints
	 */
	//void SetArmJoints(int armJoints);

	/**
	 * @brief Set the joint position
	 * The method updates the internal joint position state. This method should be called before the evaluate methods in order to 
	 * have the updated values 
	 * @param[in] q the joint position vector (must be an armJoints x 1 vector)
	 */
	void SetJointPosition(const Eigen::VectorXd& q);

	/**
	 * @brief Get the joint position
	 * @return q the joint position vector (an armJoints x 1 vector)
	 */
	const Eigen::VectorXd& GetJointPosition() const;



	/**
	 * @brief Evaluates the manipulability measure and its Jacobian
	 * This method returns he manipulability measure and its Jacobian
	 * @param[out] mu the manipulability measure
	 * @param[out] Jmu the manipulability measure Jacobian
	 */
	void EvaluateManipulability(Eigen::MatrixXd& Jmu, double& mu);

	/**
	 * @brief Evaluates the transformation matrix (w.r.t. robot base) of the specified joint
	 *
	 * @param[out] bTj        Joint transformation matrix
	 * @param[in] jointIndex    Joint index
	 */
	void EvaluateBase2JointTransf(Eigen::TransfMatrix& bTj, int jointIndex);

	/**
	 * @brief Evaluates the jacobian matrix (w.r.t. robot base) of the specified joint
	 *
	 * @param[out] bJj          Joint jacobian matrix
	 * @param[in] jointIndex    Joint index
	 */
	void EvaluateBase2JointJacobian(Eigen::MatrixXd& bJj, int jointIndex);


	int GetNumJoints() const {
		return numberOfJoints_;
	}

	const Eigen::TransfMatrix& GetBaseTransf() {
		return baseTb0_;// = baseTb0;
	}

	void SetBaseTransf(const Eigen::TransfMatrix& baseTb0) {
		baseTb0_ = baseTb0;
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

	const RobotLink& GetLink(int jointIndex) throw (ArmModelException) {
		if(jointIndex < numberOfJoints_)
			return links_.at(jointIndex);
		else
			throw ArmModelException();
	}

protected:

	/**
	 * @brief Evaluates the Jacobian of the arm
	 * This method returns the Jacobian matrix
	 * @return wJt the Jacobian matrix
	 */
	void EvaluatebJt();

	/**
	 * @brief Evaluates the tool transformation matrix
	 * This method returns tool transformation matrix
	 * @return the tool transformation matrix
	 */
	void EvaluatebTt();

	/**
	 * @brief Loads the model matrices from the files located in file_path
	 * (file names: wTb0, eTt, biTri1, biTri2, biTri3, etc...)
	 *
	 * @param[in] file_path		where all the model matrices files are
	 */
	void ReadModelMatricesFromFile(std::string file_path);

	/**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 */
	void EvaluatedJdqNumeric();


	void ForwardDirectGeometry(int jointNumber);
	void BackwardDirectGeometry(int jointNumber, int endEffectorIndex);
	void BackwardDirectGeometryToolFrame(int jointNumber);

	bool hasBeenInitialized_;
	std::vector<RobotLink> links_;
	int numberOfJoints_;
	Eigen::VectorXd q_;
	std::vector<Eigen::TransfMatrix> baseTei_; 		///< Matrice di Trasformazione dalla base del robot all'endeffector della BRU i-esima
	std::vector<Eigen::TransfMatrix> biTri_;		///< Matrice di Trasformazione dalla base all'endeffector della BRU i-esima (costante)
	std::vector<Eigen::TransfMatrix> biTei_;		///< biTei = biTri * Tz(qi); Matrice di T dalla base all'ee della BRU i-esima tenuto conto della rotazione del giunto
	Eigen::TransfMatrix baseTb0_;						///< Matrice di Trasformazione dal mondo alla base del Robot(costante)
	Eigen::TransfMatrix baseTbi_;
	Eigen::TransfMatrix Tz_;
	Eigen::Vector3d base_ki_;
	std::vector<Eigen::Vector6d> h_;
	Eigen::TransfMatrix bTt_;
	Eigen::TransfMatrix eTt_;		///< Matrice di Trasformazione dall'endeffector al tool (costante)

	std::vector<Eigen::MatrixXd> dJdq_;
	Eigen::MatrixXd Jpinv_;
	Eigen::MatrixXd djdqJpinv_;

	Eigen::MatrixXd bJt_;
	Eigen::RotMatrix I3_;
	Eigen::VectorXd ZeroQ_;


	bool modelReadFromFile_;

};

}

#endif /* __ARMMODEL_H__ */

