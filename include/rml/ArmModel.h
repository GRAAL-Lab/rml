/*
 * ctrl_armmodel.h
 *
 *  Created on: Feb 8, 2018
 *      Author: francescow
 */

#ifndef __ARMMODEL_H__
#define __ARMMODEL_H__

#include <vector>
#include <algorithm>		// for std::copy
#include <OrtoMath.h>
#include <eigen3/Eigen/Dense>

namespace CTRL {

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
	friend void swap(CTRL::ArmModel& first, CTRL::ArmModel& second);

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
	const Eigen::Vector4d& GetJointPosition() const;

	/**
	 * @brief Evaluates the Jacobian of the arm
	 * This method returns the Jacobian matrix
	 * @param[out] wJt the Jacobian matrix
	 */
	void EvaluatewJt(Eigen::MatrixXd& wJt);

	void EvaluatebJt(Eigen::MatrixXd& bJt);

	/**
	 * @brief Evaluates the tool transformation matrix
	 * This method returns tool transformation matrix
	 * @param[out] wJt the tool transformation matrix
	 */
	void EvaluatewTt(Eigen::Matrix4d& wTt);



	void EvaluatebTt(Eigen::Matrix4d& bTt);

	/**
	 * @brief Evaluates the manipulability measure and its Jacobian
	 * This method returns he manipulability measure and its Jacobian
	 * @param[out] mu the manipulability measure
	 * @param[out] wJt the manipulability measure Jacobian
	 */
	void EvaluateManipulability(Eigen::Matrix4d& mu, CMAT::Matrix& Jmu);

	/**
	 * @brief Evaluates the transformation matrix (w.r.t. world) of the specified joint
	 *
	 * @param[out] bTelw        Joint transformation matrix
	 * @param[in] jointIndex    Joint index
	 */
	void EvaluateWorld2JointTransf(Eigen::Matrix4d& wTj, int jointIndex);

	void EvaluateBase2JointTransf(Eigen::Matrix4d& bTj, int jointIndex);
    /**
     * @brief Evaluates the jacobian matrix (w.r.t. world) of the specified joint
     *
     * @param[out] bJj          Joint jacobian matrix
     * @param[in] jointIndex    Joint index
     */
	void EvaluateWorld2JointJacobian(Eigen::MatrixXd& wJj, int jointIndex);

	void EvaluateBase2JointJacobian(Eigen::MatrixXd& bJj, int jointIndex);

	/**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 *
	 * @param[out] dJdq
	 */
	void EvaluatedJdqNumeric(std::vector<Eigen::MatrixXd> dJdq);

	/**
	 * @brief Evaluates the Jacobian derivative w.r.t. joint variations
	 *
	 * @param[out] dJdq
	 */
	virtual void EvaluatedJdq(std::vector<Eigen::MatrixXd> dJdq);

	int GetNumJoints() const {
		return numberOfJoints_;
	}

	void SetwTb(const Eigen::Matrix4d& wTb) {
		wTb0_ = wTb;
	}

	const Eigen::MatrixXd& GetwJt() const {
		//EvaluatewJt(wJt_);
		return wJt_;
	}

    const Eigen::MatrixXd& GetbJt() const {
        //EvaluatewJt(wJt_);
        return bJt_;
    }

    const Eigen::Matrix4d& GetwTt() const {
        //EvaluatewTt(wTt_);
        return wTt_;
    }

    const Eigen::Matrix4d& GetbTt() const {
        //EvaluatewTt(wTt_);
        bTt_ = wTb0_.inverse() * wTt_;
        return bTt_;
    }


	const Eigen::Matrix4d& GeteTt() const {
		return eTt_;
	}

	void SeteTt(const Eigen::Matrix4d& eTt) const {
		eTt_ = eTt;
	}

	const std::vector<Eigen::MatrixXd>& GetdJdq() const {
        return dJdq_;
    }

protected:

	/**
	 * @brief Loads the model matrices from the files located in file_path
	 * (file names: wTb0, eTt, biTri1, biTri2, biTri3, etc...)
	 *
	 * @param[in] file_path		where all the model matrices files are
	 */
	void ReadModelMatricesFromFile(std::string file_path);


	void ForwardDirectGeometry(int jointNumber);
	void BackwardDirectGeometry(int jointNumber, int endEffectorIndex);
	void BackwardDirectGeometryToolFrame(int jointNumber);


	int numberOfJoints_;
	Eigen::VectorXd q_;
	std::vector<Eigen::Matrix4d> wTei_; 		///< Matrice di Trasformazione dal mondo all'endeffector della BRU i-esima
	std::vector<Eigen::Matrix4d> biTri_;		///< Matrice di Trasformazione dalla base all'endeffector della BRU i-esima (costante)
	std::vector<Eigen::Matrix4d> biTei_;		///< biTei = biTri * Tz(qi); Matrice di T dalla base all'ee della BRU i-esima tenuto conto della rotazione del giunto
	Eigen::Matrix4d wTb0_;		///< Matrice di Trasformazione dal mondo alla base del Robot(costante)
	Eigen::Matrix4d wTbi_;
	Eigen::Matrix4d Tz_;
	Eigen::Vector3d w_ki_;
	std::vector<Eigen::Vector6d> h_;
	Eigen::Matrix4d wTt_, bTt_;
	Eigen::Matrix4d eTt_;		///< Matrice di Trasformazione dall'endeffector al tool(costante)
	Eigen::Vector3d w_r_et_;	// vector of the distance between the end effector frame and the tool frame, projected on the world frame

	std::vector<Eigen::MatrixXd> dJdq_;
	Eigen::MatrixXd Jpinv_;
	Eigen::MatrixXd djdqJpinv_;

	Eigen::Matrix4d wTe_;
	Eigen::MatrixXd wJt_, bJt_;
	std::vector<double> arrayJ_;
	std::vector<double> arrayQ_;
	Eigen::Matrix3d I3_;
	Eigen::VectorXd ZeroQ_;
	bool modelReadFromFile;

};



}

#endif /* __ARMMODEL_H__ */

