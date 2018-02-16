/*
 * ctrl_armmodel.h
 *
 *  Created on: Jul 25, 2015
 *      Author: francescow
 */

#ifndef __CTRL_ARMMODEL_H__
#define __CTRL_ARMMODEL_H__

#include <vector>
#include <algorithm>	// for std::copy
#include <cmat/cmat.h>

#include "ctrl_defines.h"

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
	 * @brief Select the model representation
	 * The method allows to select which internal representation should be used for all the subsequent calls
	 * backward-forward method for computing the Jacobian and tool transformation matrix (CTRL_MODEL_BACKWARDFORWARD)
	 * @param[in] representation the chosen representation
	 * @return CTRL_RV_FAIL if the representation does not exist
	 * @return CTRL_RV_OK otherwise
	 */
	virtual int SelectModelRepresentation(int representation);

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
	void SetJointPosition(const CMAT::Matrix& q);

	/**
	 * @brief Get the joint position
	 * @param[in] q the joint position vector (an armJoints x 1 vector)
	 */
	const CMAT::Matrix& GetJointPosition() const;

	/**
	 * @brief Evaluates the Jacobian of the arm
	 * This method returns the Jacobian matrix
	 * @param[out] wJt the Jacobian matrix
	 */
	void EvaluatewJt(CMAT::Matrix& wJt);

	void EvaluatebJt(CMAT::Matrix& bJt);

	/**
	 * @brief Evaluates the tool transformation matrix
	 * This method returns tool transformation matrix
	 * @param[out] wJt the tool transformation matrix
	 */
	void EvaluatewTt(CMAT::TransfMatrix& wTt);

	void EvaluatebTt(CMAT::TransfMatrix& bTt);

	/**
	 * @brief Evaluates the manipulability measure and its Jacobian
	 * This method returns he manipulability measure and its Jacobian
	 * @param[out] mu the manipulability measure
	 * @param[out] wJt the manipulability measure Jacobian
	 */
	void EvaluateManipulability(CMAT::Matrix& mu, CMAT::Matrix& Jmu);

	/**
	 * @brief Evaluates the transformation matrix (w.r.t. world) of the specified joint
	 *
	 * @param[out] bTelw        Joint transformation matrix
	 * @param[in] jointIndex    Joint index
	 */
	void EvaluateWorld2JointTransf(CMAT::TransfMatrix& wTj, int jointIndex);

	void EvaluateBase2JointTransf(CMAT::TransfMatrix& bTj, int jointIndex);
    /**
     * @brief Evaluates the jacobian matrix (w.r.t. world) of the specified joint
     *
     * @param[out] bJj          Joint jacobian matrix
     * @param[in] jointIndex    Joint index
     */
	void EvaluateWorld2JointJacobian(CMAT::Matrix& wJj, int jointIndex);

	void EvaluateBase2JointJacobian(CMAT::Matrix& bJj, int jointIndex);

	/**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 *
	 * @param[out] dJdq
	 */
	void EvaluatedJdqNumeric(CMAT::Matrix* dJdq);

	/**
	 * @brief Evaluates the Jacobian derivative w.r.t. joint variations
	 *
	 * @param[out] dJdq
	 */
	virtual void EvaluatedJdq(CMAT::Matrix* dJdq);

	int GetNumJoints() const {
		return numberOfJoints_;
	}

	void SetwTb(const CMAT::TransfMatrix& wTb) {
		wTb0_ = wTb;
	}

	const CMAT::Matrix& GetwJt() {
		//EvaluatewJt(wJt_);
		return wJt_;
	}

    const CMAT::Matrix& GetbJt() {
        //EvaluatewJt(wJt_);
        return bJt_;
    }

    const CMAT::TransfMatrix& GetwTt() {
        //EvaluatewTt(wTt_);
        return wTt_;
    }

    const CMAT::TransfMatrix& GetbTt() {
        //EvaluatewTt(wTt_);
        bTt_ = wTb0_.Inverse() * wTt_;
        return bTt_;
    }


	const CMAT::TransfMatrix& GeteTt() const {
		return eTt_;
	}

	void SeteTt(const CMAT::TransfMatrix& eTt) {
		eTt_ = eTt;
	}

    CMAT::Matrix* const GetdJdq() const {
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

	/**
	 * @brief Reads from the give file and loads the values in a CMAT::TransfMatrix
	 *
	 * @param matrix_path
	 * @param target_mat
	 */
	void FromFile2TransfMatrix(std::string matrix_path, CMAT::TransfMatrix &target_mat);

	/**
	 * @brief Given that the number of joints is known, this function calls the function
	 * "FromFile2TransfMatrix" for all the biTri#
	 *
	 * @param file_path
	 */
	void ReadbiTriFromFile(std::string file_path);

	void ForwardDirectGeometry(int jointNumber);
	void BackwardDirectGeometry(int jointNumber, int endEffectorIndex);
	void BackwardDirectGeometryToolFrame(int jointNumber);


	int numberOfJoints_;
	int modelRepresentation_;
	CMAT::Matrix q_;
	CMAT::TransfMatrix* wTei_; 		///< Matrice di Trasformazione dal mondo all'endeffector della BRU i-esima
	CMAT::TransfMatrix* biTri_;		///< Matrice di Trasformazione dalla base all'endeffector della BRU i-esima (costante)
	CMAT::TransfMatrix* biTei_;		///< biTei = biTri * Tz(qi); Matrice di T dalla base all'ee della BRU i-esima tenuto conto della rotazione del giunto
	CMAT::TransfMatrix wTb0_;		///< Matrice di Trasformazione dal mondo alla base del Robot(costante)
	CMAT::TransfMatrix wTbi_;
	CMAT::TransfMatrix Tz_;
	CMAT::Vect3 w_ki_;
	CMAT::Vect6* h_;
	CMAT::TransfMatrix wTt_, bTt_;
	CMAT::TransfMatrix eTt_;		///< Matrice di Trasformazione dall'endeffector al tool(costante)
	CMAT::Vect3 w_r_et_;	// vector of the distance between the end effector frame and the tool frame, projected on the world frame

	CMAT::Matrix* dJdq_;
	CMAT::Matrix Jpinv_;
	CMAT::Matrix djdqJpinv_;

	CMAT::TransfMatrix wTe_;
	CMAT::Matrix wJt_, bJt_;
	double* arrayJ_;
	double* arrayQ_;
	CMAT::Matrix I3_;
	CMAT::Matrix ZeroQ_;
	bool modelReadFromFile;

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

template <typename Derived>
class ArmModel_CRTP : public ArmModel {
public:
    virtual ArmModel *clone() const {
        return new Derived(static_cast<Derived const&>(*this));
    }
};

}

#endif /* __CTRL_ARMMODEL_H__ */

