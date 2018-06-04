/**
 * \file
 *
 * \date 	Feb 8, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef __ARMMODEL_H__
#define __ARMMODEL_H__

#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_map>

#include "Types.h"
#include "RobotLink.h"

namespace rml
{

typedef std::pair<int, Eigen::TransfMatrix> IndexedTMat;

/**
 * \brief Exception to be thrown when the joint index out of bounds
 */
class ArmModelException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "[ArmModel] Joint index out of bounds!";
	}
};
/**
 * \class ArmModel
 *
 * \brief Arm Model class for serial kinematic chains (manipulators).
 *
 * \details This class implements a model for serial kinematic chains. It contains a
 * vector of RobotLink which can be added up using the AddLink() function, which attaches
 * each new link to the previous one. Optionally we can also add rigid bodies to each
 * link with the function AddRigidBodyFrame() which takes as input the \p linkIndex to be
 * attached to and a \p IDstring to identify it.
 *
 * Once the model has been constructed, the class
 * provides all the necessary functions to evaluate the transformation and jacobian matrices
 * for every link and rigid body added.
 *
 * The standard Tool Control Point (TCP) assumed in the
 * GetBase2ToolTransf(), GetBaseToToolJacobian() and GetdJdq() functions, here called the \p ToolFrame,
 * is defined as the last link frame added (the EndEffector) plus the \p eTt = \f$ ^{ee}_{tool}T \f$
 * (EndEffector to Tool) matrix:
 *
 * \f$ ^{base}_{tool}T = ^{base}_{ee}T \cdot ^{ee}_{tool}T \f$
 *
 * By default \p eTt is an identity, but can be set using the SeteTt() function.
 *
 * To evaluate transformation and jacobians matrices different from the tool frame related ones
 * there are dedicated functions such as:
 * GetBase2JointTransf(), GetAttachedBodyTransf() GetBase2JointJacobian() etc... which take as
 * an input parameter the joint index or the attached body string identifier.
 *
 * <b>This class has been designed with two use cases in mind</b>:
 *   -# Used by itself in can be exploited to control fixed base manipulators.
 *   -# Loaded in a RobotModel, using RobotModel::LoadArm(), and used in conjuction with a VehicleModel,
 *   it can be used to control mobile manipulators.
 */
class ArmModel
{
public:

	/**
	 * \brief Default constructor
	 */
	ArmModel();

	/**
	 * \brief Default destructor
	 */
	virtual ~ArmModel();

	/**
	 * \brief Adds a link to the kinematic chain of the model
	 *
	 * \param type 			The JointType, whether: JointType::Fixed, JointType::Revolute, JointType::Prismatic
	 * \param axis			The axis along which the joint rotates or translates
	 * \param baseTransf 	Transformation matrix from previous to current
	 * \param jointLimMin Minimum excursion for the joint
	 * \param jointLimMax Maximum excursion for the joint
	 */
	void AddLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin,
			double jointLimMax);

	/**
	 * \brief Set the joint position
	 *
	 * The method updates the internal joint position state. This method updates also the internal transformation
	 * matrices and jacobians.
	 *
	 * \param[in] q		the joint position vector (must be an numJoints x 1 vector)
	 */
	void SetJointsPosition(const Eigen::VectorXd& q);

	/**
	 * \brief Get the joint position
	 * \return q the joint position vector (an armJoints x 1 vector)
	 */
	const Eigen::VectorXd& GetJointsPosition() const;

	void SetJointsVelocity(const Eigen::VectorXd& qdot);

	const Eigen::VectorXd& GetJointsVelocity() const;

	void SetJointsAcceleration(const Eigen::VectorXd& qddot);

	const Eigen::VectorXd& GetJointsAcceleration() const;

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
	 * @param[in] jointIndex    Joint index
	 * @return				   Joint transformation matrix
	 */
	Eigen::TransfMatrix GetBase2JointTransf(int jointIndex);

	/**
	 * @brief Evaluates the jacobian matrix (w.r.t. robot base) of the specified joint
	 *
	 * @param[in] jointIndex    Joint index
	 * @return				   Joint jacobian matrix
	 */
	Eigen::MatrixXd GetBase2JointJacobian(int jointIndex);

	void AddRigidBodyFrame(std::string ID, int jointIndex, Eigen::TransfMatrix TMat);

	Eigen::TransfMatrix GetAttachedBodyFrame(std::string& ID);
	Eigen::TransfMatrix GetAttachedBodyTransf(std::string& ID);

	Eigen::MatrixXd GetAttachedBodyJacobian(std::string& ID);

	int GetNumJoints() const
	{
		return links_.size();
	}

	const Eigen::TransfMatrix& GetBaseTransf()
	{
		return baseTb0_; // = baseTb0;
	}

	void SetBaseTransf(const Eigen::TransfMatrix& baseTb0)
	{
		baseTb0_ = baseTb0;
	}

  const Eigen::MatrixXd& GetBase2ToolJacobian() const
	{
		return bJt_;
	}

	const Eigen::TransfMatrix& GetCurrentLinkTransf(int ji)
	{
		return biTei_.at(ji);
	}

	const Eigen::TransfMatrix& GetBase2ToolTransf()
	{
		return bTt_;
	}

	const Eigen::TransfMatrix& GeteTt() const
	{
		return eTt_;
	}

	void SeteTt(const Eigen::TransfMatrix& eTt)
	{
		eTt_ = eTt;
	}

	const std::vector<Eigen::MatrixXd>& GetdJdq() const
	{
		return dJdq_;
	}

	RobotLink& GetLink(int jointIndex) throw (ArmModelException)
	{
		if (jointIndex < links_.size())
			return links_.at(jointIndex);
		else
			throw ArmModelException();
	}

	bool IsModelInitialized() const
	{
		return modelInitialized_;
	}

	const Eigen::VectorXd& GetControlVector() const
	{
		return controlRef_;
	}

	void SetControlVector(const Eigen::VectorXd& controlRef) throw (std::exception)
	{
		if (controlRef.rows() == numberOfJoints_) {
			controlRef_ = controlRef;
		} else {
			throw ArmModelException();
		}
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

	/*
	 * @brief Loads the model matrices from the files located in file_path
	 * (file names: wTb0, eTt, biTri1, biTri2, biTri3, etc...)
	 *
	 * @param[in] file_path		where all the model matrices files are
	 */
	//void ReadModelMatricesFromFile(std::string file_path);

	/**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 */
	void EvaluatedJdqNumeric();

	void ForwardDirectGeometry(int jointNumber);
	void BackwardDirectGeometry(int jointNumber, int endEffectorIndex);
	void BackwardDirectGeometryToolFrame(int jointNumber);

	bool modelInitialized_;
	int numberOfJoints_;
	std::vector<RobotLink> links_;
	std::unordered_map<std::string, IndexedTMat> attachedBodyFrames_;

	Eigen::VectorXd q_, q_dot_, q_ddot_, controlRef_;
	std::vector<Eigen::TransfMatrix> baseTei_;
	std::vector<Eigen::TransfMatrix> biTei_;
	Eigen::TransfMatrix baseTb0_;
	Eigen::TransfMatrix baseTbi_;
	Eigen::TransfMatrix Tz_;
	Eigen::Vector3d base_ki_;
	std::vector<Eigen::Vector6d> h_;
	Eigen::TransfMatrix bTt_;
	Eigen::TransfMatrix eTt_;

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

