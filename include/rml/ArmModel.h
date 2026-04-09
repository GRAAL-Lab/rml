/**
 * \file
 *
 * \date 	Feb 8, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef __ARMMODEL_H__
#define __ARMMODEL_H__

#include "RMLExceptions.h"
#include "RobotLink.h"
#include "Types.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace rml {

//typedef std::pair<int, Eigen::TransformationMatrix> IndexedTMat;
typedef std::pair<std::string, Eigen::TransformationMatrix> IndexedTMat;

/**
 * \class ArmModel
 *
 * \brief Arm Model class for serial kinematic chains (manipulators).
 *
 * \details This class implements a model for serial kinematic chains, the whole structure is identified by an ID given by the user in the constructor. It contains a
 * vector of RobotLink which can be added up using the AddFixedLink() function for fixed links and the AddJointLink() one for moving links, the two methods attach
 * each new link to the previous one. Optionally we can also add rigid bodies to each link with the function SetRigidBodyFrame() which takes as input
 * the \p frameName to identify the frame to which attach the body and a \p IDstring to identify it.
 *
 * Once the model has been constructed, the class provides all the necessary functions to evaluate the transformation and jacobian matrices
 * for every link and rigid body added.
 * Each frame is identified by a label. The following policy is used:
 *
 * * Joint Frame: armID + FrameID::Joint + joint°;
 *
 * * Rigid Body: armID + "_" + frameID
 *
 * In order to get the transformation matrix the GetTransformation() method is provided. The method takes as input a string which is the id of the frame asked.
 *

 * All the transformation matrices are expressed wrt the arm base.
 * In order to get the jacobian the GetJacobian() method is provided. The method takes as input a string which is the id of the frame asked.
 * All the jacobian matrix are expressed wrt the arm base.
 *
 * <b>This class has been designed with two use cases in mind</b>:
 *
 * * Used by itself in can be exploited to control fixed base manipulators.
 *
 * * Loaded in a RobotModel, using RobotModel::LoadArm(), it can be used to control either mobile manipulators or multiple arms.
 */
class ArmModel {
public:
    /**
	 * \brief Default constructor
     * @param[in] id Arm id.
	 */
    ArmModel(const std::string id) noexcept(false);
    /**
	 * \brief Default destructor
	 */
    virtual ~ArmModel();
    /**
	 * \brief Adds a link to the kinematic chain of the model
	 *
     * \param type 			The JointType, whether: JointType::Revolute, JointType::Prismatic
	 * \param axis			The axis along which the joint rotates or translates
     * \param baseTransf 	Transformation matrix from previous link to current link
     * \param jointLimMin   Minimum excursion for the joint
     * \param jointLimMax   Maximum excursion for the joint
	 */
    void AddJointLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransformationMatrix& baseTransf, double jointLimMin, double jointLimMax);
    /**
    * \brief Adds a fixed link to the kinematic chain of the model
    *
    * \param baseTransf 	Transformation matrix from previous to current
    */
    void AddFixedLink(const Eigen::TransformationMatrix& baseTransf);
    /**
	 * \brief Set the joint position
	 *
     * The method updates the internal joint position state [only the moving joints]. This method updates also the internal transformation
	 * matrices and jacobians.
	 *
     * \param[in] q the joint position vector (must be a numMovingJoints x 1 vector)
	 */
    virtual void JointsPosition(const Eigen::VectorXd q) noexcept(false);
    /**
     * \brief Get the moving joint position
     * \return q the joint position vector (a numMovingJoints x 1 vector)
	 */
    virtual auto JointsPosition() const -> const Eigen::VectorXd& { return q_moving_; }
    /**
     * @brief Set the moving joints velocity
     * @param qdot the joint velocity vector (must be a numMovingJoints x 1 vector)
     */
    virtual void JointsVelocity(const Eigen::VectorXd qdot) noexcept(false);
    /**
     * @brief Get the moving joints velocity
     * @return  the joints velocity vector (must be a numMovingJoints x 1 vector)
     */
    virtual auto JointsVelocity() const -> const Eigen::VectorXd& { return q_dot_moving_; }
    /**
     * @brief Set the moving joints acceleration.
     * @param qddot the joints acceleration vector (must be a numMovingJoints x 1 vector)
     */
    void JointsAcceleration(const Eigen::VectorXd qddot) noexcept(false);
    /**
     * @brief Get moving joints acceleration.
     * @return  the joints acceleration vector (must be a numMovingJoints x 1 vector)
     */
    auto JointsAcceleration() const -> const Eigen::VectorXd& { return q_ddot_moving_; }
    /**
     * @brief Method adding a rigid body frame to a joint.
     * @param frameID Id of the frame to add
     * @param attachedFrameID ID of the frame to attach
     * @param TMat Transformation matrix of the frame.
     */
    void AttachRigidBodyFrame(std::string frameID, std::string attachedFrameID, Eigen::TransformationMatrix attachedFrameID_T_frameID) noexcept(false);
    /**
     * @brief Method returning the transformation matrix related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return  transformation matrix
     */


    /*
    * For CartesianJacobian, TransformationMatrix, Jacobian and ManipulabilityJacobian the id must be provieded according the following logic:
    *
    * * Joint Frame: armID + FrameID::Joint + joint°;
    *
    * * Rigid Body: armID + "_" + frameID
    */
    Eigen::TransformationMatrix TransformationMatrix(const std::string& frameID) noexcept(false);
    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix jTk.
     */
    Eigen::TransformationMatrix TransformationMatrix(const std::string& frameID_j, const std::string& frameID_k);
    /**
     * @brief Method returning the jacobian related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return jacobian matrix
     */
    virtual Eigen::MatrixXd Jacobian(const std::string& frameID) noexcept(false);
    /**
     * @brief Method returning the manipulability jacobian related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return jacobian matrix
     */
    Eigen::MatrixXd ManipulabilityJacobian(const std::string& frameID);
    /**
     * @brief Method returning the arm number of moving  joints
     * @return  arm number of moving joints
     */
    virtual auto NumJoints() const -> unsigned int { return movingNumJoints_; }
    /**
     * @brief Method returning dJdq evaluated numerically
     * @return  dJdq
     */
    auto dJdq() const -> const std::vector<Eigen::MatrixXd>& { return dJdq_; }
    /**
     * @brief Method returning the arm link
     * @param jointIndex link index
     * @return robot link
     */
    virtual RobotLink& Link(int jointIndex) noexcept(false);
    /**
     * @brief Method returning true if the model is initialized false otherwise.
     */
    auto IsModelInitialized() const -> bool { return modelInitialized_; }
    /**
     * @brief Method returning the arm control vector
     * @return Control vector
     */
    auto ControlVector() const -> const Eigen::VectorXd& { return controlRef_; }
    /**
     * @brief Method setting the arm control vector
     * @param controlRef control vector
     */
    void ControlVector(const Eigen::VectorXd controlRef) noexcept(false);
    /**
     * @brief Method returning the arm manipulability
     * @return  manipulability value.
     */
    double Manipulability(const std::string& frameID);
    /**
     * @brief Method returning the dexterity jacobian related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return jacobian matrix
     */
    Eigen::MatrixXd DexterityJacobian(const std::string& frameID);
    /**
     * @brief Method returning the arm dexterity
     * @return  dexterity value.
     */
    double Dexterity(const std::string& frameID);
    /**
     * @brief Method returning the arm id.
     * @return arm id
     */
    auto ID() const -> const std::string& { return id_; }
    /**
     * @brief Method setting the arm id
     * @param id arm id.
     */
    auto ID() -> std::string& { return id_; }

    /**
     * @brief Method returning the list of rigid body frame IDs
     * @return vector of frame IDs
     */
    std::vector<std::string> GetRigidBodyFrameIDs() const noexcept;

    /**
     * @brief Method returning the list of joint frame IDs
     * @return vector of frame IDs
     */
    std::vector<std::string> GetJointFrameIDs() const noexcept;

    /**
     * @brief Method returning the list of Jacobian frame IDs
     * @return vector of frame IDs
     */
    std::vector<std::string> GetJacobianFrameIDs() const noexcept;

    /**
     * @brief Method returning the end-effector frame ID
     * @return frame ID
     */
    std::string GetEndEffectorFrameID() const noexcept;
    
protected:
    /**
     * @brief Method evaluating the total forward geometry for the arm
     */
    void EvaluateTotalForwardGeometry();
    /**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 */
    virtual void EvaluatedJdqNumeric();
    /**
     * @brief Evaluates the manipulability measure and its Jacobian
     * This method returns the manipulability measure and its Jacobian
     */
    virtual Eigen::MatrixXd EvaluateManipulability(const std::string& frameID);
    /**
     * @brief Evaluates the dexterity measure and its Jacobian
     * This method returns the dexterity measure and its Jacobian
     */
    virtual Eigen::MatrixXd EvaluateDexterity(const std::string& frameID);
    /**
     * @brief Method returning the attached body frame wrt to the arm base.
     * @param ID frame ID.
     * @return  transformation matrix.
     */
    Eigen::TransformationMatrix EvaluateRigidBodyTransf(const std::string& frameID);
    /**
     * @brief Method returning the attached body frame jacobian wrt to the arm base.
     * @param ID frame ID.
     * @return jacobian matrix.
     */
    Eigen::MatrixXd EvaluateRigidBodyJacobian(const std::string& frameID);
    /**
     * @brief Evaluates the jacobian matrix (w.r.t. robot base) of the specified joint
     *
     * @param[in] jointIndex    Joint index
     * @return				   Joint jacobian matrix
     */
    Eigen::MatrixXd EvaluateBase2JointJacobian(unsigned int jointIndex);
    /**
     * @brief Method performing the forward direct geometry untill the input joint number.
     * @param jointNumber which joint is intended as last of the chain
     */
    void ForwardDirectGeometry(unsigned int jointNumber);
    /**
     * @brief Backward Direct Geometry from the input joint number to the input end effector index
     * @param jointNumber which joint is intended as last of the chain
     * @param endEffectorIndex
     */
    void BackwardDirectGeometry(unsigned int jointNumber, unsigned int endEffectorIndex); 

    bool modelInitialized_; //!< boolean stating whether the model is initialized
    bool isMapInitialized_; //!< boolean stating whether the transformation and jacobian maps are initialized
    unsigned int totalNumJoints_; //<! links number (fixedLink+movingJoints)
    unsigned int movingNumJoints_; //!< moving joints number
    std::vector<unsigned int> movingJoints_; //!< vector containing the indexes of the moving joints
    std::vector<RobotLink> links_; //!< vector of the arm links
    std::unordered_map<std::string, IndexedTMat> rigidBodyFrames_; //<! map of the attached body frames
    std::unordered_map<std::string, Eigen::MatrixXd> jacobians_; //<! map of the jacobians
    std::unordered_map<std::string, Eigen::TransformationMatrix> transformation_; //<! map of the transformations
    std::unordered_map<std::string, Eigen::MatrixXd> manipulabilityJacobians_; //!< map of the manipulability jacobians
    std::unordered_map<std::string, double> manipulability_; //!< map of the manipulability values
    Eigen::VectorXd q_total_; //<! vector of links position
    Eigen::VectorXd q_moving_; //<! vector of moving joints position
    Eigen::VectorXd q_dot_moving_; //<! vector of moving joints velocity
    Eigen::VectorXd q_ddot_moving_; //<! vector of moving joints acceleration
    Eigen::VectorXd controlRef_; //<! control vector
    std::vector<Eigen::TransformationMatrix> baseTei_; //<! vector of transformation matrix from base to joint   ??
    std::vector<Eigen::TransformationMatrix> biTei_; //<! vector of transformation matrix from joint i-1 to joint i ??
    Eigen::TransformationMatrix baseTbi_; //<! transformation matrix from base to joint i ??
    Eigen::TransformationMatrix Tz_; //!< transformation matrix for rotation or prismatic joint
    Eigen::Vector3d base_ki_; //!<
    std::vector<Eigen::Vector6d> h_; //!<
    std::vector<Eigen::MatrixXd> dJdq_; //!< dJdq evaluated numerically
    Eigen::MatrixXd djdqJpinv_; //!< djdq * jacobian pseudoinverse
    Eigen::RotationMatrix I3_; //!< identity matrix
    bool modelReadFromFile_; //!< boolean stating whether the model is read from file
    std::string id_; //!< arm id
    std::unordered_map<std::string, Eigen::MatrixXd> dexterityJacobians_; //!< map of the manipulability jacobians
    std::unordered_map<std::string, double> dexterity_; //!< map of the manipulability values
    Eigen::MatrixXd Jdjdq_; //!< jacobian * djdq 
    Eigen::MatrixXd Jpinvdjpinvdq_; //!< jacobian pseudoinverse * djpinvdq
};
}

#endif /* __ARMMODEL_H__ */
