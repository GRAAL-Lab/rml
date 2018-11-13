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

//typedef std::pair<int, Eigen::TransfMatrix> IndexedTMat;
typedef std::pair<std::string, Eigen::TransfMatrix> IndexedTMat;

/**
 * \class ArmModel
 *
 * \brief Arm Model class for serial kinematic chains (manipulators).
 *
 * \details This class implements a model for serial kinematic chains, the whole structure is identified by an ID given by the user in the constructor. It contains a
 * vector of RobotLink which can be added up using the AddFixedLink() function for fixed links and the AddJointLink() one for moving links, the two methods attach
 * each new link to the previous one. Optionally we can also add rigid bodies to each
 * link with the function SetRigidBodyFrame() which takes as input the \p frameName to identify the frame
 * to which attach the body and a \p IDstring to identify it.\n
 *
 * Once the model has been constructed, the class provides all the necessary functions to evaluate the transformation and jacobian matrices
 * for every link and rigid body added.\n
 * Each frame is identified by a label. The following policy is used: \n
 *
 * * Joint Frame: armID + FrameID::Joint + joint°;
 *
 * * Rigid Body: armID + FrameID::Body + frameID
 *
 * In order to get the transformation matrix the GetTransformation() method is provided. The method takes as input a string which is the id of the frame asked. \n
 * All the transformation matrices are expressed wrt the arm base.\n
 * In order to get the jacobian the GetJacobian() method is provided. The method takes as input a string which is the id of the frame asked. \n
 * All the jacobian matrix are expressed wrt the arm base.\n
 * \n
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
    ArmModel(const std::string id) throw(std::exception);

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
    void AddJointLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin,
        double jointLimMax);

    /**
    * \brief Adds a fixed link to the kinematic chain of the model
    *
    * \param baseTransf 	Transformation matrix from previous to current
    */
    void AddFixedLink(const Eigen::TransfMatrix& baseTransf);

    /**
	 * \brief Set the joint position
	 *
     * The method updates the internal joint position state [only the moving joints]. This method updates also the internal transformation
	 * matrices and jacobians.
	 *
     * \param[in] q the joint position vector (must be a numMovingJoints x 1 vector)
	 */
    void SetJointsPosition(const Eigen::VectorXd& q) throw(std::exception);

    /**
     * \brief Get the moving joint position
     * \return q the joint position vector (a numMovingJoints x 1 vector)
	 */
    const Eigen::VectorXd& GetJointsPosition() const;

    /**
     * @brief Set the moving joints velocity
     * @param qdot the joint velocity vector (must be a numMovingJoints x 1 vector)
     */
    void SetJointsVelocity(const Eigen::VectorXd& qdot) throw(std::exception);

    /**
     * @brief Get the moving joints velocity
     * @return  the joints velocity vector (must be a numMovingJoints x 1 vector)
     */
    const Eigen::VectorXd& GetJointsVelocity() const;

    /**
     * @brief Set the moving joints acceleration.
     * @param qddot the joints acceleration vector (must be a numMovingJoints x 1 vector)
     */
    void SetJointsAcceleration(const Eigen::VectorXd& qddot) throw(std::exception);

    /**
     * @brief Get moving joints acceleration.
     * @return  the joints acceleration vector (must be a numMovingJoints x 1 vector)
     */
    const Eigen::VectorXd& GetJointsAcceleration() const;

    /**
     * @brief Method adding a rigid body frame to a joint.
     * @param frameID Id of the frame to add
     * @param attachedFrameID ID of the frame to attach
     * @param TMat Transformation matrix of the frame.
     */
    void SetRigidBodyFrame(std::string frameID, std::string attachedFrameID, Eigen::TransfMatrix TMat) throw(ExceptionWithHow);

    /**
     * @brief Method returning the transformation matrix related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return  transformation matrix
     */

    Eigen::TransfMatrix GetTransformation(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.\n
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix jTk.
     */

    Eigen::TransfMatrix GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k);
    /**
     * @brief Method returning the jacobian related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return jacobian matrix
     */
    Eigen::MatrixXd GetJacobian(const std::string& frameID) throw(ExceptionWithHow);

    /**
     * @brief Method returning the manipulability jacobian related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return jacobian matrix
     */

    Eigen::MatrixXd GetManipulabilityJacobian(const std::string& frameID);
    /**
     * @brief Method returning the arm number of moving  joints
     * @return  arm number of moving joints
     */
    int GetNumJoints() const;

    /**
     * @brief Method returning dJdq evaluated numerically
     * @return  dJdq
     */
    const std::vector<Eigen::MatrixXd>& GetdJdq() const;
    /**
     * @brief Method returning the arm link
     * @param jointIndex link index
     * @return robot link
     */
    RobotLink& GetLink(int jointIndex) throw(std::exception);

    /**
     * @brief Method returning true if the model is initialized false otherwise.
     */

    bool IsModelInitialized() const;

    /**
     * @brief Method returning the arm control vector
     * @return Control vector
     */
    const Eigen::VectorXd& GetControlVector() const;

    /**
     * @brief Method setting the arm control vector
     * @param controlRef control vector
     */
    void SetControlVector(const Eigen::VectorXd& controlRef) throw(std::exception);

    /**
     * @brief Method returning the arm manipulability
     * @return  manipulability value.
     */
    double GetManipulability(std::string frameID);
    /**
     * @brief Method returning the arm id.
     * @return arm id
     */
    std::string GetID();
    /**
     * @brief Method setting the arm id
     * @param id arm id.
     */
    void SetID(std::string id);

protected:
    /**
     * @brief Method evaluating the total forward geometry for the arm
     */
    void EvaluateTotalForwardGeometry();

    /**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 */
    void EvaluatedJdqNumeric();

    /**
     * @brief Evaluates the manipulability measure and its Jacobian
     * This method returns the manipulability measure and its Jacobian
     */
    Eigen::MatrixXd EvaluateManipulability(const std::string frameID);

    /**
     * @brief Method returning the attached body frame wrt to the arm base.
     * @param ID frame ID.
     * @return  transformation matrix.
     */
    Eigen::TransfMatrix EvaluateRigidBodyTransf(const std::string& frameID);

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
    Eigen::MatrixXd EvaluateBase2JointJacobian(int jointIndex);

    /**
     * @brief Method performing the forward direct geometry untill the input joint number.
     * @param jointNumber which joint is intended as last of the chain
     */
    void ForwardDirectGeometry(int jointNumber);

    /**
     * @brief Backward Direct Geometry from the input joint number to the input end effector index
     * @param jointNumber which joint is intended as last of the chain
     * @param endEffectorIndex
     */
    void BackwardDirectGeometry(int jointNumber, int endEffectorIndex);

    bool modelInitialized_; //!< boolean stating whether the model is initialized
    bool isMapInitialized_; //!< boolean stating whether the transformation and jacobian maps are initialized
    int totalNumJoints_; //<! links number (fixedLink+movingJoints)
    int movingNumJoints_; //!< moving joints number
    std::vector<int> movingJoints_; //!< vector containing the indexes of the moving joints
    std::vector<RobotLink> links_; //!< vector of the arm links
    std::unordered_map<std::string, IndexedTMat> rigidBodyFrames_; //<! map of the attached body frames
    std::unordered_map<std::string, Eigen::MatrixXd> jacobians_; //<! map of the jacobians
    std::unordered_map<std::string, Eigen::TransfMatrix> transformation_; //<! map of the transformations
    std::unordered_map<std::string, Eigen::MatrixXd> manipulabilityJacobians_; //!< map of the manipulability jacobians
    std::unordered_map<std::string, double> manipulability_; //!< map of the manipulability values
    Eigen::VectorXd q_total_; //<! vector of links position
    Eigen::VectorXd q_moving_; //<! vector of moving joints position
    Eigen::VectorXd q_dot_moving_; //<! vector of moving joints velocity
    Eigen::VectorXd q_ddot_moving_; //<! vector of moving joints acceleration
    Eigen::VectorXd controlRef_; //<! control vector
    std::vector<Eigen::TransfMatrix> baseTei_; //<! vector of transformation matrix from base to joint   ??
    std::vector<Eigen::TransfMatrix> biTei_; //<! vector of transformation matrix from joint i-1 to joint i ??
    Eigen::TransfMatrix baseTbi_; //<! transformation matrix from base to joint i ??
    Eigen::TransfMatrix Tz_; //!< transformation matrix for rotation or prismatic joint
    Eigen::Vector3d base_ki_; //!<
    std::vector<Eigen::Vector6d> h_; //!<
    std::vector<Eigen::MatrixXd> dJdq_; //!< dJdq evaluated numerically
    Eigen::MatrixXd djdqJpinv_; //!< djdq * jacobian pseudoinverse
    Eigen::RotMatrix I3_; //!< identity matrix
    bool modelReadFromFile_; //!< boolean stating whether the model is read from file
    std::string id_; //!< arm id
};
}

#endif /* __ARMMODEL_H__ */
