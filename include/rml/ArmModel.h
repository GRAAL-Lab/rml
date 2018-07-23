/**
 * \file
 *
 * \date 	Feb 8, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef __ARMMODEL_H__
#define __ARMMODEL_H__

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "RobotLink.h"
#include "Types.h"

namespace rml {

typedef std::pair<int, Eigen::TransfMatrix> IndexedTMat;
/**
 * \class ArmModel
 *
 * \brief Arm Model class for serial kinematic chains (manipulators).
 *
 * \details This class implements a model for serial kinematic chains. It contains a
 * vector of RobotLink which can be added up using the AddLink() function, which attaches
 * each new link to the previous one. Optionally we can also add rigid bodies to each
 * link with the function AddRigidBodyFrame() which takes as input the \p linkIndex to be
 * attached to and a \p IDstring to identify it.\n
 * The arm model is identified by an ID given by the user in the constructor.
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
 * Each frame is identified by a label. The following policy is used:
 * Tool Frame : armID+ “_Tool“
 * Joint Frame : armID+ “FrameID::Joint“+ joint°
 * Rigid Body: armID+ “_Body_“+ frameID
 * In order to get the transformation matrix the GetTransformation(string) method is provided.
 * The string in input is the id of the frame asked. \n
 * All the transformation matrix are expressed wrt the robot base.\n
 * In order to get the jacobian the GetJacobian(string) method is provided.\n
 * The string in input is the frameid of the frame asked.\n
 * All the jacobian matrix are expressed wrt the robot base.
 * <b>This class has been designed with two use cases in mind</b>:
 *   -# Used by itself in can be exploited to control fixed base manipulators.
 *   -# Loaded in a RobotModel, using RobotModel::LoadArm(), and used in conjuction with a VehicleModel,
 *   it can be used to control mobile manipulators.
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
	 * \param type 			The JointType, whether: JointType::Fixed, JointType::Revolute, JointType::Prismatic
	 * \param axis			The axis along which the joint rotates or translates
	 * \param baseTransf 	Transformation matrix from previous to current
	 * \param jointLimMin Minimum excursion for the joint
	 * \param jointLimMax Maximum excursion for the joint
	 */
    void AddJointLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin,
        double jointLimMax);

    /**
   * \brief Adds a link to the kinematic chain of the model
   *
   * \param type 			The JointType, whether: JointType::Fixed, JointType::Revolute, JointType::Prismatic
   * \param axis			The axis along which the joint rotates or translates
   * \param baseTransf 	Transformation matrix from previous to current
   * \param jointLimMin Minimum excursion for the joint
   * \param jointLimMax Maximum excursion for the joint
   */
    void AddFixedLink(const Eigen::TransfMatrix& baseTransf);

    /**
	 * \brief Set the joint position
	 *
	 * The method updates the internal joint position state. This method updates also the internal transformation
	 * matrices and jacobians.
	 *
	 * \param[in] q		the joint position vector (must be an numJoints x 1 vector)
	 */
    void SetJointsPosition(const Eigen::VectorXd& q) throw(std::exception);

    /**
	 * \brief Get the joint position
	 * \return q the joint position vector (an armJoints x 1 vector)
	 */
    const Eigen::VectorXd& GetJointsPosition() const;

    /**
     * @brief Set the joints velocity
     * @param qdot the joint velocity vector (must be a vector of dimension equal to numJoints)
     */
    void SetJointsVelocity(const Eigen::VectorXd& qdot) throw(std::exception);

    /**
     * @brief Get the joints velocity
     * @return  the joints velocity vector ( vector of dimension equal to numJoints)
     */
    const Eigen::VectorXd& GetJointsVelocity() const;

    /**
     * @brief Set the joints acceleration.
     * @param qddot the joints acceleration vector (must be a vector of dimension equal to numJoints)
     */
    void SetJointsAcceleration(const Eigen::VectorXd& qddot) throw(std::exception);

    /**
     * @brief Get joints acceleration.
     * @return  the joints acceleration vector (vector of dimension equal to numJoints)
     */
    const Eigen::VectorXd& GetJointsAcceleration() const;

    /**
     * @brief Method adding a rigid body frame to a joint.
     * @param ID Id of the frame-
     * @param jointIndex index of the joint to which the frame is attached.
     * @param TMat Transformation ,atrix of the frame.
     */
    void SetRigidBodyFrame(std::string ID, int jointIndex, Eigen::TransfMatrix TMat) throw(std::exception);

    /**
     * @brief Method returning the transformation matrix related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return  transformation matrix
     */

    Eigen::TransfMatrix GetTransformation(const std::string& frameID) throw(std::exception);
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
     * @return jacobianID the string identificator for the jacobian
     */
    Eigen::MatrixXd GetJacobian(const std::string& frameID) throw(std::exception);

    /**
     * @brief Method returning the manipulability jacobian related to the input frameID wrt to the arm base.
     * @param frameId frame id
     * @return frameID the string identificator for the frame
     */

    Eigen::MatrixXd GetManipulabilityJacobian(const std::string& frameID);
    /**
     * @brief Method returning the arm number of joints
     * @return  arm number of joints
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
     * @brief ArmModel::EvaluateTotalForwardGeometry
     */
    void EvaluateTotalForwardGeometry();

    /**
	 * @brief Evaluates numerically the Jacobian derivative w.r.t. joint variations
	 */
    void EvaluatedJdqNumeric();

    /**
     * @brief Evaluates the manipulability measure and its Jacobian
     * This method returns he manipulability measure and its Jacobian
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


    bool modelInitialized_; //!< boolean stating whether the model is initialized.
    bool isMapInitialized_; //!< boolean stating whether the transformation and jacobian maps are initialized.
    int totalNumJoints_; //<! joints number
    int movingNumJoints_;
    std::vector<int> movingJoints_;
    std::vector<RobotLink> links_; //!< vector of the arm links.
    std::unordered_map<std::string, IndexedTMat> rigidBodyFrames_; //<! map of the attached body frames.
    std::unordered_map<std::string, Eigen::MatrixXd> jacobians_; //<! map of the jacobians.
    std::unordered_map<std::string, Eigen::TransfMatrix> transformation_; //<! map of the transformations.
    std::unordered_map<std::string, Eigen::MatrixXd> manipulabilityJacobians_;
    std::unordered_map<std::string, double> manipulability_;
    Eigen::VectorXd q_total_; //<! vector of joints position.
    Eigen::VectorXd q_moving_; //<! vector of joints position.
    Eigen::VectorXd q_dot_moving_; //<! vector of joints velocity.
    Eigen::VectorXd q_ddot_moving_; //<! vector of joints acceleration.
    Eigen::VectorXd controlRef_; //<! control vector .
    std::vector<Eigen::TransfMatrix> baseTei_; //<! vector of transformation matrix from base to joint.   ??
    std::vector<Eigen::TransfMatrix> biTei_; //<! vector of transformation matrix from joint i-1 to joint i. ??
    Eigen::TransfMatrix baseTbi_; //<! transformation matrix from base to joint i.??
    Eigen::TransfMatrix Tz_; //!< transformation matrix for rotation or prismatic joint.
    Eigen::Vector3d base_ki_; //!<
    std::vector<Eigen::Vector6d> h_; //!<
    std::vector<Eigen::MatrixXd> dJdq_; //!< dJdq evaluated numerically.
    Eigen::MatrixXd djdqJpinv_; //!< djdq * jacobian pseudoinverse.
    Eigen::RotMatrix I3_; //!< identity matrix
    //Eigen::VectorXd ZeroQ_; //!< zero vector
    bool modelReadFromFile_; //!< boolean stating whether the model is read from file.
    std::string id_; //!< arm id.
};
}

#endif /* __ARMMODEL_H__ */
