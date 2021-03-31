/**
 * \file
 *
 * \date 	Feb 27, 2018
 * \author 	Francesco Wanderlingh
 */
#ifndef INCLUDE_RML_ROBOTMODEL_H_
#define INCLUDE_RML_ROBOTMODEL_H_

#include <memory>
#include <vector>

#include "ArmModel.h"
#include "RMLDefines.h"
#include "RMLExceptions.h"
#include "VehicleModel.h"

namespace rml {

/**
 * \class RobotModel
 *
 * \brief Robot Model class, to evaluate either mobile or fixed manipulators total transformation matrices and jacobians.
 *
 * \details
 * This class provides a container for storing a multi-arm robot model. The robot could either be mobile one
 * (hence characterized by a moving body Frame) or a static one (hence with a fixed body frame).
 * Use the constructor a Jacobian for the body Frame is provided as input in the constructor).
 *
 * In order to add arm to the robot the method LoadArm() is provided.
 * It is in addition possible to add rigid body frames to the robot frames by using the method AttachedRigidBodyFrame().
 *

 * The robot model contains an internal unique representation of the whole system, hence all the information about the robot
 * model are given by organizing the vectors in an order a priori defined, where the moving body frame, if present, corresponds
 * to the first 6 position of the vector.
 * It is possible to obtain unified information about the robot position, velocity, acceleration and control by using the given
 * methods (e.g. PositionVector() to know the position of the whole system) but methods to obtain information of a single part
 * of the robot model are provided (e.g. PositionVector(partId) for the position of a part of the robot method, such function takes as
 * input the id of the desired part).
 *
 * The user must update the feedback for the arms and the mobile platform, if present, by using the methods PositionVector() = feedbackVector
 * if the feedback for the whole robot are in a unique vector or PositionVector(partID) = part_feedbackVecotr if the data are separated into vectors.
 * Similar methods are provided also with the purpose of setting the feedback in velocity and acceleration and control. For setting uniquely
 * the body frame position, the dedicated method PositionOnInertialFrame() is provided.

 * By exploiting the methods provided in the ArmModel and VehicleModel class, the robot model is able to provide the transformation matrices
 * and the jacobians for all the frames defined in the robot.
 * For this purpose the methods TransformationMatrix() and CartesianJacobian() are provided. These methods take as input the id of the frame
 * for which computing the matrices.
 *
 * The id must be provieded according the following logic:
 * - joint n-th: armID + rml::FrameID::Joint + "n"
 * - rigidBody attached on arm: armID + "_" + "rigidBodyID"
 * - rigid body attached on body frame: robotName + "_" + rigidBodyID
 *
 *
 *
 * It is worth noticing that the transformation matrices are expressed wrt to the world frame meanwhile
 * the jacobians are observed by the inertial frame and expressed wrt to the body frame.
 * In addition the method TransformationMatrix() is provided in order to compute the transformation matrices in between the two frames identified by the input ids.\n
 * Furthermore the two methods JointSpaceJacobian() and ManipulabilityJacobian() are provided. \n
 * The former takes as input the armID and returns the related joint space jacobian.
 * The latter takes as input a frame id and returns the related manipulability jacobian.
 * In order to obtain the manipulability value for a frame the method Manipulability() must be used\n
 * It is worth noticing that the jacobians given by the RobotModel takes into account the degrees of freedom of the whole robot
 * (hence of all the arms and of the moving platform, if present) in the aforementioned a priori defined order.
 * >
 *

 *
 */
class RobotModel {
public:
    /**
     * @brief Constructor for fixed robot model
     * @param[in] inertialF_T_bodyF transformation matrix from the world frame to body Frame.
     * @param[in] bodyFrameID id of the body Frame
     */
    RobotModel(Eigen::TransformationMatrix inertialF_T_bodyF, std::string bodyFrameID);
    /**
     * @brief Constructor for mobile robot model
     * @param[in] inertialF_T_bodyF transformation matrix from the world frame to body Frame.
     * @param[in] bodyFrameID id of the body Frame
     * @param[in] JBodyFrame jacobian of the bodyFrame in the body frame
     */
    RobotModel(Eigen::TransformationMatrix inertialF_T_bodyF, std::string bodyFrameID, Eigen::MatrixXd bodyF_JBodyFrame);
    /**
     * @brief Default deconstructor
     */
    virtual ~RobotModel();
    /**
     * @brief Returns the number of total degrees of fredoom of the system composed (both of the base if mobile and the arms).
     * @return The total number of DOFs (mobile base + arms)
     */
    auto Dof() const -> int { return DoF_; }
    /**
     * @brief Loads an arm in the robot model
     *
     * @details Loads a arm in the robot model, there is no limit to the number of arms that can
     * be loaded. The ArmModel must be initialized before loading, otherwise is not
     * accepted. A <robotFrame-to-base> transformation must be specified, which tells the
     * position of the arm's base frame with respect to the robotFrame.
     * @param[in] arm		The arm model
     * @param[in] bodyFrameToArm		The bodyFrame-to-Armbase trasnformation matrix
     * @return			True if the arm has been loaded false otherwise
     */
    bool LoadArm(const std::shared_ptr<ArmModel>& arm, const Eigen::TransformationMatrix& bodyFrameToArm) noexcept(false);
    /**
     * @brief Method to get the body Frame id.
     * @details Method to get the id for the body Frame, hence the frame common to all the arms.
     * @return body Frame id
     */
    auto BodyFrameID() const -> const std::string& { return bodyFrameID_; }
    /**
     * @brief Method to set the new body Frame position
     * @param[in] bodyFrame body Frame position wrt to the world frame
     */
    void PositionOnInertialFrame(const Eigen::TransformationMatrix& inertialF_T_bodyF);
    /**
     * @brief Method adding a rigid body frame attached to the input frame id .
     * @param[in] ID Id of the frame.
     * @param[in] TMat Transformation matrix of the frame.
     */
    void AttachRigidBodyFrame(const std::string& frameID, const std::string& attachedFrameID, const Eigen::TransformationMatrix& frameToAttachID_T_frameID) noexcept(false);
    /**
     * Checks whether the arm identified by the input id is present in the robot.
     * @param[in] armID the arm id
     * @return true if the arm is part of the robot, false otherwise
     */
    bool CheckArm(const std::string& armID) const;
    /**
     * @brief Method checking whether the robot model has a mobile body Frame .
     * @return true if the body Frame is mobile, false otherwise.
     */
    auto IsMobile() const -> bool { return isMobileRobot_; }

    /*
     * For CartesianJacobian, JointSpaceJacobian, ManipulabilityJacobian and TransformationMatrix the id must be provieded according the following logic:
     * - n-th joint: armID + rml::FrameID::Joint + "n"
     * - rigidBody attached on arm: armID + "_" + "rigidBodyID"
     * - rigid body attached on body frame: robotName + "_" + rigidBodyID
     */
    /**
     * @brief Method computing the jacobian observed by the world frame and projected on the body Frame of the input frameID
     * @param[in] ID frameID :
     * @return Jacobian observed by the world frame, projected on the body Frame.
     */
    Eigen::MatrixXd CartesianJacobian(const std::string& frameID) noexcept(false);
    /**
     * @brief Method computing the joint space jacobian of the input arm ID
     * @param[in] ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd JointSpaceJacobian(const std::string& armID) noexcept(false);
    /**
     * @brief Method computing the manipulability jacobian of the input arm ID
     * @param[in] ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd ManipulabilityJacobian(const std::string& frameID) noexcept(false);
    /**
     * @brief Method returning the manipulabity value for the jacobian related to the input frameID
     * @param[in] frameID
     * @return manipulability
     */
    double Manipulability(const std::string& frameID) noexcept(false);
    /**
     * @brief Method returning a transformation matrix of the robot model.\n
     * @details The methods returns a transformation matrix depending on the input string framID.\n
     * @param[in] transformationID
     * @return Transformation Matrix.
     */
    Eigen::TransformationMatrix TransformationMatrix(const std::string& frameID) noexcept(false);
    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.\n
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix jTk.
     */
    Eigen::TransformationMatrix TransformationMatrix(const std::string& frameID_j, const std::string& frameID_k);
    /**
     * @brief Method returning shared pointer to one of the loaded arm.
     * @param[in] ID arm id.
     * @return shared ptr to the arm.
     */
    const std::shared_ptr<ArmModel>& Arm(const std::string& ID) const noexcept(false);
    /**
     * @brief Method setting the position vector of the whole robot model.
     * @param[in] position vector.
     */
    void PositionVector(Eigen::VectorXd& position) noexcept(false);
    /**
     * @brief Method returning the position vector of the whole robot model.
     * @return position vector.
     */
    Eigen::VectorXd PositionVector();
    /**
     * @brief Method setting the velocity vector of the whole robot model.
     * @param[in] position vector.
     */
    void VelocityVector(const Eigen::VectorXd& velocity) noexcept(false);
    /**
     * @brief Method returning the velocity vector of the whole robot model.
     * @return velocity vector.
     */
    Eigen::VectorXd VelocityVector();
    /**
     * @brief Method returning the acceleration vector of the whole robot model.
     * @return acceleration vector.
     */
    Eigen::VectorXd AccelerationVector();
    /**
     * @brief Method setting the acceleration vector of the whole robot model.
     * @param[in] acceleration vector.
     */
    void AccelerationVector(const Eigen::VectorXd& acceleration) noexcept(false);
    /**
     * @brief Method setting the position vector for a robot part (joint postion for arm and cartesian position in case of mobile body Frame).
     * @param[in] partID .
     */
    void PositionVector(const std::string& partID, const Eigen::VectorXd& position) noexcept(false);
    /**
     * @brief Method returning the position of the input part (i.e. joint position for arm
     * and cartesian position in case if of mobile body Frame)
     * @param[in] partID
     * @return position vector, either cartesian position or joint position
     */
    Eigen::VectorXd PositionVector(const std::string& partID) noexcept(false);
    /**
     * @brief Method setting the velocity vector for a robot part (joint velocity for arm and cartesian velocity in case of mobile body Frame ).
     * @param[in] partID .
     */
    void VelocityVector(const std::string& partID, const Eigen::VectorXd& velocity) noexcept(false);
    /**
     * @brief GetVelocityVector Method returning the velocity of the input part (i.e. joints velocity for arm
     * and cartesian velocity in case of mobile body Frame )
     * @param[in] partID
     * @return velocity vector, either cartesian velocity or joints velocity.
     */
    Eigen::VectorXd VelocityVector(const std::string& partID) noexcept(false);
    /**
     * @brief Method setting the acceleration vector for a robot part (joints acceleration for arm and cartesian acceleration  in case of mobile body Frame ).
     * @param[in] partID .
     */
    void AccelerationVector(const std::string& partID, const Eigen::VectorXd& acceleration) noexcept(false);
    /**
     * @brief Method returning the position of the input part (i.e. joint acceleration for arm
     * and cartesian acceleration in case of mobile body Frame)
     * @param[in] partID
     * @return position vector, either cartesian position or joint position
     */
    Eigen::VectorXd AccelerationVector(const std::string& partID) noexcept(false);
    /**
     * @brief Method setting the control vector of the whole robot model.
     * @param[in] y control vector.
     */
    void ControlVector(const Eigen::VectorXd& y) noexcept(false);
    /**
     * @brief Method returning the control vector of the input part of robot model.
     * @param[in] partID either armModelID or robotFrameID
     * @return control vector.
     */
    Eigen::VectorXd ControlVector(const std::string& partID) noexcept(false);

protected:
    /**
     * @brief Method returning the isolated arm jacobian for the input frameID.
     * @param[in] ID frame id
     * @return  isolated jacobian
     */
    Eigen::MatrixXd ArmJacobian(const std::string& frameID) const noexcept(false);
    /**
     * @brief Method returning the isolated body Frame jacobian for the input frameID
     * @param[in] ID frameID
     * @return Jacobian
     */
    Eigen::Matrix6d BaseJacobian(const std::string& frameID) const;

    std::shared_ptr<VehicleModel> robotBase_; //!< the model of the body Frame;
    std::map<std::string, std::shared_ptr<ArmModel>> armsModel_; //!< map of the loaded map
    std::unordered_map<std::string, Eigen::TransformationMatrix> bodyFrameToArm_; //!< map of the transformation matrix from the arm bases to the body Frame
    std::string bodyFrameID_; //!< ID of the body Frame
    Eigen::TransformationMatrix inertialF_T_bodyF_; //!< body Frame transformaion matrix
    int DoF_; //!< total degrees of freedom of the robot
    bool isMobileRobot_; //!< boolean stating wheter the robot is a mobile one
};

} /* namespace rml */

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
