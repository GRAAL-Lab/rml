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
 * \brief Robot Model class, to evaluate either mobile or fixed manipulators total transformation
 * matrices and jacobians.
 *
 * \details This class provides a container for storing a multi-arm mobile or fixed manipulator model,
 * including a series of model related functions. The robot could either be mobile one (hence characterized by a\n
 * moving body Frame) or a static one (hence with a constant body Frame) depending on the constructor used (hence whether a Jacobian\n
 * for the body Frame is provided).\n
 * In order to add arm tp the robot the method LoadArm() is provided.\n
 * It is in addition possible to add rigid body frames to the each body Frame by using the method SetRigidBodyFrame.
 *
 */
class RobotModel {
public:
    /**
     * @brief Constructor for fixed robot model
     * @param[in] bodyFrame transformation matrix from the world frame to body Frame.
     * @param[in] frameID id of the body Frame
     */
    RobotModel(Eigen::TransfMatrix bodyFrame, std::string frameID);
    /**
     * @brief Constructor for mobile robot model
     * @param[in] bodyFrame transformation matrix from the world frame to body Frame.
     * @param[in] frameID id of the body Frame
     * @param[in] JBodyFrame jacobian of the bodyFrame
     */
    RobotModel(Eigen::TransfMatrix bodyFrame, std::string frameID, Eigen::MatrixXd JBodyFrame);
    /**
     * @brief Default deconstructor
     */
    virtual ~RobotModel();

    /**
     * @brief Returns the number of total degrees of fredoom of the system composed (both of the base if mobile and the arms).
     * @return The total number of DOFs (mobile base + arms)
     */
    int GetTotalDOFs();
    /**
     * @brief Loads an arm in the robot model
     *
     * Loads a arm in the robot model, there is no limit to the number of arms that can
     * be loaded. The ArmModel must be initialized before loading, otherwise is not
     * accepted. A <robotFrame-to-base> transformation must be specified, which tells the
     * position of the arm's base frame with respect to the robotFrame.
     * @param[in] arm		The arm model
     * @param[in] bodyFrameToArm		The bodyFrame-to-Armbase trasnformation matrix
     * @return			True if the arm has been loaded false otherwise
     */
    bool LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& bodyFrameToArm) throw(ExceptionWithHow);
    /**
     * @brief GetBodyFrameID method to get the body Frame id.
     * @details Method to get the id for the body Frame, hence the frame common to all the arms.
     * @return body Frame id
     */
    std::string GetBodyFrameID();
    /**
     * @brief SetBodyFramePosition method to set the new body Frame position
     * @param[in] bodyFrame body Frame position wrt to the world frame
     */
    void SetBodyFramePosition(Eigen::TransfMatrix bodyFrame);
    /**
     * @brief Method adding a rigid body frame attached to the input frame id .
     * @param[in] ID Id of the frame.
     * @param[in] TMat Transformation matrix of the frame.
     */
    void SetAttachedRigidBodyFrame(const std::string frameID, const Eigen::TransfMatrix TMat, const std::string attachedFrameID) throw(ExceptionWithHow);

    /**
     * Checks whether the arm identified by the input id is present in the robot.
     * @param[in] armID the arm id
     * @return true if the arm is part of the robot, false otherwise
     */
    bool CheckArm(const std::string armID) const;
    /**
     * @brief Method checking whether the robot model has a mobile body Frame .
     * @return true if the body Frame is mobile, false otherwise.
     */
    bool IsMobile() const;

    /**
     * @brief Method computing the jacobian observed by the world frame and projected on the body Frame of the input frameID
     * @param[in] ID frameID
     * @return Jacobian observed by the world frame, projected on the body Frame.
     */
    Eigen::MatrixXd GetCartesianJacobian(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief Method computing the joint space jacobian of the input arm ID
     * @param[in] ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd GetJointSpaceJacobian(const std::string& armID) throw(ExceptionWithHow);
    /**
     * @brief Method computing the manipulability jacobian of the input arm ID
     * @param[in] ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd GetManipulabilityJacobian(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief GetManipulability method returning the manipulabity value for the jacobian related to the input frameID
     * @param[in] frameID
     * @return manipulability
     */
    double GetManipulability(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief Method returning a transformation matrix of the robot model.\n
     * @details The methods returns a transformation matrix depending on the input string framID.\n
     * @param[in] transformationID
     * @return Transformation Matrix.
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
     * @brief Method returning shared pointer to one of the loaded arm.
     * @param[in] ID arm id.
     * @return shared ptr to the arm.
     */
    const std::shared_ptr<ArmModel> GetArm(const std::string ID) const throw(ExceptionWithHow);

    /**
     * @brief Method setting the position vector of the whole robot model.
     * @param[in] position vector.
     */

    void SetSystemPositionVector(Eigen::VectorXd position) throw(std::exception);

    /**
     * @brief Method returning the position vector of the whole robot model.
     * @return position vector.
     */
    Eigen::VectorXd GetSystemPositionVector();

    /**
     * @brief Method setting the velocity vector of the whole robot model.
     * @param[in] position vector.
     */

    void SetSystemVelocityVector(Eigen::VectorXd velocity) throw(std::exception);

    /**
     * @brief Method returning the velocity vector of the whole robot model.
     * @return velocity vector.
     */
    Eigen::VectorXd GetSystemVelocityVector();

    /**
     * @brief Method returning the acceleration vector of the whole robot model.
     * @return acceleration vector.
     */

    Eigen::VectorXd GetSystemAccelerationVector();

    /**
     * @brief Method setting the acceleration vector of the whole robot model.
     * @param[in] acceleration vector.
     */

    void SetSystemAccelerationVector(Eigen::VectorXd acceleration) throw(std::exception);

    /**
     * @brief Method setting the position vector for a robot part (joint postion for arm and cartesian position in case of mobile body Frame).
     * @param[in] partID .
     */

    void SetPositionVector(std::string partID, Eigen::VectorXd position) throw(ExceptionWithHow);

    /**
     * @brief GetPositionVector Method returning the position of the input part (i.e. joint position for arm
     * and cartesian position in case if of mobile body Frame)
     * @param[in] partID
     * @return position vector, either cartesian position or joint position
     */
    Eigen::VectorXd GetPositionVector(std::string partID) throw(ExceptionWithHow);
    /**
     * @brief Method setting the velocity vector for a robot part (joint velocity for arm and cartesian velocity in case of mobile body Frame ).
     * @param[in] partID .
     */
    void SetVelocityVector(std::string partID, Eigen::VectorXd velocity) throw(ExceptionWithHow);
    /**
     * @brief GetVelocityVector Method returning the velocity of the input part (i.e. joints velocity for arm
     * and cartesian velocity in case of mobile body Frame )
     * @param[in] partID
     * @return velocity vector, either cartesian velocity or joints velocity.
     */
    Eigen::VectorXd GetVelocityVector(std::string partID) throw(ExceptionWithHow);
    /**
     * @brief Method setting the acceleration vector for a robot part (joints acceleration for arm and cartesian acceleration  in case of mobile body Frame ).
     * @param[in] partID .
     */

    void SetAccelerationVector(std::string partID, Eigen::VectorXd acceleration) throw(ExceptionWithHow);
    /**
     * @brief GetAccelerationVector Method returning the position of the input part (i.e. joint acceleration for arm
     * and cartesian acceleration in case of mobile body Frame)
     * @param[in] partID
     * @return position vector, either cartesian position or joint position
     */

    Eigen::VectorXd GetAccelerationVector(std::string partID) throw(ExceptionWithHow);
    /**
     * @brief Method setting the control vector of the whole robot model.
     * @param[in] y control vector.
     */
    void SetRobotControl(const Eigen::VectorXd& y) throw(std::exception);
    /**
     * @brief Method returning the control vector of the input part of robot model.
     * @param[in] partID either armModelID or robotFrameID
     * @return control vector.
     */
    Eigen::VectorXd GetRobotControl(const std::string partID) throw(ExceptionWithHow);

protected:
    /**
     * @brief Method returning the isolated arm jacobian for the input frameID.
     * @param[in] ID frame id
     * @return  isolated jacobian
     */
    Eigen::MatrixXd GetIsolatedArmJacobianForFrame(const std::string& frameID) const throw(ExceptionWithHow);
    /**
     * @brief Method returning the isolated body Frame jacobian for the input frameID
     * @param[in] ID frameID
     * @return Jacobian
     */
    Eigen::Matrix6d GetIsolatedRobotFrameJacobianForFrame(const std::string& frameID) const;

    std::shared_ptr<VehicleModel> robotBase_; //!< the model of the body Frame;
    std::map<std::string, std::shared_ptr<ArmModel> > armsModel_; //!< map of the loaded map
    std::unordered_map<std::string, Eigen::TransfMatrix> bodyFrameToArm_; //!< map of the transformation matrix from the arm bases to the body Frame
    std::string bodyFrameID_; //!< ID of the body Frame
    Eigen::TransfMatrix bodyFrame_; //!< body Frame transformaion matrix
    int DoF_{ 0 }; //!< total degrees of freedom of the robot
    bool isMobileRobot_; //!< boolean stating wheter the robot is a mobile one.
};

} /* namespace rml */

/**
 * @brief Method returning shared pointer to one of the loaded vehicle.
 * @return shared ptr to the vehicle.
 */
//const std::shared_ptr<VehicleModel> GetVehicle() const throw(ExceptionWithHow);

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
