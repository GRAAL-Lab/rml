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
 * \brief Robot Model class, to evaluate mobile manipulators total transformation
 * matrices and jacobians.
 *
 * \details This class provides a container for storing a multi-arm mobile manipulator model,
 * including a series of model related functions. The robot could be either be mobile one (hence characterized by a\n
 * moving robot frame) or a static one (hence with a constant robot frame) depending on the constructor used (hence whether a Jacobian\n
 * for the robot frame is provided).\n
 * In order to add arm in the robot the method LoadArm() is provided.\n
 * It is in addition possible to add rigid body frames to the robot frame by using the method SetRigidBodyFrame.
 *
 */
class RobotModel {

    std::shared_ptr<VehicleModel> vehicle_;
    std::map<std::string, std::shared_ptr<ArmModel> > armsModel_;
    std::unordered_map<std::string, Eigen::TransfMatrix> robotframeToArm_;
    std::string robotFrameID_;
    Eigen::TransfMatrix robotFrame_;
    int DoF_{ 0 };
    bool isVehicle_;
    /**
     * @brief Method returning the isolated arm jacobian for the input framID.
     * @param[in] ID frame id
     * @return  isolated jacobian
     */
    Eigen::MatrixXd GetIsolatedArmJacobianForFrame(const std::string& frameID) const throw(ExceptionWithHow);
    /**
     * @brief Method reeturning the isolated vehicle jacobian for the input framID
     * @param[in] ID frameID
     * @return Jacobian
     */
    Eigen::Matrix6d GetIsolatedVehicleJacobianForFrame(const std::string& frameID) const;

public:
    /**
     * @brief Default constructor
     */
    RobotModel(Eigen::TransfMatrix robotFrame, std::string frameID);
    /**
     * @brief Default constructor
     */
    RobotModel(Eigen::TransfMatrix robotFrame, std::string frameID, Eigen::MatrixXd JRobotFrame);
    /**
     * @brief Default deconstructor
     */
    virtual ~RobotModel();

    /**
     * @brief Returns the number of total degrees of fredoom of the system composed of
     * a vehicle plus \p n arms.
     * @return	The total number of DOFs (vehicle + arms)
     */
    int GetTotalDOFs();
    /**
     * @brief Loads an arm in the robot model
     *
     * Loads a arm in the robot model, there is no limit to the number of arms that can
     * be loaded. The ArmModel must be initialized before loading, otherwise is not
     * accepted. A <vehicle-to-base> transformation must be specified, which tells the
     * position of the arm's base frame with respect to the vehicle frame. If no vehicle
     * is meant to be loaded, it identifies the transformation in between the robot base and the robot
     * common frame.
     * @param arm		The arm model
     * @param robotframeToArm		The vehicle-to-base trasnformation matrix
     * @return			True if the arm has been loaded false otherwise
     */
    bool LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& robotframeToArm) throw(ExceptionWithHow);
    /**
     * @brief GetRobotFrameID method to get the robot frame id.
     * @details Method to get the id for the robot frame, hence the frame common to all the arms.
     * If the vehicle is loaded, it is the vehicle frame itself.
     * By default it is eaul to identity.
     * @return robot frame id
     */
    std::string GetRobotFrameID();
    /**
     * @brief SetRobotFramePosition method to set the new robot frame position
     * @param robotFrame robot frame position wrt to the world frame
     */
    void SetRobotFramePosition(Eigen::TransfMatrix robotFrame);
    /**
     * @brief Method adding a rigid body frame to the vehicle.
     * @param ID Id of the frame.
     * @param TMat Transformation matrix of the frame.
     */
    void SetRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat, const std::string frameID) throw(ExceptionWithHow);

    /**
     * Checks that the given armIndex is within the allowed range (i.e. less or equal than
     * the number of loaded arms).
     * @param armID the arm index
     * @return
     */
    bool CheckArm(const std::string armID) const;
    /**
     * @brief Method checking whether a vehicle has been loaded
     * @return true if the vehicle has been loaded, false otherwise.
     */
    bool CheckVehicle() const;

    /**
     * @brief Method computing the jacobin of the input frameID
     * @param ID frameID
     * @return Jacobian
     */
    Eigen::MatrixXd GetCartesianJacobian(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief Method computing the identity jacobian of the input arm ID
     * @param ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd GetJointSpaceJacobian(const std::string& armID) throw(ExceptionWithHow);
    /**
     * @brief Method computing the manipulability jacobian of the input arm ID
     * @param ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd GetManipulabilityJacobian(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief GetManipulability method returning the manipulabity value for the jacobian related to the input frameID
     * @param frameID
     * @return manipulability
     */
    double GetManipulability(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief Method returing a transformation matrix of the robot model.\n
     * @details The methods returns a transformation matrix depending on the input string framID.\n
     * @param transformationID
     * @return Transformation Matrix.
     */
    Eigen::TransfMatrix GetTransformation(const std::string& frameID) throw(ExceptionWithHow);
    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.\n
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix aTb.
     */
    Eigen::TransfMatrix GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k);
    /**
     * @brief Method returning shared pointer to one of the loaded arm.
     * @param ID arm id.
     * @return shared ptr to the arm.
     */
    const std::shared_ptr<ArmModel> GetArm(const std::string ID) const throw(ExceptionWithHow);

    /**
     * @brief Method returning the position vector of the whole robot model.
     * @return position vector.
     */
    Eigen::VectorXd GetSystemPositionVector();

    /**
     * @brief Method setting the position vector of the whole robot model.
     * @param[in] position vector.
     */

    void SetSystemPositionVector(Eigen::VectorXd position) throw(std::exception);

    /**
     * @brief Method setting the position vector of the whole robot model.
     * @param[in] position vector.
     */

    void SetSystemVelocityVector(Eigen::VectorXd velocity) throw(std::exception);

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
     * @brief Method returning the velocity vector of the whole robot model.
     * @return velocity vector.
     */
    Eigen::VectorXd GetSystemVelocityVector();
    /**
     * @brief Method setting the position vector for a robot part (joint postion for arm and cartesian position for vehicle).
     * @param[in] partID .
     */

    void SetPositionVector(std::string partID, Eigen::VectorXd position) throw(ExceptionWithHow);

    /**
     * @brief GetPositionVector Method returning the position of the input part (i.e. joint position for arm
     * and cartesian position for vehicle )
     * @param partID
     * @return position vector, either cartesian position or joint position
     */
    Eigen::VectorXd GetPositionVector(std::string partID) throw(ExceptionWithHow);
    /**
     * @brief Method setting the velocity vector for a robot part (joint velocity for arm and cartesian velocity for vehicle).
     * @param[in] partID .
     */
    void SetVelocityVector(std::string partID, Eigen::VectorXd velocity) throw(ExceptionWithHow);
    /**
     * @brief GetVelocityVector Method returning the velocity of the input part (i.e. joints velocity for arm
     * and cartesian velocity for vehicle )
     * @param partID
     * @return velocity vector, either cartesian velocity or joints velocity.
     */
    Eigen::VectorXd GetVelocityVector(std::string partID) throw(ExceptionWithHow);
    /**
     * @brief Method setting the acceleration vector for a robot part (joints acceleration for arm and cartesian acceleration for vehicle).
     * @param[in] partID .
     */

    void SetAccelerationVector(std::string partID, Eigen::VectorXd acceleration) throw(ExceptionWithHow);
    /**
     * @brief GetAccelerationVector Method returning the position of the input part (i.e. joint acceleration for arm
     * and cartesian acceleration for vehicle )
     * @param partID
     * @return position vector, either cartesian position or joint position
     */

    Eigen::VectorXd GetAccelerationVector(std::string partID) throw(ExceptionWithHow);
    /**
     * @brief Method setting the control vector of the whole robot model.
     * @param[in] y  control vector.
     */
    void SetRobotControl(const Eigen::VectorXd& y) throw(std::exception);
    /**
     * @brief Method returning the control vector of a part of  robot model.
     * @param[in] partID either armModelID or vehicleModelID
     * @return control vector.
     */
    Eigen::VectorXd GetRobotControl(const std::string partID) throw(ExceptionWithHow);
};

} /* namespace rml */

/**
 * @brief Method returning shared pointer to one of the loaded vehicle.
 * @return shared ptr to the vehicle.
 */
//const std::shared_ptr<VehicleModel> GetVehicle() const throw(ExceptionWithHow);


#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
