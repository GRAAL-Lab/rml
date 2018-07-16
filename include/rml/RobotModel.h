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
 * including a series of model related functions. Using the LoadVehicle() and LoadArm() class
 * you can load one rml::VehicleModel and \a n rml::ArmModel(s).
 *
 */
class RobotModel {

    std::shared_ptr<VehicleModel> vehicle_;
    std::map<std::string, std::shared_ptr<ArmModel> > armsModel_;
    std::unordered_map<std::string, Eigen::TransfMatrix> vehicleToBase_;
    std::string vehicleID_;
    /**
     * @brief Method returning the isolated arm jacobian for the input framID.
     * @param[in] ID frame id
     * @return  isolated jacobian
     */
    Eigen::MatrixXd GetIsolatedArmJacobianForFrame(const std::string& frameID) const throw(std::exception);
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
    RobotModel();
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
     * @brief Loads a vehicle in the robot model
     *
     * Loads a vehicle in the robot model, only one vehicle is allowed. The VehicleModel
     * must be initialized before loading, otherwise is not accepted.
     *
     * @param vehicle		The vehicle model
     * @return				true on success, false otherwise
     */
    bool LoadVehicle(const std::shared_ptr<VehicleModel> vehicle) throw(std::exception);

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
     * @param vTb		The vehicle-to-base trasnformation matrix
     * @return			True if the arm has been loaded false otherwise
     */
    bool LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb) throw(std::exception);

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
    Eigen::MatrixXd GetCartesianJacobian(const std::string& frameID, JacobianObserver jacobianObserver) throw(std::exception);
    /**
     * @brief Method computing the identity jacobian of the input arm ID
     * @param ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd GetJointSpaceJacobian(const std::string& armID) throw(std::exception);
    /**
     * @brief Method computing the manipulability jacobian of the input arm ID
     * @param ID armID
     * @return Jacobian
     */
    Eigen::MatrixXd GetManipulabilityJacobian(const std::string& armID) throw(std::exception);
    /**
     * @brief Method returing a transformation matrix of the robot model.\n
     * @details The methods returns a transformation matrix depending on the input string framID.\n
     * @param transformationID
     * @return Transformation Matrix.
     */
    Eigen::TransfMatrix GetTransformation(const std::string& frameID) throw(std::exception);
    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.\n
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix aTb.
     */
    Eigen::TransfMatrix GetTransformationFrames(const std::string& frameID_j,const std::string& frameID_k);
    /**
     * @brief Method returning shared pointer to one of the loaded arm.
     * @param ID arm id.
     * @return shared ptr to the arm.
     */
    const std::shared_ptr<ArmModel> GetArm(const std::string ID) const throw(std::exception);
    /**
     * @brief Method returning shared pointer to one of the loaded vehicle.
     * @return shared ptr to the vehicle.
     */
    const std::shared_ptr<VehicleModel> GetVehicle() const throw(std::exception);

    /**
     * @brief Method returning the position vector of the whole robot model.
     * @return position vector.
     */
    Eigen::VectorXd GetSystemPositionVector();
    /**
     * @brief Method returning the velocity vector of the whole robot model.
     * @return velocity vector.
     */
    Eigen::VectorXd GetSystemVelocityVector();
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
    Eigen::VectorXd GetRobotControl(const std::string partID) throw(std::exception);
};

} /* namespace rml */

///**
// * @brief Method returning a jacobian of the robot model .\n
// * @details The method calls protected methods of the class depending on the input string. \n
// * The following policy is used:\n
// * ArmModel Frame only vehicle contribution (0 for arm ): “Frame_“+”VehicleID”+arm_frame_id.
// * ArmModel Frame Jacobian both vehicle and arm contribution: : "Frame_" + arm_frame_id.
// * Joints Identity Jacobian: "Identity_" + arm ID.
// * Vehicle Body Frame (0 for the arm ): “Vehicle_” + Vehicle_frame_id.
// * Manipulability : “Manipulability_“+ armID. * *
// * @param jacobianID
// * @return  Jacobian
// */
//Eigen::MatrixXd GetJacobian(std::string jacobianID) throw(std::exception);

///**
// * @brief Method computing the vehicle jacobian and rigid body attached to the vehicle jacobian depending on the input ID
// * @param ID ID
// * @return Jacobian
// */
//Eigen::MatrixXd GetJacobian_Vehicle(std::string ID);

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
