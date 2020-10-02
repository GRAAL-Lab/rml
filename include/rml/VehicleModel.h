/**
 * \file
 *
 * \date 	May 16, 2017
 * \author 	Francesco Wanderlingh
 */

#ifndef __CTRL_VEHICLEMODEL_H__
#define __CTRL_VEHICLEMODEL_H__

#include "RMLExceptions.h"
#include "Types.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace rml {
/**
 * @brief Vehicle Model base class
 * @details This class implements a base vehicle model class, it is used in the RobotModel class in order to implement the manipulators moving platform.
 */
class VehicleModel {
public:
    /**
	 * @brief Default constructor
     * @param[in] id VehicleModel id.
	 */
    VehicleModel(const std::string id);
    /**
	 * @brief Default destructor
	 */
    virtual ~VehicleModel();
    /**
	 * @brief Jacobian initialization
	 */
    void Jacobian(Eigen::Matrix6d J);
    /**
     * @brief Method returning the jacobian related to the frame id in input.
     * @param ID frame id.
     * @return transformation matrix.
     */
    Eigen::MatrixXd Jacobian(const std::string& ID) noexcept(false);
    /**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values 
     * @param[in] inertialF_T_vehicleF the postion of the vehicle w.r.t the inertial frame
     */
    void PositionOnInertialFrame(Eigen::TransformationMatrix inertialF_T_vehicleF);
    /**
     * @brief Get the inertialF_T_vehicleF
     * independently from the DOF of the vehicle.
     * @return inertialF_T_vehicleF the postion of the vehicle w.r.t the inertial frame
     */
    auto PositionOnInertialFrame() const -> const Eigen::TransformationMatrix& { return inertialF_T_vehicleF_; }
    /**
	 * @brief Set the base position
	 * The method updates the internal base velocity state. This method should be called before the evaluate methods in order to
     * have the updated value
	 */
    auto VelocityVector() -> Eigen::Vector6d& { return velocity_; }
    /**
     * @brief Get the complete 6D velocity, in the form of [x_dot y_dot z_dot wx wy wz],
     * independently from the Dof of the vehicle.
     * @return       The vehicle velocity
     */
    auto VelocityVector() const -> const Eigen::Vector6d& { return velocity_; }
    /**
     * @brief set the vehicle acceleration expressed wrt to the vehicle frame
     * @param fbkAcc vehicle acceleration
     */
    auto AccelerationVector() -> Eigen::Vector6d& { return acceleration_; }
    /**
     * @brief Method returning the acceleration wrt to the vehicle frame.
     * @return acceleration vector
     */
    auto AccelerationVector() const -> const Eigen::Vector6d& { return acceleration_; }
    /**
     * @brief Method adding a rigid body frame to the vehicle.
     * @param ID Id of the frame.
     * @param TMat Transformation matrix of the frame.
     */
    void AttachRigidBodyFrame(const std::string& frameID, const Eigen::TransformationMatrix& vehicleF_T_frameID);
    /**
     * @brief Method returning the transformation matrix related to the frame id in input.
     * @param ID frame id.
     * @return transformation matrix.
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
     * @brief Method returning whether the model is initialized.
     * @return true if the model is initialized, false otherwise.
     */
    auto IsModelInizialized() const -> bool { return modelInitialized_; }
    /**
     * @brief Method returning the vehicle control vector
     * @return vehicle control vector
     */
    auto ControlVector() const -> const Eigen::Vector6d& { return controlRef_; }
    /**
     * @brief Method setting the control vector.
     * @param controlRef control vector.
     */
    auto ControlVector() -> Eigen::Vector6d& { return controlRef_; }
    /**
     * @brief Method setting the id.
     * @param id vehicle id.
     */
    auto ID() -> std::string& { return id_; }
    /**
     * @brief Method returning the id
     * @return  vehicle id.
     */
    auto ID() const -> const std::string& { return id_; }

protected:
    bool modelInitialized_; //!< boolean stating whether the model is initialized
    bool isMapInitialized_; //!< boolean stating whether the transformation and jacobian maps are initialized
    std::unordered_map<std::string, Eigen::TransformationMatrix> rigidBodyFrames_; //!< map of the attached body frames
    Eigen::TransformationMatrix inertialF_T_vehicleF_; //!< vehicle position w.r.t the inertial frame
    Eigen::Vector6d velocity_; //!< vehicle velocity
    Eigen::Vector6d acceleration_; //!< vehicle acceleration
    Eigen::Vector6d controlRef_; //!< vehicle control vector
    std::unordered_map<std::string, Eigen::MatrixXd> jacobians_; //!< map of vehicle jacobians
    std::unordered_map<std::string, Eigen::TransformationMatrix> transformation_; //!< map of vehicle transformations
    Eigen::Matrix6d vJv_; //!< vehicle jacobians
    Eigen::RotationMatrix I3_; //!< identity matrix
    std::string id_; //!< vehicle id
};
}

#endif /* __CTRL_VEHICLEMODEL_H__ */
