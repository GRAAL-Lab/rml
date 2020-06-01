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
 *
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
    void Jacobian(Eigen::Matrix6d vehicleJacobian);

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
	 * @param[in] fbkPos the base position vector in the form of [r p y x y z]
	 */
    void PositionOnInertial(Eigen::Vector6d fbkPos);

    /**
     * @brief Get the complete 6D position, in the form of [r p y x y z],
     * independently from the DOF of the vehicle.
     *
     * @return      The vehicle position
     */
    auto PositionOnInertial() const -> const Eigen::Vector6d& { return fbkPosition_; }

    /**
	 * @brief Set the base position
	 * The method updates the internal base velocity state. This method should be called before the evaluate methods in order to
     * have the updated value
	 */
    auto VelocityOnVehicle() -> Eigen::Vector6d& { return velocityOnVehicle_; }

    /**
     * @brief Get the complete 6D velocity, in the form of [wx wy wz x y z],
     * independently from the Dof of the vehicle.
     * @return       The vehicle velocity
     */
    auto VelocityOnVehicle() const -> const Eigen::Vector6d& { return velocityOnVehicle_; }

    /**
     * @brief set the vehicle acceleration expressed wrt to the vehicle frame
     * @param fbkAcc vehicle acceleration
     */
    auto AccelerationOnVehicle() -> Eigen::Vector6d& { return accelerationOnVehicle_; }

    /**
     * @brief Method returning the acceleration wrt to the vehicle frame.
     * @return acceleration vector
     */
    auto AccelerationOnVehicle() const -> const Eigen::Vector6d& { return accelerationOnVehicle_; }

    /**
     * @brief Method adding a rigid body frame to the vehicle.
     * @param ID Id of the frame.
     * @param TMat Transformation matrix of the frame.
     */
    void SetRigidBodyFrame(const std::string ID, const Eigen::TransfMatrix TMat);

    /**
     * @brief Method returning the transformation matrix related to the frame id in input.
     * @param ID frame id.
     * @return transformation matrix.
     */
    Eigen::TransfMatrix GetTransformation(const std::string& frameID) noexcept(false);

    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix jTk.
     */
    Eigen::TransfMatrix GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k);

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
    std::unordered_map<std::string, Eigen::TransfMatrix> rigidBodyFrames_; //!< map of the attached body frames
    Eigen::Vector6d fbkPosition_; //!< vehicle position
    Eigen::Vector6d velocityOnVehicle_; //!< vehicle velocity
    Eigen::Vector6d accelerationOnVehicle_; //!< vehicle acceleration
    Eigen::Vector6d controlRef_; //!< vehicle control vector
    std::unordered_map<std::string, Eigen::MatrixXd> jacobians_; //!< map of vehicle jacobians
    std::unordered_map<std::string, Eigen::TransfMatrix> transformation_; //!< map of vehicle transformations
    Eigen::Matrix6d vJv_; //!< vehicle jacobians
    Eigen::RotMatrix I3_; //!< identity matrix
    std::string id_; //!< vehicle id
};
}

#endif /* __CTRL_VEHICLEMODEL_H__ */
