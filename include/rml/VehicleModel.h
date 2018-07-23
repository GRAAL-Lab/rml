/**
 * \file
 *
 * \date 	May 16, 2017
 * \author 	Francesco Wanderlingh
 */

#ifndef __CTRL_VEHICLEMODEL_H__
#define __CTRL_VEHICLEMODEL_H__

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

#include "Types.h"

namespace rml {

/**
 * @brief Vehicle Model base class
 *
 * @details This class implements a base vehicle model class.
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
    void SetJacobian(Eigen::Matrix6d vehicleJacobian);

    /**
	 * @brief Set the base position
	 * The method updates the internal base position state. This method should be called before the evaluate methods in order to
	 * have the updated values 
	 * @param[in] fbkPos the base position vector in the form of [r p y x y z]
	 */
    void SetPositionOnInertial(const Eigen::Vector6d& fbkPos);

    /**
	 * @brief Set the base position
	 * The method updates the internal base velocity state. This method should be called before the evaluate methods in order to
	 * have the updated values
	 * @param[in] fbkVel the base velocity vector in the form of [wx wy wz x y z]
	 */
    void SetVelocityOnVehicle(const Eigen::Vector6d& fbkVel);

    /**
     * @brief set the vehicle acceleration expressed wrt to the vehicle frame
     * @param fbkAcc vehicle acceleration
     */
    void SetAccelerationOnVehicle(const Eigen::Vector6d& fbkAcc);

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
    Eigen::TransfMatrix GetTransformation(const std::string frameID) throw(std::exception);

    /**
     * @brief Method returing a transformation matrix from frameID_j to frameID_k, i.e. jTk.\n
     * @param[in] frameID_j first frame;
     * @param[in] framID_k second frame;
     * @return Transformation Matrix jTk.
     */
    Eigen::TransfMatrix GetTransformationFrames(const std::string& frameID_j, const std::string& frameID_k);
    /**
     * @brief Method returning the jacobian related to the frame id in input.
     * @param ID frame id.
     * @return transformation matrix.
     */
    Eigen::MatrixXd GetJacobian(const std::string ID) throw(std::exception);

    /**
     * @brief Method returning whether the model is initialized.
     * @return true if the model is initialized, false otherwise.
     */
    bool IsModelInitialized() const;
    /**
     * @brief Method returning the vehicle control vector
     * @return vehicle control vector
     */
    const Eigen::Vector6d& GetControlVector() const;
    /**
     * @brief Method setting the control vector.
     * @param controlRef control vector.
     */
    void SetControlVector(const Eigen::Vector6d& controlRef);
    /**
     * @brief Method setting the id.
     * @param id vehicle id.
     */
    void SetID(std::string id);
    /**
     * @brief Method returning the id
     * @return  vehicle id.
     */
    std::string GetID();
    /**
     * @brief Get the complete 6D position, in the form of [r p y x y z],
     * independently from the DOF of the vehicle.
     *
     * @return      The vehicle position
     */
    const Eigen::Vector6d& GetPositionOnInertial();
    /**
     * @brief Get the complete 6D velocity, in the form of [wx wy wz x y z],
     * independently from the Dof of the vehicle.
     *
     * @return       The vehicle velocity
     */
    const Eigen::Vector6d& GetVelocityOnVehicle();
    /**
     * @brief Method returning the acceleration wrt to the vehicle frame.
     * @return acceleration vector
     */
    const Eigen::Vector6d& GetAccelerationOnVehicle();

protected:
    bool modelInitialized_; //!< boolean stating whether the model is initialized.
    bool isMapInitialized_; //!< boolean stating whether the transformation and jacobian maps are initialized.
    std::unordered_map<std::string, Eigen::TransfMatrix> rigidBodyFrames_; //!< map of the attached body frames.
    Eigen::Vector6d fbkPosition_; //!< vehicle position.
    Eigen::Vector6d velocityOnVehicle_; //!< vehicle velocity.
    Eigen::Vector6d accelerationOnVehicle_; //!< vehicle acceleration.
    Eigen::Vector6d controlRef_; //!< vehicle control vector.
    std::unordered_map<std::string, Eigen::MatrixXd> jacobians_; //!< map of vehicle jacobians.
    std::unordered_map<std::string, Eigen::TransfMatrix> transformation_; //!< map of vehicle transformations.
    Eigen::Matrix6d vJv_; //!< vehicle jacobians.
    Eigen::RotMatrix I3_; //!< identity matrix.
    std::string id_; //!< vehicle id.
};
}

#endif /* __CTRL_VEHICLEMODEL_H__ */
/**
 * @brief Method returning the world to vehicle transf matrix.
 * @return wTv
 */
// const Eigen::TransfMatrix GetwTv();
/**
 * @brief Method returning the vehicle jacobian.
 * @return vJv
 */
//const Eigen::Matrix6d& GetvJv() const throw(std::exception);
/**
 * @brief Method returning the attached body frame.
 * @param ID frame ID.
 * @return transformation matrix.
 */
//Eigen::TransfMatrix GetAttachedBodyTransf(const std::string& ID) throw(std::exception);
/**
 * @brief Method returning the attached body frame jacobian.
 * @param ID frame ID.
 * @return jacobian matrix.
 */
//Eigen::MatrixXd GetAttachedBodyJacobian(const std::string ID);
/**
 * @brief Method returning the attached body transformation matrix.
 * @param ID frame ID.
 * @return  transformation matrix.
 */
//Eigen::TransfMatrix GetCurrentAttachedBodyTransf(const std::string ID);
