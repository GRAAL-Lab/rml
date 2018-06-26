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
#include "VehicleModel.h"

namespace rml {

/**
 * @brief Exception to be thrown when the joint index out of bounds
 */
class RobotModelArmException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: Arm index out of bounds!!!";
    }
};

/**
 * @brief Exception to be thrown when vehicle is not present
 */
class RobotModelVehicleException : public std::exception {
    virtual const char* what() const throw()
    {
        return "[RobotModel] Error: No vehicle!!!";
    }
};

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
    std::map<std::string, int> jacobianMethodsMap_;

    Eigen::MatrixXd GetIsolatedArmJacobianForFrame(std::string ID) const;
    Eigen::Matrix6d GetIsolatedVehicleJacobianForFrame(std::string ID) const;

    Eigen::VectorXd ExtractVehicleSlice(const Eigen::VectorXd& y) const;
    Eigen::VectorXd ExtractArmSlice(const Eigen::VectorXd& y, std::string ID);

public:
    RobotModel();
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
    bool LoadVehicle(const std::shared_ptr<VehicleModel> vehicle);

    /**
     * @brief Loads an arm in the robot model
     *
     * Loads a arm in the robot model, there is no limit to the number of arms that can
     * be loaded. The ArmModel must be initialized before loading, otherwise is not
     * accepted. A <vehicle-to-base> transformation must be specified, which tells the
     * position of the arm's base frame with respect to the vehicle frame. If no vehicle
     * is meant to be loaded, it can be the identity matrix. The returning value is an
     * ID which identifies the loaded arm within the RobotModel class.
     *
     * @param arm		The arm model
     * @param vTb		The vehicle-to-base trasnformation matrix
     * @return			The arm ID
     */
    bool LoadArm(const std::shared_ptr<ArmModel> arm, const Eigen::TransfMatrix& vTb);

    /**
     * Checks that the given armIndex is within the allowed range (i.e. less or equal than
     * the number of loaded arms).
     * @param armIndex
     * @return
     */
    bool CheckArm(std::string armID) const throw(std::exception);
    bool CheckVehicle() const throw(std::exception);

    Eigen::MatrixXd GetJacobian_Frame(std::string ID);
    Eigen::MatrixXd GetJacobian_Identity(std::string ID);
    Eigen::MatrixXd GetJacobian_Manipulability(std::string ID);
    Eigen::MatrixXd GetJacobian_Vehicle(std::string ID);
    Eigen::MatrixXd GetJacobian(std::string jacobianID);

    Eigen::TransfMatrix GetTransformation(std::string transformationID);
    const std::shared_ptr<ArmModel> GetArm(std::string ID) const
    {
        return armsModel_.at(ID);
    }

    const std::shared_ptr<VehicleModel> GetVehicle() const
    {
        return vehicle_;
    }

    Eigen::VectorXd GetSystemPositionVector();
    Eigen::VectorXd GetSystemVelocityVector();

    void SetRobotControl(const Eigen::VectorXd& y);
    Eigen::VectorXd GetRobotControl();
};

} /* namespace rml */

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
