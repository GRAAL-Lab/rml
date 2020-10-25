/**
 * \file
 *
 * \date 	Mar 12, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_NEWTONEULER_H_
#define INCLUDE_RML_NEWTONEULER_H_

#include <memory>

#include "RobotModel.h"

namespace rml {

const double STD_GRAVITY = 9.80665;

/**
 * \class NewtonEuler
 *
 * \brief Implementation of the Newton-Euler equation for rigid-body chains
 *
 * \details This class makes use of rml::RobotModel to find the forces and moments
 * acting on the manipulator. Some facilites equations are provided to use the class
 * in order to simulated dynamic systems. The nomenclature used for the manipulators
 * dynamic equation is the following:
 *
 * \f$ A(q)\ddot{q} + B(q_,\dot{q})\dot{q} + C(q) = m + \hat{m} \f$
 *
 * where \f$ \hat{m} \f$ are the external forces.
 *
 * I can evaluate the various A, B, C matrix by imposing q_dot, q_ddot
 * equal to zero. After, I can compute q_ddot by inverting the above
 * mentioned formula. Defined \f$ \tilde{m} = B\dot{q} + C \f$, we can write:
 *
 *  \f$ \ddot{q} = A^{-1}[ m + \hat{m} - \tilde{m}] \f$
 *
 *  The newton euler algorithm can be used to evaulate the \f$ A \f$ matrix and the
 *  \f$ \tilde{m} \f$ , and the functions InertiaMatrix() and CoriolisGravityExternalForcesEffect() provide this
 *  functionalities.
 *
 *  \note Don't forget to set the dynamic properties of the ArmModel links before
 *  using the algorithms in this class, by calling RobotLink::SetDynamicProperties().
 */
class NewtonEuler {
    unsigned int numJoints_;

    //std::shared_ptr<RobotModel> model_;
    std::shared_ptr<ArmModel> armModel_;
    std::shared_ptr<RobotModel> robotModel_;

    std::vector<Eigen::RotationMatrix> R_;
    std::vector<Eigen::Vector3d> rhoVec_; // Distance between joint "i" and "i+1"
    Eigen::VectorXd q_, q_dot_, q_ddot_, q_ddot_Ai_, qZeroVec_;
    Eigen::VectorXd a_column;
    std::vector<Eigen::Vector3d> omega_, omega_dot_, c_dot_, c_ddot_;
    Eigen::Vector3d temp1_, temp2_, temp3_, temp4_, omega_prev_;
    Eigen::Vector3d gravity_, zeroVect3_;
    std::vector<Eigen::Vector3d> n_, f_;
    std::vector<Eigen::Vector3d> r_qmc_, r_qpc_;

    /**
	 * The interaction forces and moments associated with the link "i" are relative to the one between
	 * "i" and "i-1". While the "self" forces are the one relative to the acceleration of the CoM_i.
	 */
    std::vector<Eigen::Vector3d> self_f_, self_n_;
    std::vector<Eigen::Vector3d> interaction_f_, interaction_n_;

    std::vector<RobotLink> links_;

    std::string toolID_;

    void Init();
    void AddDummyBaseAndEE();
    void InterMom2Torque(Eigen::VectorXd& torques) const;

public:
    //NewtonEuler();
    NewtonEuler(std::shared_ptr<RobotModel>& model, std::string& armID, std::string& toolID);
    virtual ~NewtonEuler();

    void EvaluateAlgorithmStep(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, const Eigen::VectorXd& q_ddot, const Eigen::Vector3d& gravity, Eigen::VectorXd& torques);

    /**
     * set the gravity vector
     */
    auto Gravityvector() -> Eigen::Vector3d& { return gravity_; }

    /**
     * @return the inertia matrix matrix
	 */
    Eigen::MatrixXd InertiaMatrix();

    /**
     * @return matrix of the effect of gravity in the joint space
     */
    Eigen::VectorXd GravityEffect();

    /**
	 * @return \f$ \tilde{m} = B\dot{q} + C \f$
	 */
    Eigen::VectorXd CoriolisGravityExternalForcesEffect();

    void PrintVars() const;
};

} /* namespace rml */

#endif /* INCLUDE_RML_NEWTONEULER_H_ */
