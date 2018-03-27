/*
 * NewtonEuler.h
 *
 *  Created on: Mar 12, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_NEWTONEULER_H_
#define INCLUDE_RML_NEWTONEULER_H_

#include <memory>

#include "RobotModel.h"

namespace rml
{

const double STD_GRAVITY = 9.80665;

class NewtonEuler
{
	int numJoints_;

	//std::shared_ptr<RobotModel> model_;
	std::shared_ptr<ArmModel> armModel_;
	std::shared_ptr<VehicleModel> vehicle_;

	std::vector<Eigen::RotMatrix> R_;
	std::vector<Eigen::Vector3d> rhoVec_;		// Distance between joint "i" and "i+1"
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

	void Init();
	void AddDummyBaseAndEE();
	void InterMom2Torque(Eigen::VectorXd& torques) const;

public:
	//NewtonEuler();
	NewtonEuler(std::shared_ptr<RobotModel>& model, int armIndex);
	virtual ~NewtonEuler();

	void SetGravity(const Eigen::Vector3d& gravity);
	void EvaluateAlgorithmStep(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, const Eigen::VectorXd& q_ddot, const Eigen::Vector3d& gravity, Eigen::VectorXd& torques);
	Eigen::MatrixXd GetA();
	//void GetB();
	Eigen::VectorXd GetC();
	Eigen::VectorXd GetMTilde();

	void PrintVars() const;
};

} /* namespace rml */

#endif /* INCLUDE_RML_NEWTONEULER_H_ */
