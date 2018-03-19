/*
 * NewtonEuler.h
 *
 *  Created on: Mar 12, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_NEWTONEULER_H_
#define INCLUDE_RML_NEWTONEULER_H_

#include <memory>
#include <rml/RobotModel.h>

namespace rml
{

const double STD_GRAVITY = 9.80665;

class NewtonEuler
{
//	struct BaseLinkData
//	{
//		Eigen::Vector3d omega;
//		Eigen::Vector3d omega_dot;
//		Eigen::Vector3d c;
//		Eigen::Vector3d c_dot;
//	};

	int numJoints_;

	std::shared_ptr<RobotModel> model_;
	std::shared_ptr<ArmModel> armModel_;
	std::shared_ptr<VehicleModel> vehicle_;

	//std::vector<Eigen::TransfMatrix> biTei_;
	std::vector<Eigen::RotMatrix> R_;
	std::vector<Eigen::Vector3d> rhoVec_;		// Distance between joint "i" and "i+1"
	Eigen::VectorXd q_, q_dot_, q_ddot_, q_ddot_Ai_, qZeroVec_;
	Eigen::VectorXd m_tilde_;
	std::vector<Eigen::Vector3d> omega_, omega_dot_, c_dot_, c_ddot_;
	Eigen::Vector3d gVect_, zeroVect3_;
//	Eigen::Vector3d k_i;
//	Eigen::MatrixXd k_iT;
	std::vector<Eigen::Vector3d> n_, f_;
	Eigen::Vector3d temp1_, temp2_, temp3_, temp4_, omega_prev_;
	std::vector<Eigen::Vector3d> r_qmc_, r_qpc_;

	std::vector<RobotLink> links_;

	void UpdateInternalVars(const Eigen::MatrixXd& q);
	void Init();
	void AddDummyBaseAndEE();
	void InterMom2Torque(Eigen::VectorXd& torques) const;
	void PrintVars() const;

public:
	NewtonEuler();
	NewtonEuler(std::shared_ptr<RobotModel>& model, int armIndex);
	virtual ~NewtonEuler();

	void NEAlgorithm(const Eigen::VectorXd& q_dot, const Eigen::VectorXd& q_ddot, double gravity, Eigen::VectorXd& torques);
	void GetA(Eigen::MatrixXd& A);
	//void GetB();
	void GetC(Eigen::VectorXd& C);
	void GetMBar(const Eigen::VectorXd& q_dot, Eigen::VectorXd& mbar);
};

} /* namespace rml */

#endif /* INCLUDE_RML_NEWTONEULER_H_ */
