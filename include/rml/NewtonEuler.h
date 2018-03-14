/*
 * NewtonEuler.h
 *
 *  Created on: Mar 12, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_NEWTONEULER_H_
#define INCLUDE_RML_NEWTONEULER_H_

#include <rml/ArmModel.h>

namespace rml
{

class NewtonEuler
{
	struct BaseLinkData
	{
		Eigen::Vector3d omega;
		Eigen::Vector3d omega_dot;
		Eigen::Vector3d c;
		Eigen::Vector3d c_dot;
	};

	int numJoints_;
	Eigen::TransfMatrix wTb0_;
	std::vector<Eigen::TransfMatrix> biTri_;
	Eigen::TransfMatrix eTt_;
	std::vector<RobotLink> links_;
	std::vector<int> jointType_;
	std::vector<Eigen::TransfMatrix> biTei_;
	std::vector<Eigen::RotMatrix> R_;
	std::vector<Eigen::Vector3d> rhoVec_;		// Distance between joint "i" and "i+1"
	Eigen::MatrixXd q_, q_dot_, q_ddot_, q_ddot_Ai_, qZeroVec_;
	Eigen::MatrixXd m_tilde_;
	std::vector<Eigen::Vector3d> omega_, omega_dot_, c_dot_, c_ddot_;
	Eigen::Vector3d k_i, zeroVect3_, gVect_;
	Eigen::MatrixXd k_iT;
	std::vector<Eigen::Vector3d> n_, f_;
	Eigen::Vector3d temp1_, temp2_, temp3_, temp4_, omega_prev_;
	std::vector<Eigen::Vector3d> r_qmc_, r_qpc_;

	void UpdateInternalVars(const Eigen::MatrixXd& q);
	void Init();
	void AddDummyBase(const std::vector<RobotLink>& links, const std::vector<int> jointType);
	void InterMom2Torque(Eigen::MatrixXd& torques) const;
	void PrintVars() const;

public:
	NewtonEuler();
	virtual ~NewtonEuler();

	void NewtonEuler(const BaseLinkData& baseData, const Eigen::MatrixXd& q_dot, const Eigen::MatrixXd& q_ddot, const double gravity, Eigen::MatrixXd& torques);
	void GetA(const BaseLinkData& baseData, Eigen::MatrixXd& A);
	void GetB();
	void GetC(const BaseLinkData& baseData, Eigen::MatrixXd& C);
	void GetMBar(const BaseLinkData& baseData, const Eigen::MatrixXd& q_dot, Eigen::MatrixXd& A);
};

} /* namespace rml */

#endif /* INCLUDE_RML_NEWTONEULER_H_ */
