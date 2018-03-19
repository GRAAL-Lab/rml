/*
 * NewtonEuler.cpp
 *
 *  Created on: Mar 12, 2018
 *      Author: fraw
 */

#include "rml/NewtonEuler.h"

namespace rml
{

NewtonEuler::NewtonEuler() : numJoints_(0)
{
	// TODO Auto-generated constructor stub
}

NewtonEuler::~NewtonEuler()
{
	// TODO Auto-generated destructor stub
}

NewtonEuler::NewtonEuler(std::shared_ptr<ArmModel>& model){
	numJoints_ = model->GetNumJoints();
	armModel_ = model;
}

void NewtonEuler::Init() {

	/**
	 * The size of these internal variable is "numJoints+1" to take into account the
	 * base "dummy" joint.
	 */
	zeroVect3_ = CMAT::Matrix::Zeros(3, 1);
	qZeroVec_ = CMAT::Matrix::Zeros(numJoints_+1, 1);

	q_dot_ = qZeroVec_;
	q_ddot_ = qZeroVec_;

	m_tilde_ = CMAT::Matrix::Zeros(numJoints_,1);

	/** In R_ we put all the rotation matrices used for projecting the velocities, accelerations,
	 *  forces and moments in the Newton-Euler algorithm (extracted from wTb, biTri and eTt).
	 */
	R_.resize(numJoints_ + 2);

	biTei_.resize(numJoints_);
	rhoVec_.resize(numJoints_ + 1);

	jointType_.resize(numJoints_ + 1);
	links_.resize(numJoints_ + 2);

	omega_.resize(numJoints_ + 1, zeroVect3_);
	omega_dot_.resize(numJoints_ + 1, zeroVect3_);
	c_dot_.resize(numJoints_ + 1, zeroVect3_);
	c_ddot_.resize(numJoints_ + 1, zeroVect3_);

	/// +2 is base+endeffector
	n_.resize(numJoints_ + 2, zeroVect3_);
	f_.resize(numJoints_ + 2, zeroVect3_);

	r_qmc_.resize(numJoints_ + 1, zeroVect3_);
	r_qpc_.resize(numJoints_ + 1, zeroVect3_);


	k_i(1) = 0;
	k_i(2) = 0;
	k_i(3) = 1;
	k_iT = k_i.Transpose();

}

void NewtonEuler::AddDummyBase(const vector<RobotLink>& links, const vector<int> jointType) {

	/// Adding dummy 0 link
	jointType_.at(0) = Rot;

	links_.at(0) = RobotLink::Generator(0.0, zeroVect3_, zeroVect3_, CMAT::Matrix::Zeros(3));
	for (int i = 0; i < numJoints_; ++i) {
		jointType_.at(i + 1) = jointType.at(i);
		links_.at(i + 1) = links.at(i);
	}
	links_.at(numJoints_ + 1) = RobotLink::Generator(0.0, zeroVect3_, zeroVect3_, CMAT::Matrix::Zeros(3));
}

void NewtonEuler::NewtonEuler(const BaseLinkData& baseData, const CMAT::Matrix& qdot, const CMAT::Matrix& qddot, const double gravity, CMAT::Matrix& torques) {

	int i;

	gVect_(1) = 0;
	gVect_(2) = 0;
	gVect_(3) = -gravity;

	omega_.at(0) = baseData.omega;
	omega_dot_.at(0) = baseData.omega_dot;


	for (i = 0; i < numJoints_; ++i) {
		q_dot_(i + 2) = qdot(i + 1);
		q_ddot_(i + 2) = qddot(i + 1);

		omega_.at(i + 1) = zeroVect3_;
		omega_dot_.at(i + 1) = zeroVect3_;

		c_dot_.at(i + 1) = zeroVect3_;
		c_ddot_.at(i + 1) = zeroVect3_;
	}


	///*** FORWARD CALCULATION ***///
	/**
	 * Calculate links linear and angular acceleration
	 */

	/** Angular Velocity **/
	for (i = 1; i <= numJoints_; ++i) {
		if (jointType_.at(i) == Rot) {
			omega_.at(i) = R_.at(i).Transpose() * omega_.at(i - 1) + q_dot_(i+1) * k_i;
		} else if (jointType_.at(i) == Trans) {
			omega_.at(i) = R_.at(i).Transpose() * omega_.at(i - 1);
		}
	}

	/** Angular Acceleration **/
	for (i = 1; i <= numJoints_; ++i) {
		if (jointType_.at(i) == Rot) {
			temp1_ = R_.at(i).Transpose() * omega_dot_.at(i - 1);
			temp2_ = q_ddot_(i+1) * k_i;
			temp3_ = q_dot_(i+1) * omega_.at(i).CrossProd(k_i);
			omega_dot_.at(i) = temp1_ + temp2_ + temp3_;
		} else if (jointType_.at(i) == Trans) {
			omega_dot_.at(i) = R_.at(i).Transpose() * omega_dot_.at(i - 1);
		}
	}

	/** Linear Velocity **/
	/*
	 for(i = 1; i <= numJoints_; ++i){
	 if(jointType_.at(i) == Rot){
	 temp1_ = c_dot_.at(i-1) + omega_.at(i-1).CrossProd(omega_dot_.at(i-1));
	 c_dot_.at(i) = R_.at(i).Transpose() * temp1_;
	 }else if(jointType_.at(i) == Trans){
	 temp1_ = c_dot_.at(i-1) + omega_.at(i-1).CrossProd(omega_dot_.at(i-1));
	 temp2_ = q_dot_.at(i) * k_i;
	 c_dot_.at(i) = R_.at(i).Transpose() * temp1_ + temp2_;
	 }
	 }
	 */

	/** Linear Acceleration **/
	for (i = 1; i <= numJoints_; ++i) {
		if (jointType_.at(i) == Rot) {
			temp1_ = c_ddot_.at(i - 1) + omega_dot_.at(i - 1).CrossProd(rhoVec_.at(i - 1));
			temp2_ = omega_.at(i - 1).CrossProd(omega_.at(i - 1).CrossProd(rhoVec_.at(i - 1)));
			c_ddot_.at(i) = R_.at(i).Transpose() * (temp1_ + temp2_);
		} else if (jointType_.at(i) == Trans) {
			// Here I compute in advance the projection of omega(i-1) on
			// frame (i), (omega_prev)
			omega_prev_ = R_.at(i).Transpose() * omega_.at(i - 1);
			temp1_ = c_ddot_.at(i - 1) + omega_dot_.at(i - 1).CrossProd(rhoVec_.at(i - 1));
			temp2_ = omega_.at(i - 1).CrossProd(omega_.at(i - 1).CrossProd(rhoVec_.at(i - 1)));
			temp3_ = omega_prev_.CrossProd(k_i * q_dot_(i+1));
			temp4_ = omega_prev_.CrossProd(k_i * q_dot_(i+1));
			c_ddot_.at(i) = R_.at(i).Transpose() * (temp1_ + temp2_) + temp3_ + temp4_ + k_i * q_ddot_(i+1);
		}
	}


	/** Projection of the acceleration on the Center of Mass **/
	for (i = 1; i <= numJoints_; ++i) {
		//c_dot_.at(i) = c_dot_.at(i) + omega_.at(i).CrossProd(links_.at(i).CoM_);
		temp1_ = c_ddot_.at(i) + omega_dot_.at(i).CrossProd(links_.at(i).CoM());
		temp2_ = omega_.at(i).CrossProd(omega_.at(i).CrossProd(links_.at(i).CoM()));
		c_ddot_.at(i) = temp1_ + temp2_;
	}

	///*** BACKWARD CALCULATION ***//
	/**
	 * Calculate Forces for every body using the calculated body acceleration (c_ddot) and
	 * project the gravity vector to the last frame (end-effector)
	 */

	for (i = 0; i <= numJoints_; ++i) {
		links_.at(i).self_f = links_.at(i).Mass() * c_ddot_.at(i);
		links_.at(i).self_n = links_.at(i).Inertia() * omega_dot_.at(i) + omega_.at(i).CrossProd(links_.at(i).Inertia() * omega_.at(i));
		gVect_ = R_.at(i).Transpose() * gVect_;
	}


	for (i = numJoints_; i > 0; --i) {

		// Distance from center_of_mass of link_i to joint_i+1
		r_qpc_.at(i) = links_.at(i).LenghtVec() - links_.at(i).CoM();

		// Distance from joint_i to center_of_mass of link_i
		r_qmc_.at(i) = R_.at(i).Transpose() * (links_.at(i - 1).LenghtVec() - rhoVec_.at(i - 1)) - r_qpc_.at(i);

		//exit(0);

		links_.at(i).inter_f = links_.at(i).self_f + R_.at(i + 1) * links_.at(i + 1).inter_f - links_.at(i).Mass() * gVect_;
		links_.at(i).inter_n = links_.at(i).self_n + R_.at(i + 1) * links_.at(i + 1).inter_n -
				r_qmc_.at(i).CrossProd(links_.at(i).inter_f) + r_qpc_.at(i).CrossProd(R_.at(i + 1) * links_.at(i + 1).inter_f);
		gVect_ = R_.at(i) * gVect_;

	}

	//std::cout << TC_RED << "\n After Projection \n" << TC_NONE;
	//PrintVars();

	InterMom2Torque(torques);
}

void NewtonEuler::GetA(const BaseLinkData& baseData, CMAT::Matrix& A) {

	/// Evaluating the columns of the A matrix (Inertia Matrix)

	for (int i = 0; i < numJoints_; ++i) {

		q_ddot_Ai_ = qZeroVec_;

		q_ddot_Ai_(i+1) = 1;

		NewtonEuler(baseData, qZeroVec_, q_ddot_Ai_, 0.0, m_tilde_);

		for (int j = 0; j < numJoints_; ++j) {
			A(i + 1, j + 1) = m_tilde_(j+1);
		}
	}
}

void NewtonEuler::GetB() {

}

void NewtonEuler::GetC(const BaseLinkData& baseData, CMAT::Matrix& C) {
	NewtonEuler(baseData, qZeroVec_, qZeroVec_, STD_GRAVITY, C);
}

void NewtonEuler::GetMBar(const BaseLinkData& baseData, const CMAT::Matrix& q_dot, CMAT::Matrix& m_bar) {
	NewtonEuler( baseData, q_dot, qZeroVec_, STD_GRAVITY, m_bar );
}

void NewtonEuler::PrintVars() const {


	std::cout << tc::green << "links lenghtVec (b):\n" << tc::none;
	for(uint i = 0; i < links_.size(); ++i){
		std::cout << i << ": "; links_.at(i).LenghtVec().Transpose().PrintMtx();
	}

	std::cout << tc::green << "links CoM (r):\n" << tc::none;
	for(uint i = 0; i < links_.size(); ++i){
		std::cout << i << ": "; links_.at(i).CoM().Transpose().PrintMtx();
	}

	std::cout << tc::green << "links Mass:\n" << tc::none;
	for(uint i = 0; i < links_.size(); ++i){
		std::cout << i << ": " << links_.at(i).Mass() << std::endl;
	}

	std::cout << tc::green << "links Inertia:\n" << tc::none;
	for(uint i = 0; i < links_.size(); ++i){
		std::cout << i << ": "; links_.at(i).Inertia().Transpose().PrintMtx();
	}

	std::cout << tc::green << "omega_dot (a):\n" << tc::none;
	for(uint i = 0; i < omega_dot_.size(); ++i){
		std::cout << i << ": "; omega_dot_.at(i).Transpose().PrintMtx();
	}

	for (uint i=0; i<biTei_.size();i++){
		std::cout << i << ": "; biTei_[i].PrintMtx("BiTei");
	}

	std::cout << tc::green << "q:\n" << tc::none;
	q_.Transpose().PrintMtx("");

	std::cout << tc::green << "q_dot_:\n" << tc::none;
	q_dot_.Transpose().PrintMtx("");

	std::cout << tc::green << "q_ddot_:\n" << tc::none;
	q_ddot_.Transpose().PrintMtx("");

	std::cout << tc::green << "omega:\n" << tc::none;
	for(uint i = 0; i < omega_.size(); ++i){
		std::cout << i << ": "; omega_.at(i).Transpose().PrintMtx();
	}

	std::cout << tc::green << "omega_dot:\n" << tc::none;
	for(uint i = 0; i < omega_dot_.size(); ++i){
		std::cout << i << ": "; omega_dot_.at(i).Transpose().PrintMtx();
	}

	std::cout << tc::green << "c_ddot:\n" << tc::none;
	for(uint i = 0; i < c_ddot_.size(); ++i){
		std::cout << i << ": "; c_ddot_.at(i).Transpose().PrintMtx();
	}

	std::cout << tc::green << "r_qpc:\n" << tc::none;
	for(uint i = 0; i < r_qpc_.size(); ++i){
		std::cout << i << ": "; r_qpc_.at(i).Transpose().PrintMtx();
	}

	std::cout << tc::green << "r_qmc_:\n" << tc::none;
	for(uint i = 0; i < r_qmc_.size(); ++i){
		std::cout << i << ": "; r_qmc_.at(i).Transpose().PrintMtx();
	}

	std::cout << tc::green << "self force:\n" << tc::none;
	for (uint i=0; i<links_.size();i++){
	    std::cout << i << ": "; links_.at(i).self_f.Transpose().PrintMtx();
	}
	std::cout << std::endl;

	std::cout << tc::green << "interaction force:\n" << tc::none;
	for (uint i=0; i<links_.size();i++){
	    std::cout << i << ": "; links_.at(i).inter_f.Transpose().PrintMtx();
	}
	std::cout << std::endl;

	std::cout << tc::green << "self mom:\n" << tc::none;
	for (uint i=0; i<links_.size();i++){
	    std::cout << i << ": "; links_.at(i).self_n.Transpose().PrintMtx();
	}
	std::cout << std::endl;

	std::cout << tc::green << "interaction mom:\n" << tc::none;
	for (uint i=0; i<links_.size();i++){
	    std::cout << i << ": "; links_.at(i).inter_n.Transpose().PrintMtx();
	}
	std::cout << std::endl;
}

} /* namespace rml */
