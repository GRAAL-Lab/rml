/*
 * NewtonEuler.cpp
 *
 *  Created on: Mar 12, 2018
 *      Author: fraw
 */

#include "rml/NewtonEuler.h"

namespace rml
{

/*NewtonEuler::NewtonEuler() :
		numJoints_(0)
{
	// TODO Auto-generated constructor stub
}*/

NewtonEuler::~NewtonEuler()
{
	// TODO Auto-generated destructor stub
}

NewtonEuler::NewtonEuler(std::shared_ptr<RobotModel>& model, int armIndex)
{
	model_ = model;
	armModel_ = model->GetArm(armIndex);
	vehicle_ = model->GetVehicle();

	numJoints_ = model->GetArm(armIndex)->GetNumJoints();

	Init();

}

void NewtonEuler::Init()
{
	m_tilde_ = Eigen::VectorXd::Zero(numJoints_);

	/**
	 * The size of these internal variable is "numJoints+1" to take into account the
	 * base "dummy" joint.
	 */
	zeroVect3_.setZero();
	qZeroVec_ = Eigen::VectorXd::Zero(numJoints_ + 1);

	q_dot_ = qZeroVec_;
	q_ddot_ = qZeroVec_;

	omega_.resize(numJoints_ + 1, zeroVect3_);
	omega_dot_.resize(numJoints_ + 1, zeroVect3_);
	c_dot_.resize(numJoints_ + 1, zeroVect3_);
	c_ddot_.resize(numJoints_ + 1, zeroVect3_);

	rhoVec_.resize(numJoints_ + 1);

	r_qmc_.resize(numJoints_ + 1, zeroVect3_);
	r_qpc_.resize(numJoints_ + 1, zeroVect3_);


	/** In R_ we put all the rotation matrices used for projecting the velocities, accelerations,
	 *  forces and moments in the Newton-Euler algorithm (extracted from wTb, biTri and eTt).
	 */
	/**
	 * numjoints+2 = all links + dummybase + endeffector
	 */
	R_.resize(numJoints_ + 2);

	links_.resize(numJoints_ + 2);

	n_.resize(numJoints_ + 2, zeroVect3_);
	f_.resize(numJoints_ + 2, zeroVect3_);

	AddDummyBaseAndEE();

	/*k_i = Eigen::Vector3d::UnitZ();
	k_iT = k_i.transpose();*/
}

void NewtonEuler::InterMom2Torque(Eigen::VectorXd& torques) const
{
	/** MOM2TORQUE Returns the actual couple from the moment vector
	 *
	 * Knowing the e_i vector of the joints, gets the torque component
	 * from the moment vector (that has of course to be projected in the joint
	 * frame).
	 */
	for (int i = 0; i < numJoints_; ++i) {
		Eigen::MatrixXd e_iT = links_.at(i + 1).Axis().transpose();
		if (links_.at(i).Type() == JointType::Revolute) {
			torques(i) = (e_iT * links_.at(i + 1).inter_n_)(0);
		} else if (links_.at(i).Type() == JointType::Prismatic) {
			torques(i) = (e_iT * links_.at(i + 1).inter_f_)(0);
		}
	}
}

void NewtonEuler::AddDummyBaseAndEE()
{
	// Adding dummy 0 link
	links_.at(0) = RobotLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), Eigen::TransfMatrix(), 0.0, 0.0);
	Eigen::Matrix3d zeroMat3 = Eigen::Matrix3d::Zero();
	links_.at(0).SetPhysicalProperties(0.0, zeroVect3_, zeroVect3_, zeroMat3);

	for (int i = 0; i < numJoints_; ++i) {
		links_.at(i + 1) = armModel_->GetLink(i);
	}

	// Adding dummy EE link
	links_.at(numJoints_ + 1) = RobotLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), Eigen::TransfMatrix(), 0.0, 0.0);
	links_.at(numJoints_ + 1).SetPhysicalProperties(0.0, zeroVect3_, zeroVect3_, zeroMat3);
}

void NewtonEuler::EvaluateStep(const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, double gravity, Eigen::VectorXd& torques)
{

	//std::cout << "Check1" << std::endl;

	gVect_ = Eigen::Vector3d::UnitZ() * -gravity;

	omega_.at(0) = vehicle_->GetCartesianVelocity().GetFirstVect3();
	omega_dot_.at(0) = vehicle_->GetCartesianAcceleration().GetFirstVect3();

	rhoVec_.at(0).setZero();
	for (int i = 1; i < numJoints_; ++i) {
		rhoVec_.at(i) = armModel_->GetCurrentLinkTransf(i).GetTransl();
	}
	rhoVec_.at(numJoints_) = armModel_->GeteTt().GetTransl();

	R_.at(0) = armModel_->GetBaseTransf().GetRotMatrix(); // Base TODO Check
	for (int i = 0; i < numJoints_; ++i) {
		R_.at(i + 1) = armModel_->GetCurrentLinkTransf(i).GetRotMatrix(); // Base_i to Joint_i
	}
	R_.at(numJoints_ + 1) = armModel_->GeteTt().GetRotMatrix(); // Last joint to end effector


	for (int i = 0; i < numJoints_; ++i) {
		q_dot_(i + 1) = qdot(i);
		q_ddot_(i + 1) = qddot(i);

		omega_.at(i + 1).setZero();
		omega_dot_.at(i + 1).setZero();

		c_dot_.at(i + 1).setZero();
		c_ddot_.at(i + 1).setZero();
	}

	///*** FORWARD CALCULATION ***///
	/**
	 * Calculate links linear and angular acceleration
	 */


	//std::cout << "Check2" << std::endl;

	/** Angular Velocity **/
	for (int i = 1; i <= numJoints_; ++i) {
		if (links_.at(i).Type() == JointType::Revolute) {
			Eigen::Vector3d e_i = links_.at(i).Axis();
			omega_.at(i) = R_.at(i).Transpose() * omega_.at(i - 1) + q_dot_(i) * e_i;
		} else if (links_.at(i).Type() == JointType::Prismatic) {
			omega_.at(i) = R_.at(i).Transpose() * omega_.at(i - 1);
		}
	}

	//std::cout << "Check3" << std::endl;

	/** Angular Acceleration **/
	for (int i = 1; i <= numJoints_; ++i) {
		if (links_.at(i).Type() == JointType::Revolute) {
			Eigen::Vector3d e_i = links_.at(i).Axis();
			temp1_ = R_.at(i).Transpose() * omega_dot_.at(i - 1);
			temp2_ = q_ddot_(i) * e_i;
			temp3_ = q_dot_(i) * omega_.at(i).cross(e_i);
			omega_dot_.at(i) = temp1_ + temp2_ + temp3_;
		} else if (links_.at(i).Type() == JointType::Prismatic) {
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

	//std::cout << "Check4" << std::endl;

	/** Linear Acceleration **/
	for (int i = 1; i <= numJoints_; ++i) {
		if (links_.at(i).Type() == JointType::Revolute) {
			temp1_ = c_ddot_.at(i - 1) + omega_dot_.at(i - 1).cross(rhoVec_.at(i - 1));
			temp2_ = omega_.at(i - 1).cross(omega_.at(i - 1).cross(rhoVec_.at(i - 1)));
			c_ddot_.at(i) = R_.at(i).Transpose() * (temp1_ + temp2_);
		} else if (links_.at(i).Type() == JointType::Prismatic) {
			// Here I compute in advance the projection of omega(i-1) on
			// frame (i), (omega_prev)
			Eigen::Vector3d e_i = links_.at(i).Axis();
			omega_prev_ = R_.at(i).Transpose() * omega_.at(i - 1);
			temp1_ = c_ddot_.at(i - 1) + omega_dot_.at(i - 1).cross(rhoVec_.at(i - 1));
			temp2_ = omega_.at(i - 1).cross(omega_.at(i - 1).cross(rhoVec_.at(i - 1)));
			temp3_ = omega_prev_.cross((Eigen::Vector3d)(e_i * q_dot_(i)));
			temp4_ = omega_prev_.cross((Eigen::Vector3d)(e_i * q_dot_(i)));
			c_ddot_.at(i) = R_.at(i).Transpose() * (temp1_ + temp2_) + temp3_ + temp4_ + e_i * q_ddot_(i);
		}
	}

	//std::cout << "Check5" << std::endl;

	/** Projection of the acceleration on the Center of Mass **/
	for (int i = 1; i <= numJoints_; ++i) {
		//c_dot_.at(i) = c_dot_.at(i) + omega_.at(i).CrossProd(links_.at(i).CoM_);
		temp1_ = c_ddot_.at(i) + omega_dot_.at(i).cross(links_.at(i).CoM());
		temp2_ = omega_.at(i).cross(omega_.at(i).cross(links_.at(i).CoM()));
		c_ddot_.at(i) = temp1_ + temp2_;
	}

	///*** BACKWARD CALCULATION ***//
	/**
	 * Calculate Forces for every body using the calculated body acceleration (c_ddot) and
	 * project the gravity vector to the last frame (end-effector)
	 */

	for (int i = 0; i <= numJoints_; ++i) {
		links_.at(i).self_f_ = links_.at(i).Mass() * c_ddot_.at(i);
		links_.at(i).self_n_ = links_.at(i).Inertia() * omega_dot_.at(i)
				+ omega_.at(i).cross(links_.at(i).Inertia() * omega_.at(i));
		gVect_ = R_.at(i).Transpose() * gVect_;
	}

	for (int i = numJoints_; i > 0; --i) {

		// Distance from center_of_mass of link_i to joint_i+1
		r_qpc_.at(i) = links_.at(i).Sizes() - links_.at(i).CoM();

		// Distance from joint_i to center_of_mass of link_i
		r_qmc_.at(i) = R_.at(i).Transpose() * (links_.at(i - 1).Sizes() - rhoVec_.at(i - 1)) - r_qpc_.at(i);

		//exit(0);

		links_.at(i).inter_f_ = links_.at(i).self_f_ + R_.at(i + 1) * links_.at(i + 1).inter_f_
				- links_.at(i).Mass() * gVect_;
		links_.at(i).inter_n_ = links_.at(i).self_n_ + R_.at(i + 1) * links_.at(i + 1).inter_n_
				- r_qmc_.at(i).cross(links_.at(i).inter_f_)
				+ r_qpc_.at(i).cross(R_.at(i + 1) * links_.at(i + 1).inter_f_);
		gVect_ = R_.at(i) * gVect_;
	}

	//std::cout << TC_RED << "\n After Projection \n" << TC_NONE;
	//PrintVars();

	InterMom2Torque(torques);
}

void NewtonEuler::GetA(Eigen::MatrixXd& A)
{
	// Evaluating the columns of the A matrix (Inertia Matrix)
	for (int i = 0; i < numJoints_; ++i) {

		q_ddot_Ai_ = qZeroVec_;

		// Fixing the q_dot vector for the current joint
		q_ddot_Ai_(i) = 1;

		EvaluateStep(qZeroVec_, q_ddot_Ai_, 0.0, m_tilde_);

		// Filling current j column
		for (int j = 0; j < numJoints_; ++j) {
			A(i, j) = m_tilde_(j);
		}
	}
}

//void NewtonEuler::GetB()
//{
//
//}

void NewtonEuler::GetC(Eigen::VectorXd& C)
{
	EvaluateStep(qZeroVec_, qZeroVec_, STD_GRAVITY, C);
}

void NewtonEuler::GetMBar(const Eigen::VectorXd& q_dot, Eigen::VectorXd& m_bar)
{
	EvaluateStep(q_dot, qZeroVec_, STD_GRAVITY, m_bar);
}

//void NewtonEuler::PrintVars() const
//{
//	std::cout << tc::green << "links lenghtVec (b):\n" << tc::none;
//	for (uint i = 0; i < links_.size(); ++i) {
//		std::cout << i << ": ";
//		links_.at(i).LenghtVec().Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "links CoM (r):\n" << tc::none;
//	for (uint i = 0; i < links_.size(); ++i) {
//		std::cout << i << ": ";
//		links_.at(i).CoM().Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "links Mass:\n" << tc::none;
//	for (uint i = 0; i < links_.size(); ++i) {
//		std::cout << i << ": " << links_.at(i).Mass() << std::endl;
//	}
//
//	std::cout << tc::green << "links Inertia:\n" << tc::none;
//	for (uint i = 0; i < links_.size(); ++i) {
//		std::cout << i << ": ";
//		links_.at(i).Inertia().Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "omega_dot (a):\n" << tc::none;
//	for (uint i = 0; i < omega_dot_.size(); ++i) {
//		std::cout << i << ": ";
//		omega_dot_.at(i).Transpose().PrintMtx();
//	}
//
//	for (uint i = 0; i < biTei_.size(); i++) {
//		std::cout << i << ": ";
//		biTei_[i].PrintMtx("BiTei");
//	}
//
//	std::cout << tc::green << "q:\n" << tc::none;
//	q_.Transpose().PrintMtx("");
//
//	std::cout << tc::green << "q_dot_:\n" << tc::none;
//	q_dot_.Transpose().PrintMtx("");
//
//	std::cout << tc::green << "q_ddot_:\n" << tc::none;
//	q_ddot_.Transpose().PrintMtx("");
//
//	std::cout << tc::green << "omega:\n" << tc::none;
//	for (uint i = 0; i < omega_.size(); ++i) {
//		std::cout << i << ": ";
//		omega_.at(i).Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "omega_dot:\n" << tc::none;
//	for (uint i = 0; i < omega_dot_.size(); ++i) {
//		std::cout << i << ": ";
//		omega_dot_.at(i).Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "c_ddot:\n" << tc::none;
//	for (uint i = 0; i < c_ddot_.size(); ++i) {
//		std::cout << i << ": ";
//		c_ddot_.at(i).Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "r_qpc:\n" << tc::none;
//	for (uint i = 0; i < r_qpc_.size(); ++i) {
//		std::cout << i << ": ";
//		r_qpc_.at(i).Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "r_qmc_:\n" << tc::none;
//	for (uint i = 0; i < r_qmc_.size(); ++i) {
//		std::cout << i << ": ";
//		r_qmc_.at(i).Transpose().PrintMtx();
//	}
//
//	std::cout << tc::green << "self force:\n" << tc::none;
//	for (uint i = 0; i < links_.size(); i++) {
//		std::cout << i << ": ";
//		links_.at(i).self_f.Transpose().PrintMtx();
//	}
//	std::cout << std::endl;
//
//	std::cout << tc::green << "interaction force:\n" << tc::none;
//	for (uint i = 0; i < links_.size(); i++) {
//		std::cout << i << ": ";
//		links_.at(i).inter_f.Transpose().PrintMtx();
//	}
//	std::cout << std::endl;
//
//	std::cout << tc::green << "self mom:\n" << tc::none;
//	for (uint i = 0; i < links_.size(); i++) {
//		std::cout << i << ": ";
//		links_.at(i).self_n.Transpose().PrintMtx();
//	}
//	std::cout << std::endl;
//
//	std::cout << tc::green << "interaction mom:\n" << tc::none;
//	for (uint i = 0; i < links_.size(); i++) {
//		std::cout << i << ": ";
//		links_.at(i).inter_n.Transpose().PrintMtx();
//	}
//	std::cout << std::endl;
//}

} /* namespace rml */
