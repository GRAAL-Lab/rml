/*
 * ctrl_armModel.cc
 *
 *  Created on: Jul 25, 2015
 *      Author: francescow
 */

#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

#include <iostream>
#include <fstream>
#include <cmath>
#include <utility>
#include <vector>
#include <iterator>
#include <climits>
#include <stdlib.h>

#include "rml/MatrixOperations.h"
#include "rml/Functions.h"
#include "rml/PseudoInverse.h"
#include "rml/Types.h"
#include "rml/SVD.h"
#include "rml/ArmModel.h"
#include "rml/RobotLink.h"
#include "rml_internal/Futils.h"

using std::cout;
using std::endl;

namespace rml {

ArmModel::ArmModel() : numberOfJoints_(0), modelReadFromFile_(false), modelInitialized_(false){
}

ArmModel::~ArmModel() {

}

void ArmModel::AddLink(JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double joinLimMax) {

	links_.push_back(RobotLink(type, axis, baseTransf, jointLimMin, joinLimMax));
	numberOfJoints_ = links_.size();

	//	cout << numberOfJoints_ << " - ";
	//	futils::PrettyPrint(links_.back().baseTransf_, "baseTransf");

	baseTei_.push_back(Eigen::TransfMatrix());
	biTei_.push_back(Eigen::TransfMatrix());
	h_.push_back(Eigen::Vector6d());
	dJdq_.push_back(Eigen::MatrixXd());
	for (auto&& i : dJdq_) 				// access by forwarding reference, the type of i is auto&
		i.resize(6,numberOfJoints_);

	bJt_.resize(6, numberOfJoints_);
	ZeroQ_ = Eigen::VectorXd::Zero(numberOfJoints_);
	q_ = ZeroQ_;
	q_dot_ = ZeroQ_;
	q_ddot_ = ZeroQ_;

	modelInitialized_ = true;

	SetJointsPosition(ZeroQ_);

}

void ArmModel::SetJointsPosition(const Eigen::VectorXd& q) {

	if(!modelInitialized_){
		std::cout << "ERROR: Called SetJointPosition() on an unitialised ArmModel().\nExiting..." << std::endl;
		exit(0);
	}
	q_ = q;

	EvaluatebTt();
	EvaluatebJt();
	EvaluatedJdqNumeric();
}

void ArmModel::SetJointsVelocity(const Eigen::VectorXd& qdot) {

	if(!modelInitialized_){
		std::cout << "ERROR: Called SetJointsVelocity() on an unitialised ArmModel().\nExiting..." << std::endl;
		exit(0);
	}
	q_dot_ = qdot;

}

void ArmModel::SetJointsAcceleration(const Eigen::VectorXd& qddot) {

	if(!modelInitialized_){
		std::cout << "ERROR: Called SetJointsAcceleration() on an unitialised ArmModel().\nExiting..." << std::endl;
		exit(0);
	}
	q_ddot_ = qddot;

}

const Eigen::VectorXd& ArmModel::GetJointsPosition() const {
	return q_;
}

const Eigen::VectorXd& ArmModel::GetJointsVelocity() const {
	return q_dot_;
}

const Eigen::VectorXd& ArmModel::GetJointsAcceleration() const {
	return q_ddot_;
}


void ArmModel::EvaluatebJt() {
	//std::cerr << "I'm in EvaluateJwt" << std::endl;

	//EvaluatebTt(bTt_);
	for (int jointNumber = numberOfJoints_ - 1; jointNumber >= 0; jointNumber--)
		BackwardDirectGeometryToolFrame(jointNumber);

	bJt_ = (h_[0]);
	//wJt.PrintMtx("wJt start");
	for (int i = 1; i < numberOfJoints_; i++) {
		bJt_ = RightJuxtapose(bJt_, h_[i]);
		//wJt.PrintMtx("wJt");
	}
}


void ArmModel::EvaluatebTt() {
	for (int jointNumber = 0; jointNumber < numberOfJoints_; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	bTt_ = baseTei_[numberOfJoints_ - 1] * eTt_;
}


void ArmModel::ForwardDirectGeometry(int jointNumber) {

	// wTbi is the transformation between the base of joint <jointNumber> and the world frame <w>
	if (jointNumber == 0) {
		baseTbi_ = baseTb0_;
	} else {
		// in this case, the base of joint <jointNumber> is the position of the end-effector of the joint <jointNumber - 1>
		baseTbi_ = baseTei_[jointNumber - 1];
	}

	// biTri is the constant transformation between the base of the joint <i> and its end-effector
	// biTei also takes into account the actual rotation of the joint, so
	// biTei = biTri * Tz(qi)
	Eigen::TransfMatrix Tz_;

	if (links_.at(jointNumber).Type() == JointType::Revolute){
		Eigen::AngleAxisd rot = Eigen::AngleAxisd(q_(jointNumber), links_.at(jointNumber).Axis());
		Tz_.SetRotMatrix(rot.toRotationMatrix());

	}else if(links_.at(jointNumber).Type() == JointType::Prismatic){
		Eigen::Vector3d transl = q_(jointNumber) * links_.at(jointNumber).Axis();
		Tz_.SetTransl(transl);
	}

	// biTei = biTri * Tz(qi)
	biTei_[jointNumber] = links_.at(jointNumber).BaseTransf() * Tz_;

	// wTei is the transformation between the end-effector of joint <i> and world frame <w>
	// wTei = wTbi * biTei
	baseTei_[jointNumber] = baseTbi_ * biTei_[jointNumber];

	//cout << "forward index = " << jointNumber << endl;
	//biTri_[jointNumber - 1].PrintMtx("biTri");
	//Tz_.PrintMtx("Tz");
	//wTei_[jointNumber - 1].PrintMtx("wTei");
}

void ArmModel::BackwardDirectGeometry(int jointNumber, int endEffectorIndex) {
	// Calcolo w_ki
	// Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione
	// ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
	// Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
	base_ki_ = baseTei_.at(jointNumber).block(0,2,3,1); //GetSubMatrix(1, 3, 3, 3));

	h_[jointNumber].SetFirstVect3(base_ki_);
	h_[jointNumber].SetSecondVect3(base_ki_.cross(baseTei_[endEffectorIndex].GetTransl() - baseTei_[jointNumber].GetTransl()));
}


void ArmModel::BackwardDirectGeometryToolFrame(int jointNumber) {
	// Calcolo w_ki
	// Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione
	// ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
	// Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
	base_ki_ = baseTei_.at(jointNumber).block(0,2,3,1);//GetSubMatrix(1, 3, 3, 3));

	h_[jointNumber].SetFirstVect3(base_ki_);
	h_[jointNumber].SetSecondVect3(base_ki_.cross(bTt_.GetTransl() - baseTei_[jointNumber].GetTransl()));

	//cout << "backward index = " << jointNumber << endl;
	//wTei_[jointNumber - 1].PrintMtx("wTei");
	//w_ki_.PrintMtx("wki");
	//h_[jointNumber - 1].PrintMtx("h");
}


void ArmModel::EvaluateManipulability(Eigen::MatrixXd& Jmu, double& mu) {
	int flag;
	double myMu;
	//EvaluatewJt(wJt_);
	//EvaluatedJdqNumeric(dJdq_);
	Jmu.resize(1, numberOfJoints_);
	Jmu.setZero();

	SVDData mySVD;

	if(numberOfJoints_ < 6){
		mySVD.params.lambda = 0.0001;
		mySVD.params.threshold = 0.0001;
		/// For defective manipulators
		//std::cout << "nrow: " << dJdq_[0].rows() << " ncol:" << dJdq_[0].cols() << std::endl;
		Jpinv_ = rml::RegularizedPseudoInverse((Eigen::MatrixXd)bJt_.transpose(), mySVD);
		//Jpinv_ = .RegPseudoInverse(, );
		for(int k = 0; k < numberOfJoints_; k++)
		{
			Jmu(k) = 0;
			djdqJpinv_ = dJdq_[k].transpose() * Jpinv_;

			for(int i = 0; i < 5; i++) // now 5 is correct
				Jmu(k) += djdqJpinv_(i,i);

			Jmu(k) *= mySVD.results.mu;
		}

		//mu.PrintMtx("mu");
	} else {
		mySVD.params.lambda = 0.01;
		mySVD.params.threshold = 0.01;
		Jpinv_ = rml::RegularizedPseudoInverse(bJt_, mySVD);

		for (int k = 0; k < numberOfJoints_; k++) {
			Jmu(k) = 0;
			djdqJpinv_ = dJdq_[k] * Jpinv_;
			for (int i = 0; i < 6; i++)
				Jmu(k) += djdqJpinv_(i, i);
			Jmu(k) *= mySVD.results.mu;
		}
	}

	mu = mySVD.results.mu;
}


void ArmModel::EvaluatedJdqNumeric() {

	Eigen::MatrixXd bJt_0, bJt_dQ;
	Eigen::MatrixXd dQ, qVar, q_orig;
	double delta_q = 1E-6;
	q_orig = q_;
	bJt_0 = GetbJt();
	//futils::PrettyPrint(wJt_0,"wJt_0");

	/// Here we iterate till "numJoints - 1" since the last joint will not influence
	/// on the Jacobian since nothing is connected to it.
	for (int i = 0; i < numberOfJoints_ - 1; ++i) {
		/// Computing the single q variation vector dQ.
		dQ = ZeroQ_;    		 // Initialising a zero vector
		dQ(i) = delta_q;   		 // Getting the q variation
		qVar = q_orig + dQ;      // Computing the new single q variation based on previous state

		/// Computing scalar delta_q (the denominator of the numerical integration).
		q_ = qVar;

		EvaluatebTt();
		EvaluatebJt();
		bJt_dQ = GetbJt();
		//futils::PrettyPrint(wJt_dQ,"wJt_dQ");
		for (int iJrow = 0; iJrow < 6; ++iJrow) {
			for (int iJcol = 0; iJcol < numberOfJoints_; ++iJcol) {
				//std::cout << "(i,j) = " << iJrow << "," << iJcol << std::endl;
				//futils::PrettyPrint(dJdq_[i],"dJdq[i]");
				dJdq_.at(i)(iJrow, iJcol) = (bJt_dQ(iJrow, iJcol) -  bJt_0(iJrow, iJcol))/delta_q;
				if(std::fabs(dJdq_.at(i)(iJrow, iJcol)) < 1E-6) dJdq_.at(i)(iJrow, iJcol) = 0.0;
			}
		}
	}
	q_ = q_orig;

	EvaluatebTt();
	EvaluatebJt();

}


Eigen::TransfMatrix ArmModel::GetBase2JointTransf(int jointIndex) {
	for (int jointNumber = 0; jointNumber <= jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	return baseTei_[jointIndex];
}


Eigen::MatrixXd ArmModel::GetBase2JointJacobian(int jointIndex) {

	for (int jointNumber = 0; jointNumber < jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}

	//Here we calculate the h columns involved in our joint's jacobian
	for (int jointNumber = jointIndex; jointNumber >= 0; jointNumber--)
		BackwardDirectGeometry(jointNumber, jointIndex);

	//Then we set all the remainings columns to zero
	for (int i = jointIndex + 1; i < numberOfJoints_; i++) {
		h_[i].setZero();
	}

	Eigen::MatrixXd bJj;// = (h_[0]);
	for (int i = 0; i < numberOfJoints_; i++) {
		bJj = RightJuxtapose(bJj, h_[i]);
	}
	return bJj;
}

void ArmModel::AddRigidBodyFrame(std::string ID, int jointIndex, Eigen::TransfMatrix TMat) {

	IndexedTMat myMat(jointIndex, TMat);
	attachedBodyFrames_.insert(std::make_pair(ID, myMat));
}

Eigen::TransfMatrix ArmModel::GetAttachedBodyTransf(std::string& ID) {
	return attachedBodyFrames_.at(ID).second;
}

Eigen::TransfMatrix ArmModel::GetCurrentAttachedBodyTransf(std::string& ID) {

	int jointIndex = attachedBodyFrames_.at(ID).first;
	Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID).second;
	return GetBase2JointTransf(jointIndex) * TMat;
}

Eigen::MatrixXd ArmModel::GetAttachedBodyJacobian(std::string& ID) {

	int jointIndex = attachedBodyFrames_.at(ID).first;
	Eigen::TransfMatrix TMat = attachedBodyFrames_.at(ID).second;
	Eigen::Vector3d projectedTransl = GetBase2JointTransf(jointIndex).GetRotMatrix() * TMat.GetTransl();
	return GetRigidBodyMatrix(projectedTransl) * GetBase2JointJacobian(jointIndex);
}


void ArmModel::ReadModelMatricesFromFile(std::string folder_path) {

	modelReadFromFile_ = true;
	modelInitialized_ = true;
	std::cout << "To be implemented" << std::endl;

}

}

