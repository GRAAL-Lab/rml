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

#include "rml/ArmModel.h"
#include "rml/MatrixOperations.h"
#include "rml/PseudoInverse.h"
#include "rml/Defines.h"
#include "rml/SVD.h"

//#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)
//#define DBG_PRINT

using std::cout;
using std::endl;

namespace rml {

ArmModel::ArmModel() : numberOfJoints_(0), modelReadFromFile_(false), hasBeenInitialized_(false){

}

ArmModel::ArmModel(const ArmModel& other) : hasBeenInitialized_(other.hasBeenInitialized_), numberOfJoints_(other.numberOfJoints_),
		q_(other.q_), wTb0_(other.wTb0_), wTbi_(other.wTbi_), Tz_(other.Tz_), w_ki_(other.w_ki_), eTt_(other.eTt_),
		w_r_et_(other.w_r_et_), Jpinv_(other.Jpinv_), djdqJpinv_(other.djdqJpinv_),
		wTe_(other.wTe_), I3_(other.I3_), ZeroQ_(other.ZeroQ_) {

	/*
	 * If the arm model we are copying is not initialised we have to initialise all the pointers to NULL since
	 * we don't know yet the size of the containers.
	 */
	if (numberOfJoints_ == 0) {
		modelReadFromFile_ = false;
	}
	/*
	 * While if the arm has been already initialised we can copy all the necessary containers.
	 */
	else {
		wTei_.resize(numberOfJoints_);
		biTri_.resize(numberOfJoints_);
		biTei_.resize(numberOfJoints_);
		h_.resize(numberOfJoints_);
		dJdq_.resize(numberOfJoints_);
		jointLimitsMin_.resize(numberOfJoints_);
		jointLimitsMAX_.resize(numberOfJoints_);
		for (int i = 0; i < numberOfJoints_; i++) {
			wTei_[i] = other.wTei_[i];
			biTri_[i] = other.biTri_[i];
			biTei_[i] = other.biTei_[i];
			h_[i] = other.h_[i];
			dJdq_[i] = other.dJdq_[i];
		}

	}
}

ArmModel& ArmModel::operator=(ArmModel other) {
	swap(*this, other);
	return *this;
}

ArmModel* ArmModel::clone() const {
	return new ArmModel(*this);
}

ArmModel::~ArmModel() {

}

void ArmModel::SetJointPosition(const Eigen::VectorXd& q) {

	if(!hasBeenInitialized_){
		std::cout << "ERROR: Called SetJointPosition() on an unitialized ArmModel().\nExiting..." << std::endl;
		exit(0);
	}
	q_ = q;

	EvaluatebTt();
	EvaluatebJt();
	EvaluatedJdqNumeric();
}


const Eigen::VectorXd& ArmModel::GetJointPosition() const {
	return q_;
}


void ArmModel::SetArmJoints(int armJoints) {
	numberOfJoints_ = armJoints;

	wTei_.resize(numberOfJoints_);
	biTri_.resize(numberOfJoints_);
	biTei_.resize(numberOfJoints_);
	h_.resize(numberOfJoints_);
	dJdq_.resize(numberOfJoints_);
	jointLimitsMin_.resize(numberOfJoints_);
	jointLimitsMAX_.resize(numberOfJoints_);

	for (int i = 0; i < numberOfJoints_; ++i) {
		dJdq_.at(i) = Eigen::MatrixXd::Zero(6, numberOfJoints_);
	}

	bJt_.resize(6, numberOfJoints_);
	ZeroQ_ = Eigen::MatrixXd::Zero(numberOfJoints_, 1);
	q_ = ZeroQ_;

	hasBeenInitialized_ = true;

}


void ArmModel::InitMatrix() {
	//I3_ = Eigen::Matrix3d::Identity();
	//Tz_ = Eigen::Matrix4d::Identity();

}


void ArmModel::InitMatrix(std::string init_matrices_path) {

	InitMatrix();

	wTb0_ = Eigen::Matrix4d::Identity();
	eTt_ = Eigen::Matrix4d::Identity();
	ReadModelMatricesFromFile(init_matrices_path);
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
	Eigen::Matrix4d orig_wTb = wTb0_;
	wTb0_ = Eigen::Matrix4d::Identity();

	for (int jointNumber = 0; jointNumber < numberOfJoints_; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	bTt_ = wTei_[numberOfJoints_ - 1] * eTt_;
	wTb0_ = orig_wTb;

}


void ArmModel::ForwardDirectGeometry(int jointNumber) {
	// wTbi is the transformation between the base of joint <jointNumber> and the world frame <w>
	if (jointNumber == 0) {
		wTbi_ = wTb0_;
	} else {
		// in this case, the base of joint <jointNumber> is the position of the end-effector of the joint <jointNumber - 1>
		// a further one is subtracted because arrays are indexed from 0
		wTbi_ = wTei_[jointNumber - 1];
	}

	// biTri is the constant transformation between the base of the joint <i> and its end-effector
	// biTei also takes into account the actual rotation of the joint, so
	// biTei = biTri * Tz(qi)
	double cos_q, sin_q;

	cos_q = std::cos(static_cast<long double>(q_(jointNumber)));
	sin_q = std::sin(static_cast<long double>(q_(jointNumber)));
	//cos_q = cos( q_(jointNumber) );
	//sin_q = sin( q_(jointNumber) );

	Tz_(0, 0) = cos_q;
	Tz_(0, 1) = -sin_q;
	Tz_(1, 0) = sin_q;
	Tz_(1, 1) = cos_q;

	// biTei = biTri * Tz(qi)
	biTei_[jointNumber] = biTri_[jointNumber] * Tz_;

	// wTei is the transformation between the end-effector of joint <i> and world frame <w>
	// wTei = wTbi * biTei
	wTei_[jointNumber] = wTbi_ * biTei_[jointNumber];

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
	w_ki_ = wTei_.at(jointNumber).block(0,2,3,1); //GetSubMatrix(1, 3, 3, 3));

	h_[jointNumber].SetFirstVect3(w_ki_);
	h_[jointNumber].SetSecondVect3(w_ki_.cross(wTei_[endEffectorIndex].GetTransl() - wTei_[jointNumber].GetTransl()));
}


void ArmModel::BackwardDirectGeometryToolFrame(int jointNumber) {
	// Calcolo w_ki
	// Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione
	// ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
	// Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
	w_ki_ = wTei_.at(jointNumber).block(0,2,3,1);//GetSubMatrix(1, 3, 3, 3));

	h_[jointNumber].SetFirstVect3(w_ki_);
	h_[jointNumber].SetSecondVect3(w_ki_.cross(bTt_.GetTransl() - wTei_[jointNumber].GetTransl()));

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

	SVDParameters mySVD;

	if(numberOfJoints_ < 6){
		mySVD.lambda = 0.0001;
		mySVD.threshold = 0.0001;
		/// For defective manipulators
		//std::cout << "nrow: " << dJdq_[0].GetNumRows() << " ncol:" << dJdq_[0].GetNumColumns() << std::endl;
		rml::RegularizedPseudoInverse((Eigen::MatrixXd)bJt_.transpose(), mySVD);
		//Jpinv_ = .RegPseudoInverse(, );

		for(int k = 0; k < numberOfJoints_; k++)
		{
			Jmu(k) = 0;
			djdqJpinv_ = dJdq_[k].transpose() * Jpinv_;

			for(int i = 0; i < 5; i++) // now 5 is correct
				Jmu(k) += djdqJpinv_(i,i);

			Jmu(k) *= mySVD.mu;
		}
		//mu.PrintMtx("mu");
	} else {
		mySVD.lambda = 0.01;
		mySVD.threshold = 0.01;
		Jpinv_ = rml::RegularizedPseudoInverse(bJt_, mySVD);

		for (int k = 0; k < numberOfJoints_; k++) {
			Jmu(k) = 0;
			djdqJpinv_ = dJdq_[k] * Jpinv_;
			for (int i = 0; i < 6; i++)
				Jmu(k) += djdqJpinv_(i, i);
			Jmu(k) *= mySVD.mu;
		}
	}

	mu = mySVD.mu;
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


void ArmModel::EvaluateWorld2JointTransf(Eigen::TransfMatrix& wTj, int jointIndex) {
	for (int jointNumber = 0; jointNumber < jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	wTj = wTei_[jointIndex];
}

void ArmModel::EvaluateBase2JointTransf(Eigen::TransfMatrix& bTj, int jointIndex) {

	Eigen::Matrix4d orig_wTb = wTb0_;
	wTb0_ = Eigen::Matrix4d::Identity();

	for (int jointNumber = 0; jointNumber < jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	wTb0_ = orig_wTb;
	bTj = wTei_[jointIndex];
}


void ArmModel::EvaluateWorld2JointJacobian(Eigen::MatrixXd& wJj, int jointIndex) {

	for (int jointNumber = 0; jointNumber < jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	for (int jointNumber = jointIndex; jointNumber >= 0; jointNumber--)
		BackwardDirectGeometry(jointNumber, jointIndex);

	for (int i = jointIndex; i < numberOfJoints_; i++) {
		h_[i].setZero();
	}

	wJj = (h_[0]);
	for (int i = 1; i < numberOfJoints_; i++) {
		wJj = RightJuxtapose(wJj, h_[i]);
	}

}

void ArmModel::EvaluateBase2JointJacobian(Eigen::MatrixXd& bJj, int jointIndex) {

	Eigen::Matrix4d orig_wTb = wTb0_;
	wTb0_ = Eigen::Matrix4d::Identity();

	for (int jointNumber = 0; jointNumber < jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	for (int jointNumber = jointIndex; jointNumber >= 0; jointNumber--)
		BackwardDirectGeometry(jointNumber, jointIndex);

	for (int i = jointIndex; i < numberOfJoints_; i++) {
		h_[i].setZero();
	}

	bJj = (h_[0]);
	for (int i = 1; i < numberOfJoints_; i++) {
		bJj = RightJuxtapose(bJj, h_[i]);
	}

	wTb0_ = orig_wTb;
}

void ArmModel::ReadModelMatricesFromFile(std::string folder_path) {

	modelReadFromFile_ = true;
	hasBeenInitialized_ = true;
	std::cout << "To be implemented" << std::endl;

#ifdef DBG_PRINT
	cout << endl;
	wTb0_.PrintToDebugConsole("wtb0");
	char j_i[INTSTRSIZE];
	for (int n = 0; n < numberOfJoints_; n++) {
		sprintf(j_i, "%d", n);
		std::string biTri_name = "biTri_" + std::string(j_i);
		biTri_[n].PrintToDebugConsole(biTri_name.c_str());
	}
	eTt_.PrintToDebugConsole("eTt_");
#endif

}


void swap(rml::ArmModel& first, rml::ArmModel& second) {

	using std::swap;
	swap(first.hasBeenInitialized_, second.hasBeenInitialized_);
	swap(first.numberOfJoints_, second.numberOfJoints_);
	swap(first.q_, second.q_);
	swap(first.wTei_, second.wTei_);
	swap(first.biTri_, second.biTri_);
	swap(first.biTei_, second.biTei_);
	swap(first.wTb0_, second.wTb0_);
	swap(first.wTbi_, second.wTbi_);
	swap(first.Tz_, second.Tz_);
	swap(first.w_ki_, second.w_ki_);
	swap(first.h_, second.h_);
	swap(first.bTt_, second.bTt_);
	swap(first.eTt_, second.eTt_);
	swap(first.w_r_et_, second.w_r_et_);

	swap(first.jointLimitsMin_, second.jointLimitsMin_);
	swap(first.jointLimitsMAX_, second.jointLimitsMAX_);

	swap(first.dJdq_, second.dJdq_);
	swap(first.Jpinv_, second.Jpinv_);
	swap(first.djdqJpinv_, second.djdqJpinv_);

	swap(first.wTe_, second.wTe_);
	swap(first.bJt_, second.bJt_);
	swap(first.I3_, second.I3_);
	swap(first.ZeroQ_, second.ZeroQ_);
}

}



