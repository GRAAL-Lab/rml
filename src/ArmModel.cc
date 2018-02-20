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

#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

//#define DBG_PRINT

using std::cout;
using std::endl;

namespace rml {

ArmModel::ArmModel() : numberOfJoints_(0), modelReadFromFile_(false), hasBeenInitialized_(false){
	/*wTei_ = NULL;
	biTri_ = NULL;
	biTei_ = NULL;
	h_ = NULL;
	arrayJ_ = NULL;
	arrayQ_ = NULL;
	dJdq_ = NULL;*/
}

ArmModel::ArmModel(const ArmModel& other) : hasBeenInitialized_(other.hasBeenInitialized_),numberOfJoints_(other.numberOfJoints_), q_(other.q_), wTb0_(
	other.wTb0_), wTbi_(other.wTbi_), Tz_(other.Tz_), w_ki_(other.w_ki_), wTt_(other.wTt_),
	eTt_(other.eTt_), w_r_et_(other.w_r_et_), Jpinv_(other.Jpinv_), djdqJpinv_(other.djdqJpinv_),
	wTe_(other.wTe_), wJt_(other.wJt_), I3_(other.I3_), ZeroQ_(other.ZeroQ_) {

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
		arrayJ_.resize(6 * numberOfJoints_);
		arrayQ_.resize(numberOfJoints_);

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

	EvaluatewTt(wTt_);
	EvaluatebTt(bTt_);
	EvaluatewJt(wJt_);
	EvaluatebJt(bJt_);
	EvaluatedJdqNumeric(dJdq_);
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
	arrayJ_.resize(6 * numberOfJoints_);
	arrayQ_.resize(numberOfJoints_);
	dJdq_.resize(numberOfJoints_);

	for (int i = 0; i < numberOfJoints_; ++i) {
		dJdq_.at(i).setZero();// = Eigen::MatrixXd::Zero(6, numberOfJoints_);
	}

	wJt_.resize(6, numberOfJoints_);
	ZeroQ_ = Eigen::MatrixXd::Zero(numberOfJoints_, 1);
	q_ = ZeroQ_;

	hasBeenInitialized_ = true;

}

void ArmModel::InitMatrix() {
	I3_ = Eigen::Matrix3d::Identity();

}

void ArmModel::InitMatrix(std::string init_matrices_path) {

	InitMatrix();

	wTb0_ = Eigen::Matrix4d::Identity();
	eTt_ = Eigen::Matrix4d::Identity();
	ReadModelMatricesFromFile(init_matrices_path);
}


void ArmModel::ForwardDirectGeometry(int jointNumber) {
	// wTbi is the transformation between the base of joint <jointNumber> and the world frame <w>
	if (jointNumber == 1) {
		wTbi_ = wTb0_;
	} else {
		// in this case, the base of joint <jointNumber> is the position of the end-effector of the joint <jointNumber - 1>
		// a further one is subtracted because arrays are indexed from 0
		wTbi_ = wTei_[(jointNumber - 1) - 1];
	}

	// biTri is the constant transformation between the base of the joint <i> and its end-effector
	// biTei also takes into account the actual rotation of the joint, so
	// biTei = biTri * Tz(qi)
	double cos_q, sin_q;

	cos_q = std::cos(static_cast<long double>(q_(jointNumber)));
	sin_q = std::sin(static_cast<long double>(q_(jointNumber)));
	//cos_q = cos( q_(jointNumber) );
	//sin_q = sin( q_(jointNumber) );

	Tz_(1, 1) = cos_q;
	Tz_(1, 2) = -sin_q;
	Tz_(2, 1) = sin_q;
	Tz_(2, 2) = cos_q;

	// biTei = biTri * Tz(qi)
	biTei_[jointNumber - 1] = biTri_[jointNumber - 1] * Tz_;

	// wTei is the transformation between the end-effector of joint <i> and world frame <w>
	// wTei = wTbi * biTei
	wTei_[jointNumber - 1] = wTbi_ * biTei_[jointNumber - 1];

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
	w_ki_ = wTei_.at(jointNumber - 1).block(0,2,3,1); //GetSubMatrix(1, 3, 3, 3));

	SetFirstVect3(h_[jointNumber - 1],w_ki_);
	SetSecondVect3(h_[jointNumber - 1],w_ki_.cross((GetTrasl(wTei_[endEffectorIndex - 1]) - GetTrasl(wTei_[jointNumber - 1]))));
}

void ArmModel::BackwardDirectGeometryToolFrame(int jointNumber) {
	// Calcolo w_ki
	// Dal momento che ri_ki e ei_ki sono ruotate lungo ki e per convenzione
	// ogni giunto ruota lungo z si ha che ri_ki = ei_ki = [ 0 0 1 ]'
	// Di conseguenza w_ki e' la 3a colonna della R che lo proietta sul mondo
	w_ki_ = wTei_.at(jointNumber - 1).block(0,2,3,1);//GetSubMatrix(1, 3, 3, 3));

	SetFirstVect3(h_[jointNumber - 1],w_ki_);
	SetSecondVect3(h_[jointNumber - 1],w_ki_.cross((GetTrasl(wTt_) - GetTrasl(wTei_[jointNumber - 1]))));

	//cout << "backward index = " << jointNumber << endl;
	//wTei_[jointNumber - 1].PrintMtx("wTei");
	//w_ki_.PrintMtx("wki");
	//h_[jointNumber - 1].PrintMtx("h");
}

void ArmModel::EvaluatewJt(Eigen::MatrixXd& wJt) {
	//std::cerr << "I'm in EvaluateJwt" << std::endl;

	for (int jointNumber = numberOfJoints_; jointNumber > 0; jointNumber--)
		BackwardDirectGeometryToolFrame(jointNumber);

	wJt = (h_[0]);
	//wJt.PrintMtx("wJt start");
	for (int i = 1; i < numberOfJoints_; i++) {
		wJt = RightJuxtapose(wJt, h_[i]);
		//wJt.PrintMtx("wJt");
	}

}

void ArmModel::EvaluatebJt(Eigen::MatrixXd& bJt) {
	//std::cerr << "I'm in EvaluateJwt" << std::endl;

	//EvaluatebTt(bTt_);
	for (int jointNumber = numberOfJoints_; jointNumber > 0; jointNumber--)
		BackwardDirectGeometryToolFrame(jointNumber);

	bJt = (h_[0]);
	//wJt.PrintMtx("wJt start");
	for (int i = 1; i < numberOfJoints_; i++) {
		bJt = RightJuxtapose(bJt, h_[i]);
		//wJt.PrintMtx("wJt");
	}
}

void ArmModel::EvaluatewTt(Eigen::Matrix4d& wTt) {

	for (int jointNumber = 1; jointNumber <= numberOfJoints_; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	wTt_ = wTei_[numberOfJoints_ - 1] * eTt_;
	wTt = wTt_;

}

void ArmModel::EvaluatebTt(Eigen::Matrix4d& bTt) {
	Eigen::Matrix4d orig_wTb = wTb0_;
	wTb0_ = Eigen::Matrix4d::Identity();

	for (int jointNumber = 1; jointNumber <= numberOfJoints_; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	bTt_ = wTei_[numberOfJoints_ - 1] * eTt_;
	bTt = bTt_;
	wTb0_ = orig_wTb;

}

void ArmModel::EvaluateManipulability(Eigen::MatrixXd& mu, Eigen::MatrixXd& Jmu) {
	int flag;
	double myMu;
	//EvaluatewJt(wJt_);
	//EvaluatedJdqNumeric(dJdq_);



	if(numberOfJoints_ < 6){
		/// For defective manipulators
		//std::cout << "nrow: " << dJdq_[0].GetNumRows() << " ncol:" << dJdq_[0].GetNumColumns() << std::endl;
		rml::RegularizedPseudoInverse((Eigen::MatrixXd)wJt_.transpose(), 0.0001, 0.0001, &myMu, &flag);
		//Jpinv_ = .RegPseudoInverse(, );

		for(int k = 0; k < numberOfJoints_; k++)
		{
			Jmu(k + 1) = 0;
			djdqJpinv_ = dJdq_[k].transpose() * Jpinv_;

			for(int i = 1; i <= 5; i++) // now 5 is correct
				Jmu(k + 1) += djdqJpinv_(i,i);

			Jmu(k + 1) *= mu(1,1);
		}
		//mu.PrintMtx("mu");
	} else {
		Jpinv_ = rml::RegularizedPseudoInverse(wJt_,0.01, 0.01, &myMu, &flag);

		for (int k = 0; k < numberOfJoints_; k++) {
			Jmu(k + 1) = 0;
			djdqJpinv_ = dJdq_[k] * Jpinv_;
			for (int i = 1; i <= 6; i++)
				Jmu(k + 1) += djdqJpinv_(i, i);
			Jmu(k + 1) *= mu(1, 1);
		}
	}

	mu(0,0) = myMu;
}

void ArmModel::EvaluatedJdqNumeric(std::vector<Eigen::MatrixXd>& dJdq) {

	Eigen::MatrixXd wJt_0, wJt_dQ;
	Eigen::MatrixXd dQ, qVar, q_orig;
	double delta_q = 1E-6;
	q_orig = q_;
	wJt_0 = GetwJt();

	/// Here we iterate till "numJoints - 1" since the last joint will not influence
	/// on the Jacobian since nothing is connected to it.
	for (int i = 0; i < numberOfJoints_ - 1; ++i) {
		/// Computing the single q variation vector dQ.
		dQ = ZeroQ_;    		// Initialising a zero vector
		dQ(i + 1) = delta_q;    // Getting the q variation
		qVar = q_orig + dQ;     // Computing the new single q variation based on previous state

		/// Computing scalar delta_q (the denominator of the numerical integration).

		q_ = qVar;
		EvaluatewTt(wTt_);
		EvaluatewJt(wJt_dQ);// = GetwJt();;
		//wJt_dQ.PrintMtx("wJt_dQ");
		for (int iJrow = 0; iJrow < 6; ++iJrow) {
			for (int iJcol = 0; iJcol < numberOfJoints_; ++iJcol) {
				dJdq[i](iJrow + 1, iJcol + 1) = (wJt_dQ(iJrow + 1, iJcol + 1 ) -  wJt_0(iJrow + 1, iJcol + 1 ))/delta_q;
				if(std::fabs(dJdq[i](iJrow + 1, iJcol + 1)) < 1E-6) dJdq[i](iJrow + 1, iJcol + 1 ) = 0.0;
			}
		}
	}
	q_ = q_orig;
}


void ArmModel::EvaluateWorld2JointTransf(Eigen::Matrix4d& wTj, int jointIndex) {
	for (int jointNumber = 1; jointNumber <= jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	wTj = wTei_[jointIndex - 1];
}

void ArmModel::EvaluateBase2JointTransf(Eigen::Matrix4d& bTj, int jointIndex) {

	Eigen::Matrix4d orig_wTb = wTb0_;
	wTb0_ = Eigen::Matrix4d::Identity();

	for (int jointNumber = 1; jointNumber <= jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	wTb0_ = orig_wTb;
	bTj = wTei_[jointIndex - 1];
}


void ArmModel::EvaluateWorld2JointJacobian(Eigen::MatrixXd& wJj, int jointIndex) {

	for (int jointNumber = 1; jointNumber <= jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	for (int jointNumber = jointIndex; jointNumber > 0; jointNumber--)
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

	for (int jointNumber = 1; jointNumber <= jointIndex; jointNumber++) {
		ForwardDirectGeometry(jointNumber);
	}
	for (int jointNumber = jointIndex; jointNumber > 0; jointNumber--)
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
	swap(first.wTt_, second.wTt_);
	swap(first.eTt_, second.eTt_);
	swap(first.w_r_et_, second.w_r_et_);

	swap(first.dJdq_, second.dJdq_);
	swap(first.Jpinv_, second.Jpinv_);
	swap(first.djdqJpinv_, second.djdqJpinv_);

	swap(first.wTe_, second.wTe_);
	swap(first.wJt_, second.wJt_);
	swap(first.arrayJ_, second.arrayJ_);
	swap(first.arrayQ_, second.arrayQ_);
	swap(first.I3_, second.I3_);
	swap(first.ZeroQ_, second.ZeroQ_);
}

}



