/*
 *  rml_test.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 *
 *  This executable tests all the functionalities of the Robotic Mathematical Library (RML)
 */

#include "test/rml_test_defines.h"

using std::cout;
using std::endl;
using futils::PrettyPrint;

int main(int argc, char* argv[]){

	const int m = 4, n = 4;

	Eigen::MatrixXd A(m, n), U(m, m), S(m, n), V(n, n);
	timeval t1, t2;
	double d;

	A.setRandom();
	U.setRandom();
	S.setRandom();
	V.setRandom();

	///////////////////////////////
	///    MATRIX OPERATIONS    ///
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### MATRIX OPERATIONS Test ###" << tc::none << std::endl;

	S = rml::RightJuxtapose(A,U);
	V = rml::UnderJuxtapose(A,U);
	PrettyPrint(A, "A");
	PrettyPrint(U, "U");
	PrettyPrint(S, "rml::RightJuxtapose(A,U)");
	PrettyPrint(V, "rml::UnderJuxtapose(A,U)");

	Eigen::Vector6d vect6;
	Eigen::Vector3d vect3_1, vect3_2;
	vect3_1.setConstant(1);
	vect3_2.setConstant(2);
	vect6.setZero();
	PrettyPrint(vect6.transpose(), "vect6'");
	PrettyPrint(vect3_1.transpose(), "vect3_1'");
	PrettyPrint(vect3_2.transpose(), "vect3_2'");

	rml::SetFirstVect3(vect6, vect3_1);
	PrettyPrint(vect6.transpose(), "vect6' after SetFirstVect3()");
	rml::SetSecondVect3(vect6, vect3_2);
	PrettyPrint(vect6.transpose(), "vect6' after SetSecondVect3()");

	Eigen::Matrix4d AtMat = A;

	PrettyPrint(rml::GetRotMatrix(AtMat), "rml::GetRotMatrix(A)");
	PrettyPrint(rml::GetTrasl(AtMat), "rml::GetTrasl(A)");


	///////////////////////////////
	//////     PINV TEST     //////
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### PINV Test (with timings) ###" << tc::none << std::endl;

	Eigen::MatrixXd Avar, Apinv;
	int iterations = 1000;
	int rowMinSize = 4, colMinSize = 4;
	int rowMaxSize = 12, colMaxSize = 12;
	int step = 4;

	TimeResults timings;
	PinvSpecs pinvSpecs;
	pinvSpecs.SVDparams.threshold = 0.01;
	pinvSpecs.SVDparams.lambda = 0.0001;

	for (int rows = rowMinSize; rows <= rowMaxSize; rows += step) {
		for (int cols = colMinSize; cols <= colMaxSize; cols += step) {

			pinvSpecs.nRows = rows;
			pinvSpecs.nCols = cols;
			PseudoInverseTest(iterations, Avar, pinvSpecs, Apinv, timings);
		}
	}

	Apinv = rml::RegularizedPseudoInverse(A, pinvSpecs.SVDparams.threshold, pinvSpecs.SVDparams.lambda);
	PrettyPrint((A * Apinv), "A * Apinv (4x4)");

	///////////////////////////////
	//////     SVD TEST      //////
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### SVD Test ###" << tc::none << std::endl;

	for (int i = 0; i < A.rows(); i++){
		for (int j = 0; j < A.cols(); j++){
			A(i, j) = i * A.cols() + j;
		}
	}

	rml::SVD(A, U, S, V);

	PrettyPrint(A, "A");
	PrettyPrint(U, "U");
	PrettyPrint(S, "S");
	PrettyPrint(V, "V");

	Eigen::MatrixXd A_usv = U * S * V.transpose();
	PrettyPrint(A_usv, "A as the result of: A = U*S*V'");

	///////////////////////////////
	//////  ARM MODEL TEST   //////
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### Arm Model Test ###" << tc::none << std::endl;

	int numJoints(0);
	double elapsed_OLD(0), elapsed_NEW(0);

	//std::shared_ptr<CTRL::BaxterLeftArmModel> baxterAM = std::make_shared<CTRL::BaxterLeftArmModel>();
	//std::shared_ptr<CTRL::YouBotArmModel> youbotAM = std::make_shared<CTRL::YouBotArmModel>();

	std::shared_ptr<rml::ArmModel> armModel = std::make_shared<rml::ArmModel>();

	//armModel = youbotAM;

	numJoints = armModel->GetNumJoints();
	armModel->InitMatrix();

	cout << tc::magL << "*dJdq Test*" << tc::none << std::endl;
	cout << "numJoints=" << armModel->GetNumJoints() << endl;
	//std::vector<Eigen::MatrixXd> dJdq_OLD(numJoints, Eigen::MatrixXd::Zero(6,numJoints));
	std::vector<Eigen::MatrixXd> dJdq_NEW(numJoints, Eigen::MatrixXd::Zero(6,numJoints));


	Eigen::MatrixXd wJt_0, wJt_dQ;
	Eigen::Matrix4d wTt_0, wTt_dQ;
	Eigen::MatrixXd zeroQ = Eigen::MatrixXd::Zero(numJoints, 1);
	double q_0_generic[7] = { 0.0, 0.5, 1.0, 0.7, 0.3, 0.0, 0.0 }, q_0_doub[numJoints];

	if(numJoints < 7){
		for(int i=0; i<7; i++){
			q_0_doub[i] = q_0_generic[i];
		}
	}else{
		std::cerr << "Max num joint exceeded, exiting" << std::endl;
		exit(0);
	}

	Eigen::MatrixXd q_0 = Eigen::MatrixXd(numJoints, 1);//, q_0_doub);
	PrettyPrint(q_0.transpose(),"q_0");
	Eigen::MatrixXd dQ, qVar;
	double delta_q = 1E-6;


	gettimeofday(&t1, NULL);
	armModel->SetJointPosition(q_0);
	//armModel->EvaluatewJt(wJt_0);
	//wJt_0.PrintMtx("wJt_0");
	armModel->EvaluatedJdqNumeric(dJdq_NEW);
	gettimeofday(&t2, NULL);
	elapsed_NEW += TimeDiff(t1, t2);

	for (int i = 0; i < numJoints - 1; ++i) {

		cout << i << "_NEW_";
		PrettyPrint(dJdq_NEW[i],"dJdq:");
		cout << "------------------------------------------------------------" << endl;

	}



	return 0;
}



