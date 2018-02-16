/*
 * dJdq_test.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: francesco
 */

#include <cstdio>
#include <iostream>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include <fstream>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <iomanip>

#include "baxterLeft_armmodel.h"
#include "youbot_armmodel.h"
#include "futils.h"

using std::vector;
using std::cout;
using std::endl;

double TimeDiff(timeval t1, timeval t2) {
	double t;
	t = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	t += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
	return t;
}

int main(int argc, char **argv) {

	if(argc<2){
		printf("%s** ERROR **\n"
				"argv[1]: [baxter|youbot]%s\n", tc::red, tc::none);
		exit(EXIT_FAILURE);
	}

	std::string robotName(argv[1]);
	/**
	 * nTests is the number of test we want to perform (the timing will be an average)
	 */
	int nTests(1E0), numJoints(0);
	timeval t1, t2;
	double elapsed_OLD(0), elapsed_NEW(0);

	/**
	 *  Timer usage example:
	 *
	 *  gettimeofday(&t1, NULL);
	 *  // do operations to time //
	 *  gettimeofday(&t2, NULL);
	 *  d = TimeDiff(t1, t2);
	 */

	std::shared_ptr<CTRL::BaxterLeftArmModel> baxterAM = std::make_shared<CTRL::BaxterLeftArmModel>();
	std::shared_ptr<CTRL::YouBotArmModel> youbotAM = std::make_shared<CTRL::YouBotArmModel>();


	std::shared_ptr<CTRL::ArmModel> armModel = std::make_shared<CTRL::ArmModel>();

	if(robotName == "baxter"){
		armModel = baxterAM;

	}else if (robotName == "youbot"){
		armModel = youbotAM;
	}else{
		std::cout << tc::redL << "Unrecognized robot name, quitting..." << tc::none << std::endl;
		exit(0);
	}

	numJoints = armModel->GetNumJoints();
	armModel->InitMatrix();
	armModel->SelectModelRepresentation(CTRL_MODEL_BACKWARDFORWARD);

	cout << tc::magL << "*dJdq Test*" << tc::none << std::endl;
	cout << "numJoints=" << armModel->GetNumJoints() << endl;
	CMAT::Matrix* dJdq_OLD = new CMAT::Matrix[numJoints];
	CMAT::Matrix* dJdq_NEW = new CMAT::Matrix[numJoints];
	for (int var = 0; var < numJoints; ++var) {
		dJdq_OLD[var] = CMAT::Matrix::Zeros(6,numJoints);
		dJdq_NEW[var] = CMAT::Matrix::Zeros(6,numJoints);
	}
	CMAT::Matrix wJt_0, wJt_dQ;
	CMAT::TransfMatrix wTt_0, wTt_dQ;
	CMAT::Matrix zeroQ = CMAT::Matrix::Zeros(numJoints, 1);
	double q_0_generic[7] = { 0.0, 0.5, 1.0, 0.7, 0.3, 0.0, 0.0 }, q_0_doub[numJoints];
	if(numJoints < 7){
	    for(int i=0; i<7; i++){
	        q_0_doub[i] = q_0_generic[i];
	    }
	}else{
	    std::cerr << "Max num joint exceeded, exiting" << std::endl;
	    exit(0);
	}
	//double q_0_doub[numJoints] = { 0.0, 0.5, 1.0, 0.7, 0.3 };
	CMAT::Matrix q_0 = CMAT::Matrix(numJoints, 1, q_0_doub);
	q_0.Transpose().PrintMtx("q_0");
	CMAT::Matrix dQ, qVar;
	double delta_q = 1E-6;

	for (int n = 0; n < nTests; ++n) {
		/**
		 *  ========= OLD precomputed dJdq  =========
		 */
		gettimeofday(&t1, NULL);
		armModel->SetJointPosition(q_0);
		armModel->EvaluatedJdq(dJdq_OLD);
		gettimeofday(&t2, NULL);
		elapsed_OLD += TimeDiff(t1, t2);

		/**
		 * ========= NEW Numerical Computation =========
		 */
		gettimeofday(&t1, NULL);
		armModel->SetJointPosition(q_0);
		//armModel->EvaluatewJt(wJt_0);
		//wJt_0.PrintMtx("wJt_0");
		armModel->EvaluatedJdqNumeric(dJdq_NEW);
		gettimeofday(&t2, NULL);
		elapsed_NEW += TimeDiff(t1, t2);

		for (int i = 0; i < numJoints - 1; ++i) {
			cout << i << "_";
			dJdq_OLD[i].PrintMtx("dJdq:");
			cout << i << "_NEW_";
			dJdq_NEW[i].PrintMtx("dJdq:");
			/*cout << i << "_NEW_FUNC";
			dJdq_NEW_FUNC[i].PrintMtx("dJdq:");*/
			cout << "------------------------------------------------------------" << endl;
		}
	}

	std::cout << std::fixed;
	std::cout << std::setprecision(4);
	std::cout << "dJdq_OLD time avg on " << nTests << " tests: "
			<< elapsed_OLD/static_cast<double>(nTests) << "ms " << "(tot " << elapsed_OLD << "ms)" << std::endl;
	std::cout << "dJdq_NEW time avg on " << nTests << " tests: "
			<< elapsed_NEW/static_cast<double>(nTests) << "ms " << "(tot " << elapsed_NEW << "ms)" << std::endl;

	delete[] dJdq_NEW, dJdq_OLD;
}
