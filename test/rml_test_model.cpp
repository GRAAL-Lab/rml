/*
 *  rml_test.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 *
 *  This executable tests the model functionalities of the Robotic Mathematical Library (RML)
 */

#include "test/rml_test_defines.h"
#include "test/youbot_armmodel.h"
#include "test/baxterLeft_armmodel.h"

using std::cout;
using std::endl;
using futils::PrettyPrint;

int main(int argc, char* argv[]){

	timeval t1, t2;

	///////////////////////////////
	//////  ARM MODEL TEST   //////
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### Arm Model Test ###" << tc::none << std::endl;

	int numJoints(0);
	double elapsed_OLD(0), elapsed_NEW(0);

	std::shared_ptr<rml::BaxterLeftArmModel> baxterAM = std::make_shared<rml::BaxterLeftArmModel>();
	std::shared_ptr<rml::YouBotArmModel> youbotAM = std::make_shared<rml::YouBotArmModel>();
	std::shared_ptr<rml::ArmModel> armModel = std::make_shared<rml::ArmModel>();

	armModel = youbotAM;

	numJoints = armModel->GetNumJoints();
	armModel->InitMatrix();

	cout << tc::magL << "*dJdq Test*" << tc::none << std::endl;
	cout << "numJoints=" << armModel->GetNumJoints() << endl;
	std::vector<Eigen::MatrixXd> dJdq_NEW(numJoints, Eigen::MatrixXd::Zero(6,numJoints));


	Eigen::MatrixXd zeroQ = Eigen::MatrixXd::Zero(numJoints, 1);
	double q_0_generic[7] = { 0.0, 0.5, 1.0, 0.7, 0.3, 0.0, 0.0 }, q_0_doub[numJoints];

	if(numJoints <= 7){
		for(int i=0; i<7; i++){
			q_0_doub[i] = q_0_generic[i];
		}
	}else{
		std::cerr << "Max num joint exceeded, exiting" << std::endl;
		exit(0);
	}

	Eigen::MatrixXd q_0 = Eigen::MatrixXd(numJoints, 1);//, q_0_doub);
	rml::Double2Matrix(q_0_doub, numJoints, 1, q_0);
	PrettyPrint(q_0.transpose(),"q_0");

	gettimeofday(&t1, NULL);
	armModel->SetJointPosition(q_0);

	dJdq_NEW = armModel->GetdJdq();
	gettimeofday(&t2, NULL);
	elapsed_NEW += TimeDiff(t1, t2);

	for (int i = 0; i < numJoints - 1; ++i) {
		cout << i << "_NEW_";
		PrettyPrint(dJdq_NEW.at(i),"dJdq:");
		cout << "------------------------------------------------------------" << endl;
	}







	return 0;
}



