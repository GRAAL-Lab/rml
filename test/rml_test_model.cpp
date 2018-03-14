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
#include "test/youbot_vehiclemodel.h"
//#include "test/baxterLeft_armmodel.h"

using std::cout;
using std::endl;
using futils::PrettyPrint;

int main(int argc, char* argv[]){

	timeval t1, t2;
	futils::Timer myTimer;

	///////////////////////////////
	//////  ARM MODEL TEST   //////
	///////////////////////////////

	std::cout << std::endl << tc::yel << "### Arm Model Test ###" << tc::none << std::endl;

	int numJoints(0);
	double elapsed_Timer(0);

	//std::shared_ptr<rml::BaxterLeftArmModel> baxterAM = std::make_shared<rml::BaxterLeftArmModel>();
	std::shared_ptr<rml::YouBotArmModel> youbotAM = std::make_shared<rml::YouBotArmModel>();
	std::shared_ptr<rml::ArmModel> armModel = std::make_shared<rml::ArmModel>();

	armModel = youbotAM;
	numJoints = armModel->GetNumJoints();

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

	Eigen::MatrixXd q_0 = Eigen::MatrixXd(numJoints, 1);
	rml::Double2Matrix(q_0_doub, numJoints, 1, q_0);
	PrettyPrint(q_0.transpose(),"q_0");

	myTimer.Start();
	armModel->SetJointsPosition(q_0);
	dJdq_NEW = armModel->GetdJdq();
	elapsed_Timer = myTimer.Lap()*1000;

	std::cout << "elapsed_Timer=" << elapsed_Timer << "ms" << std::endl;

	for (int i = 0; i < numJoints - 1; ++i) {
		cout << i << "_NEW_";
		PrettyPrint(dJdq_NEW.at(i),"dJdq:");
		cout << "------------------------------------------------------------" << endl;
	}

	for (int i = 0; i < youbotAM->GetNumJoints(); ++i) {
		PrettyPrint(youbotAM->GetLink(i).GetJointLimitsMin(),"GetJointLimitsMin()");
		PrettyPrint(youbotAM->GetLink(i).GetJointLimitsMax(),"GetJointLimitsMax()");
	}

	////////////////////////////////////////////////////////////

	std::shared_ptr<rml::YouBotVehicleModel> youbotVM = std::make_shared<rml::YouBotVehicleModel>();
	std::shared_ptr<rml::RobotModel> robotModel = std::make_shared<rml::RobotModel>();

	int armIndex1 = robotModel->LoadArm(youbotAM, Eigen::TransfMatrix());
	int armIndex2 = robotModel->LoadArm(youbotAM, Eigen::TransfMatrix());
	robotModel->LoadVehicle(youbotVM);

	PrettyPrint(robotModel->GetVehicleJacobian_ToolFrame(armIndex1), "GetVehicleJacobianTF(1)");
	PrettyPrint(robotModel->GetArmJacobian_ToolFrame(armIndex1), "GetArmJacobianTF(1)");
	PrettyPrint(robotModel->GetArmJacobian_ToolFrame(armIndex2), "GetArmJacobianTF(2)");

	double mu;
	PrettyPrint(robotModel->GetArmJacobian_JointControl(armIndex1),"GetArmJacobian_JointControl()");
	PrettyPrint(robotModel->GetArmJacobian_Manipulability(armIndex1, mu),"GetArmJacobian_Manipulability()");
	PrettyPrint(robotModel->GetVehicleJacobian(),"GetVehicleJacobian()");

	return 0;
}



