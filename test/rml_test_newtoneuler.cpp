/*
 *  rml_test_newton_euler.cpp
 *
 *  Created on: Mar 19, 2018
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




const int NUM_JOINTS = 2;
const int NUM_LINKS = 2;
//const double LINK_THICKNESS = 0.085; // Supposed thickness in meters
const double pi = 3.1415926535897932384626433832795;

Eigen::Matrix3d CuboidInertiaAboutCOM(const double mass, const Eigen::Vector3d& dims);
void saveSimToFile(double time[], int simNSteps, std::vector<Eigen::MatrixXd>& q_h, int numJoints);
std::string get_selfpath();
std::string getCurrentDateFormatted();



int main(int argc, char* argv[]){

	timeval t1, t2;
	futils::Timer myTimer;

	std::cout << std::endl << tc::yel << "### Newton Euler Test ###" << tc::none << std::endl;

	int numJoints(0);
	double elapsed_Timer(0);

	//std::shared_ptr<rml::BaxterLeftArmModel> baxterAM = std::make_shared<rml::BaxterLeftArmModel>();
	std::shared_ptr<rml::YouBotArmModel> youbotAM = std::make_shared<rml::YouBotArmModel>();
	std::shared_ptr<rml::ArmModel> armModel = std::make_shared<rml::ArmModel>();

	armModel = youbotAM;
	numJoints = armModel->GetNumJoints();

	vector<Eigen::Vector3d> linkDim(numJoints);
	double linkMass;
	Eigen::Vector3d CoM(numJoints);
	Eigen::Matrix3d Inertia(numJoints);

	/// Getting link length information from links transformations
	for (int i = 0; i < numJoints-1; ++i) {
		linkDim.at(i) = armModel->GetLink(i+1).BaseTransf().GetTransl();
	}
	linkDim.at(numJoints-1) = armModel->GeteTt().GetTransl();

	/// Populating Link Physical Properties
	for (int i = 0; i < numJoints; ++i) {
		linkMass = 1.0;
		CoM = linkDim.at(i) / 2;
		Inertia = CuboidInertiaAboutCOM(linkMass, linkDim.at(i));

		armModel->GetLink(i).SetPhysicalProperties(linkMass, linkDim.at(i), CoM, Inertia);
	}

	rml::NewtonEuler myNE(youbotAM);

	rml::NewtonEuler::BaseLinkData baseData;


	ArmDynamics myArmDyn(wTb0, biTri, eTt, links, jointTypes);

	///--------------------------------------------------------///

	double duration = 5.0; ///Total seconds of simulation
	double dt = 0.0001; ///Time step
	int t_steps = duration / dt;

	//cout<< "t_steps: " << t_steps << endl;

	double t[t_steps];
	t[0] = 0;
	for (int i = 1; i < t_steps; ++i) {
		t[i] = t[i - 1] + dt;
	}

	///*** INITIALIZATION ***///
	CMAT::Matrix zeroVect3 = CMAT::Matrix::Zeros(3, 1);
	CMAT::Matrix qZeroVec = CMAT::Matrix::Zeros(NUM_JOINTS, 1);

	base.c = zeroVect3;
	base.c_dot = zeroVect3;
	base.omega = zeroVect3;
	base.omega_dot = zeroVect3;

	q = qZeroVec;
	q_dot = qZeroVec;
	q_ddot = qZeroVec;

	m_bar = qZeroVec;
	A = CMAT::Matrix::Zeros(NUM_JOINTS);


	//double q_0[NUM_JOINTS] = { 0, 0 };
	double q_0[NUM_JOINTS] = { 0, 3.0 / 4.0 * pi };
	double q_dot_0[NUM_JOINTS] = { 0, 0 };

	q.CopyFrom(q_0);
	q_dot.CopyFrom(q_dot_0);

	vector<CMAT::Matrix> q_h(t_steps), q_dot_h(t_steps);

	FUTILS::Dotter myDotter(2);

	/********************************************************
	 *                      MAIN LOOP                       *
	 ********************************************************/
	/**
	 * Given the following equation for the dynamic of robotic manipulators:
	 *
	 *         A(q)*q_ddot + B(q_,q_dot)*q_dot + C(q) = m + m_hat
	 *
	 * I can evaluate the various A, B, C matrix by imposing q_dot, q_ddot
	 * equal to zero. After, I can compute q_ddot by inverting the above
	 * mentioned formula:
	 *
	 *           q_ddot = inv(A)*[ m + m_hat - (B*q_dot + C)]
	 */

	for (int i = 0; i < t_steps; ++i) {

		q_h.at(i) = q;

		myArmDyn.SetJointPosition(q);

		myArmDyn.GetMBar( baseData, q_dot, m_bar );

		myArmDyn.GetA( baseData, A );

		//A.PrintMtx("A");
		//exit(0);

		/// Evaluating q_ddot by inverting the dynamic equation of motion for manipulators
		q_ddot = -1.0 * A.RegPseudoInverse(0.0,0.0) * m_bar;
		//q_ddot.PrintMtx("q_ddot:");

		/// Integrating q_ddot to find q
		q_dot = q_dot + q_ddot*dt;
		q = q + q_dot*dt;

		//q.Transpose().PrintMtx("q:");

		myDotter();
	}

	saveSimToFile(t, t_steps, q_h, NUM_JOINTS);

	return 0;
}

Eigen::Matrix3d CuboidInertiaAboutCOM(const double mass, const Eigen::Vector3d& dims){

	Eigen::Matrix3d Inertia = Eigen::Matrix3d::Zero();

	double l = dims(1), w = dims(2), h = dims(3);

	if(l == 0){
	    l = 0.05;
	}

	if(w == 0){
	    w = 0.05;
	}

	if(h == 0){
	    h = 0.05;
	}

	Inertia(0,0) = mass/12*(w*w + h*h);
	Inertia(1,1) = mass/12*(l*l + h*h);
	Inertia(2,2) = mass/12*(l*l + w*w);

	//Inertia.PrintMtx("Inertia: ");

	return Inertia;
}

void saveSimToFile(double time[], int simNSteps, std::vector<Eigen::MatrixXd>& q_h, int numJoints){

	std::string save_path = get_selfpath();

	save_path = save_path + "/simData_" + getCurrentDateFormatted();

	cout << TC_CYAN << "Saving sim data in: " << save_path << TC_NONE << endl;

	std::ofstream results_file;

	try {
		results_file.open ( save_path.c_str(), std::fstream::app );

		if( results_file.is_open() ) {

			for(int i = 0; i < simNSteps; ++i){
				results_file << time[i] << "\t";
				for(int j = 0; j < numJoints; ++j) results_file <<  q_h.at(i)(j) << "\t";
				results_file << "\n";
			}
			results_file.close();
		}else{
			std::cerr << TC_RED << "Exception opening/reading file\n" << TC_NONE;
		}
	}catch ( std::ofstream::failure& e ) {
		std::cerr << TC_RED << "Exception opening/reading file\n" << TC_NONE;
	}

}

std::string get_selfpath() {
	char buff[2048];
	ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
	if (len != -1) {
		buff[len] = '\0';
		std::string path(buff);   ///Here the executable name is still in
		std::string::size_type t = path.find_last_of("/");   // Here we find the last "/"
		path = path.substr(0,t);                             // and remove the rest (exe name)
		return path;
	} else {
		printf("Cannot determine file path!\n");
		return std::string('\0');
	}
}


std::string getCurrentDateFormatted(){

	std::time_t t = std::time(NULL);
	char mbstr[20];
	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
	std::string currentDate(mbstr);

	return currentDate;
}




