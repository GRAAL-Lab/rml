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

Eigen::Matrix3d CuboidInertiaAboutCOM(const double mass, const Eigen::Vector3d& dims);
void saveSimToFile(double time[], int simNSteps, std::vector<Eigen::VectorXd>& q_h, int numJoints);
std::string get_selfpath();
std::string getCurrentDateFormatted();

int main(int argc, char* argv[])
{

	timeval t1, t2;
	futils::Timer myTimer;

	std::cout << std::endl << tc::yel << "### Newton Euler Test ###" << tc::none << std::endl;

	int numJoints(0);
	double elapsed_Timer(0);

	std::shared_ptr<rml::YouBotArmModel> youbotAM = std::make_shared<rml::YouBotArmModel>();
	std::shared_ptr<rml::YouBotVehicleModel> youbotVM = std::make_shared<rml::YouBotVehicleModel>();
	std::shared_ptr<rml::RobotModel> robotModel = std::make_shared<rml::RobotModel>();

	numJoints = youbotAM->GetNumJoints();

	vector<Eigen::Vector3d> linkDim(numJoints);
	double linkMass;
	Eigen::Vector3d CoM;
	Eigen::Matrix3d Inertia;

	/// Getting link length information from links transformations
	for (int i = 0; i < numJoints - 1; ++i) {
		linkDim.at(i) = youbotAM->GetLink(i + 1).BaseTransf().GetTransl();
	}
	linkDim.at(numJoints - 1) = youbotAM->GeteTt().GetTransl();

	/// Populating Link Physical Properties
	for (int i = 0; i < numJoints; ++i) {
		linkMass = 1.0;
		CoM = linkDim.at(i) / 2.0;
		Inertia = CuboidInertiaAboutCOM(linkMass, linkDim.at(i));

		youbotAM->GetLink(i).SetPhysicalProperties(linkMass, linkDim.at(i), CoM, Inertia);
	}

	int armIndex = robotModel->LoadArm(youbotAM, Eigen::TransfMatrix());
	robotModel->LoadVehicle(youbotVM);

	std::cout << " - Model variables have been loaded - " << std::endl;

	rml::NewtonEuler myNE(robotModel, armIndex);

	///--------------------------------------------------------///



	///*** INITIALIZATION ***///
	Eigen::Vector3d zeroVect3 = Eigen::Vector3d::Zero();
	Eigen::VectorXd qZeroVec = Eigen::VectorXd::Zero(numJoints);

//	base.c = zeroVect3;
//	base.c_dot = zeroVect3;
//	base.omega = zeroVect3;
//	base.omega_dot = zeroVect3;

	Eigen::VectorXd q, q_dot, q_ddot, m_tilde, m_bar;
	Eigen::MatrixXd A;
	q = qZeroVec;
	q_dot = qZeroVec;
	q_ddot = qZeroVec;

	m_bar = qZeroVec;
	A = Eigen::MatrixXd::Zero(numJoints, numJoints);

	// TODO
	//double q_0[NUM_JOINTS] = { 0, 0 };
	double q_0[numJoints] = { 0, 3.0 / 4.0 * M_PI, 0, 0, 0 };

	rml::Double2Vector(q_0, numJoints, q);
	q_dot.setZero();

	// Simulation parameters
	double duration = 5.0; ///Total seconds of simulation
	double dt = 0.002; ///Time step
	int t_steps = duration / dt;

	cout << "Simulation steps: " << t_steps << endl;

	double t[t_steps];
	t[0] = 0;
	for (int i = 1; i < t_steps; ++i) {
		t[i] = t[i - 1] + dt;
	}

	vector<Eigen::VectorXd> q_h(t_steps), q_dot_h(t_steps);

	futils::Dotter myDotter(2);
	futils::Percentage myPerc(t_steps);

	std::cout << " - Simulation variables have been set... Starting SIM! - " << std::endl << std::endl;

	rml::SVDData mySVD;
	mySVD.params.lambda = 0.01;
	mySVD.params.threshold = 0.0001;

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

		//std::cout << "------ Getting MBar ------" << std::endl;

		myNE.GetMBar(q_dot, m_bar);

		//PrettyPrint(m_bar, "m_bar");

		//std::cout << "------ Getting A ------" << std::endl;

		myNE.GetA(A);

		//PrettyPrint(A, "A");
		//PrettyPrint(rml::RegularizedPseudoInverse(A, mySVD), "RegularizedPseudoInverse(A)");
		//PrettyPrint(rml::PseudoInverse(A), "PseudoInverse(A)");

		//exit(0);

		//std::cout << "------ Getting qddot ------" << std::endl;

		/// Evaluating q_ddot by inverting the dynamic equation of motion for manipulators
		q_ddot = -1.0 * rml::RegularizedPseudoInverse(A, mySVD) * m_bar;
		//q_ddot.PrintMtx("q_ddot:");

		//std::cout << "------ Integrating ------" << std::endl;
		/// Integrating q_ddot to find q
		q_dot = q_dot + q_ddot * dt;
		q = q + q_dot * dt;

		//q.Transpose().PrintMtx("q:");
		myPerc();
	}

	std::cout << std::endl;

	saveSimToFile(t, t_steps, q_h, numJoints);

	return 0;
}

Eigen::Matrix3d CuboidInertiaAboutCOM(const double mass, const Eigen::Vector3d& dims)
{

	Eigen::Matrix3d Inertia = Eigen::Matrix3d::Zero();

	double l = dims(0), w = dims(1), h = dims(2);

	if (l == 0) {
		l = 0.05;
	}

	if (w == 0) {
		w = 0.05;
	}

	if (h == 0) {
		h = 0.05;
	}

	Inertia(0, 0) = mass / 12 * (w * w + h * h);
	Inertia(1, 1) = mass / 12 * (l * l + h * h);
	Inertia(2, 2) = mass / 12 * (l * l + w * w);

	//Inertia.PrintMtx("Inertia: ");

	return Inertia;
}

void saveSimToFile(double time[], int simNSteps, std::vector<Eigen::VectorXd>& q_h, int numJoints)
{

	std::string save_path = get_selfpath();

	save_path = save_path + "/simData_" + getCurrentDateFormatted();

	cout << TC_CYAN << "Saving sim data in: " << save_path << TC_NONE << endl;

	std::ofstream results_file;

	try {
		results_file.open(save_path.c_str(), std::fstream::app);

		if (results_file.is_open()) {

			for (int i = 0; i < simNSteps; ++i) {
				results_file << time[i] << "\t";
				for (int j = 0; j < numJoints; ++j)
					results_file << q_h.at(i)(j) << "\t";
				results_file << "\n";
			}
			results_file.close();
		} else {
			std::cerr << TC_RED << "Exception opening/reading file\n" << TC_NONE;
		}
	} catch (std::ofstream::failure& e) {
		std::cerr << TC_RED << "Exception opening/reading file\n" << TC_NONE;
	}

}

std::string get_selfpath()
{
	char buff[2048];
	ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff) - 1);
	if (len != -1) {
		buff[len] = '\0';
		std::string path(buff);   ///Here the executable name is still in
		std::string::size_type t = path.find_last_of("/");   // Here we find the last "/"
		path = path.substr(0, t);                             // and remove the rest (exe name)
		return path;
	} else {
		printf("Cannot determine file path!\n");
		return std::string('\0');
	}
}

std::string getCurrentDateFormatted()
{

	std::time_t t = std::time(NULL);
	char mbstr[20];
	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
	std::string currentDate(mbstr);

	return currentDate;
}

