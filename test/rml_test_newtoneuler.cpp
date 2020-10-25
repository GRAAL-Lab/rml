/*
 *  rml_test_newton_euler.cpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Francesco Wanderlingh
 *
 *  This executable tests the model functionalities of the Robotic Mathematical Library (RML)
 */

#include "test/rml_test_defines.h"
#include "test/twolinks_armmodel.h"
#include "test/youbot_armmodel.h"
#include "test/youbot_vehiclemodel.h"

using futils::PrettyPrint;

Eigen::Matrix3d CuboidInertiaAboutCOM(const double mass, const Eigen::Vector3d& dims);
void saveSimToFile(double time[], int simNSteps, std::vector<Eigen::VectorXd>& q_h, int numJoints);

int main(int argc, char* argv[])
{
    std::cout << std::endl
              << tc::yellow << "### Newton Euler Test ###" << tc::none << std::endl;

    int numJoints(0);
    double elapsed_Timer(0);

    std::shared_ptr<rml::YouBotArmModel> armModel = std::make_shared<rml::YouBotArmModel>("youbotArm");
    Eigen::TransformationMatrix world_T_vehicle;

    /// Jacobian
    Eigen::Matrix6d J_ASV;
    J_ASV.setIdentity();
    std::shared_ptr<rml::RobotModel> robotModel = std::make_shared<rml::RobotModel>(world_T_vehicle, "youbotVeh", J_ASV);

    numJoints = armModel->NumJoints();

    vector<Eigen::Vector3d> linkDim(numJoints);
    double linkMass;
    Eigen::Vector3d CoM;
    Eigen::Matrix3d Inertia;

    /// Getting link length information from links transformations
    for (int i = 0; i < numJoints - 1; ++i) {
        linkDim.at(i) = armModel->Link(i + 1).BaseTransf().TranslationVector();
    }
    linkDim.at(numJoints - 1) = armModel->TransformationMatrix("youbotArm_Joint_4").TranslationVector();

    /// Populating Link Physical Properties
    for (int i = 0; i < numJoints; ++i) {
        linkMass = 0.2;
        CoM = linkDim.at(i) / 2.0;
        Inertia = CuboidInertiaAboutCOM(linkMass, linkDim.at(i));

        armModel->Link(i).SetDynamicProperties(linkMass, linkDim.at(i), CoM, Inertia);
    }

    robotModel->LoadArm(armModel, Eigen::TransformationMatrix());

    std::cout << " - Model variables have been loaded - " << std::endl;

    std::string toolID = "toolID";

    armModel->AttachRigidBodyFrame(toolID, armModel->ID() + rml::FrameID::Joint + std::to_string(armModel->NumJoints() - 1), Eigen::TransformationMatrix());

    rml::NewtonEuler myNE(robotModel, armModel->ID(), toolID);

    ///--------------------------------------------------------///

    ///*** INITIALIZATION ***///
    Eigen::Vector3d zeroVect3 = Eigen::Vector3d::Zero();
    Eigen::VectorXd qZeroVec = Eigen::VectorXd::Zero(numJoints);

    Eigen::VectorXd q, q_dot, q_ddot, m_tilde, m_bar;
    Eigen::MatrixXd A;
    q = qZeroVec;
    q_dot = qZeroVec;
    q_ddot = qZeroVec;

    m_bar = qZeroVec;
    A = Eigen::MatrixXd::Zero(numJoints, numJoints);

    double q_0[7] = { 0, 2.0 / 4.0 * M_PI, 0, 0, 0, 0, 0 };

    rml::Double2Vector(q_0, numJoints, q);
    q_dot.setZero();

    // Simulation parameters
    double duration = 3.0; // Total seconds of simulation in SECONDS
    double dt = 0.001; // Time step in SECONDS
    int t_steps = duration / dt;

    std::cout << "Simulation steps: " << t_steps << std::endl;

    double t[t_steps];
    t[0] = 0;
    for (int i = 1; i < t_steps; ++i) {
        t[i] = t[i - 1] + dt;
    }

    vector<Eigen::VectorXd> q_h(t_steps), q_dot_h(t_steps);

    futils::Dotter myDotter(2);
    futils::Percentage myPerc(t_steps);

    std::cout << " - Simulation variables have been set... Starting SIM! - " << std::endl
              << std::endl;

    rml::RegularizationData mySVD;
    mySVD.params.lambda = 0.01;
    mySVD.params.threshold = 0.0001;

    futils::Timer myTimer;
    myTimer.Start();
    double printInterval = 0.05;

    /****************************************************************
	 *                          MAIN LOOP                           *
	 ****************************************************************/
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

        robotModel->Arm(armModel->ID())->JointsPosition(q);
        robotModel->Arm(armModel->ID())->JointsVelocity(q_dot);
        robotModel->Arm(armModel->ID())->JointsAcceleration(q_ddot);
        std::cout << "CoriolisGravityExternalForcesEffect" << std::endl;
        //std::cout << "------ Getting coriolis, gravity and external forces effect ------" << std::endl;
        m_tilde = myNE.CoriolisGravityExternalForcesEffect();

        //std::cout << "------ Getting Inertia matrix ------" << std::endl;
        A = myNE.InertiaMatrix();

        // std::cout << "------ Getting qddot ------" << std::endl;
        // Evaluating q_ddot by inverting the dynamic equation of motion for manipulators
        q_ddot = -1.0 * rml::RegularizedPseudoInverse(A, mySVD) * (m_tilde);

        //std::cout << "------ Integrating ------" << std::endl;
        // Integrating q_ddot to find q
        q_dot = q_dot + q_ddot * dt;
        q = q + q_dot * dt;

        myPerc();

        /*if (myTimer.GetCurrentLapTime() > printInterval) {
			myTimer.Lap();

			std::cout << std::endl;
            //futils::PrettyPrint(robotModel->Arm(armIndex)->GetJointsPosition().transpose(), "q in model");
			PrettyPrint(rml::RegularizedPseudoInverse(A, mySVD), "RegPinv(A)");

			myNE.PrintVars();
		}*/
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

    return Inertia;
}

void saveSimToFile(double time[], int simNSteps, std::vector<Eigen::VectorXd>& q_h, int numJoints)
{
    std::string save_path = futils::get_selfpath();
    save_path = save_path + "/simData_" + futils::GetCurrentDateFormatted();
    std::cout << tc::cyanL << "Saving sim data in: " << save_path << tc::none << std::endl;
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
            std::cerr << tc::red << "Exception opening/reading file\n"
                      << tc::none;
        }
    } catch (std::ofstream::failure& e) {
        std::cerr << tc::red << "Exception opening/reading file\n"
                  << tc::none;
    }
}
