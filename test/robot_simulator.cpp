#include <boost/numeric/odeint.hpp>
#include <fstream>
#include <iostream>
#include <vector>

#include "test/rml_test_defines.h"
#include "test/youbot_armmodel.h"

using std::cout;
using std::endl;
using std::vector;

std::ofstream fileLog;

typedef boost::array<double, 5> state_type;

Eigen::VectorXd qfb_single, qdotcontrol_single, q_single_final;
state_type init_q_single; //= { -0.086 , -1.009 , -1.134 , 1.957 , 0.662 , 1.019 , -0.494 }; // initial conditions
double t_old, simulationTime;
int numJoints;
std::shared_ptr<rml::ArmModel> armModel;
bool goalReached;
double goal[6];

// **********************************************************
// ***************** ODE Simulator Function *****************
void single_arm_sim(const state_type& q, state_type& dqdt, double t)
{

    //! Update Control state and compute new control Command
    if ((t - t_old) > 0.01 || (t - t_old) < 0.0) {
        cout << "************* solver: " << t << endl;

        for (int i = 0; i < numJoints; i++)
            qfb_single(i) = q[i];

        armModel->SetJointsPosition(qfb_single);
        armModel->SetJointsVelocity(qdotcontrol_single);

        ////TODO
        /// SET GOAL TO CARTERROR
        /// COMPUTE J_PSEUDOINVERSE
        /// COMPUTE REFERENCE

        rml::RegularizationData regData;
        Eigen::MatrixXd JPinv = rml::RegularizedPseudoInverse(armModel->GetJacobian(""), regData);
        //multiArmCtrl->ComputeControl();
        //qdotcontrol_single = multiArmCtrl->GetJointControl();

        t_old = t;
    }

    for (int i = 0; i < numJoints; i++)
        dqdt[i] = qdotcontrol_single(i);
};

// **********************************************************
// ***************** ODE Observer ***************************
void write_single_arm_sim(const state_type& q, const double t)
{

    //arm_wTt = multiArmCtrl->GetArmwTt();

    //! Show and Save Simulation
    for (int i = 0; i < numJoints; i++)
        fileLog << std::to_string(qfb_single(i)) << " ";
    fileLog << ", ";
    for (int i = 0; i < numJoints; i++)
        fileLog << std::to_string(qdotcontrol_single(i)) << " ";
    fileLog << "\n";

    if (goalReached == true) {
        simulationTime = t;
        for (int i = 0; i < numJoints; i++)
            q_single_final(i) = q[i];

        cout << "*******************" << endl;
        cout << "Successful Action!" << endl;
        cout << "Simulation Time: " << simulationTime << endl;
        cout << "Final q: ";
        for (int i = 0; i < q_single_final.rows(); i++)
            cout << q_single_final(i) << " ";
        cout << endl;
        //		return;
    }
};

int main(int, char**)
{

    double start_time(0.0), end_time(1.0);
    double step_time = 0.01;

    goalReached = false;

    std::shared_ptr<rml::YouBotArmModel> youbotAM = std::make_shared<rml::YouBotArmModel>("youbot");
    armModel = std::make_shared<rml::ArmModel>("genericAM");

    armModel = youbotAM;
    numJoints = armModel->GetNumJoints();
    qfb_single = qdotcontrol_single = q_single_final = Eigen::VectorXd::Zero(numJoints);

    // YouBot Q_unfolded
    init_q_single = { { 0.011, 0.11, -1.4, -0.11, 1.57 } };

    // Save File
    std::string DataLogPathStr = futils::get_selfpath() + "log_" + futils::GetCurrentDateFormatted() + ".txt";
    cout << "Saving to: " << DataLogPathStr << "\n";
    fileLog.open(DataLogPathStr.c_str(), std::ios::app);

    cout << "initial q: ";
    fileLog << "initial q: ";
    for (int i = 0; i < init_q_single.size(); i++) {
        fileLog << std::to_string(init_q_single[i]) << " ";
        cout << init_q_single[i] << " ";
    }
    fileLog << "\n";
    cout << endl;

    for (int i = 0; i < numJoints; i++)
        qfb_single[i] = init_q_single[i];

    // First model initialisation
    armModel->SetJointsPosition(qfb_single);
    armModel->SetJointsVelocity(qdotcontrol_single);

    Eigen::TransfMatrix bTt = armModel->GetBase2ToolTransf();
    Eigen::TransfMatrix bTg = bTt;
    bTg.SetTransl(bTg.GetTransl() + Eigen::Vector3d(0.1, 0.0, 0.0));
    Eigen::Vector6d cart_err = rml::CartesianError(bTt, bTg);

    cout << "goal Pose: ";
    fileLog << "goal Pose: ";
    for (int i = 0; i < 6; i++) {
        fileLog << goal[i] << " ";
        cout << goal[i] << " ";
    }
    fileLog << "\n";
    cout << endl;

    cout << "start time: " << start_time << endl;
    cout << "end time: " << end_time << endl;
    cout << "step time: " << step_time << endl;

    fileLog << "start time: " << std::to_string(start_time) << "\n";
    fileLog << "end time: " << std::to_string(end_time) << "\n";
    fileLog << "step time: " << std::to_string(step_time) << "\n";

    cout << "*********************" << endl;
    cout << "simulation start: " << endl;

    fileLog << "*********************"
            << "\n";
    fileLog << "simulation start: "
            << "\n";



    //multiArmCtrl->SetCartGoalEE(armIndex, goal);

    //auto rhs_wrapper = [this](const state_type& q, state_type& dqdt, double t) { single_arm_sim(q, dqdt, t); };
    //auto obs = [this](const state_type& q, const double t) { write_single_arm_sim(q, t); };
    auto rhs_wrapper = [](const state_type& q, state_type& dqdt, double t) { single_arm_sim(q, dqdt, t); };
    auto obs = [](const state_type& q, const double t) { write_single_arm_sim(q, t); };

    //			boost::numeric::odeint::integrate( std::bind(&simulator_class::single_arm_sim, std::ref(*this), PH::_1,PH::_2,PH::_3 ) , init_q_single , start_time ,end_time , step_time , write_single_arm_sim );
    boost::numeric::odeint::integrate(rhs_wrapper, init_q_single, start_time, end_time, step_time, obs);

    return 0;
}
