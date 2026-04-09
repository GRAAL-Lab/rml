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

std::string robotID = "youbot";

// Base to tool transformation matrix
Eigen::TransformationMatrix bTt;

// Base to goal transformation matrix
Eigen::TransformationMatrix bTg;

// **********************************************************
// ***************** ODE Simulator Function *****************
void single_arm_sim(const state_type& q, state_type& dqdt, double t)
{

    //! Update Control state and compute new control Command
    if ((t - t_old) > 0.01 || (t - t_old) < 0.0) {
        cout << "***** Solver: " << t << std::endl;

        for (int i = 0; i < numJoints; i++)
            qfb_single(i) = q[i];

        armModel->JointsPosition(qfb_single);
        armModel->JointsVelocity(qdotcontrol_single);

        rml::RegularizationData regData;

        std::string eeFrameID = armModel->GetEndEffectorFrameID();

        Eigen::MatrixXd JPinv;
        try{
            JPinv = rml::RegularizedPseudoInverse(armModel->Jacobian(eeFrameID), regData);
        } catch(rml::WrongFrameException& e){
            cout << "Exception in RegularizedPseudoInverse: " << e.how() << endl;
        }
        
        // Compute Control Command
        // bTg is defined outside this function as goal pose
        bTt = armModel->TransformationMatrix(armModel->GetEndEffectorFrameID());
        Eigen::Vector6d cart_err = rml::CartesianError(bTt, bTg);
        Eigen::Vector6d x_dot = cart_err * 1.0; // P controller with gain 1.0

        // Compute joint velocities
        qdotcontrol_single = JPinv * x_dot;

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

    auto rigidBodyFramesIDs = armModel->GetRigidBodyFrameIDs();
    cout << "Rigid Body Frame IDs size: " << rigidBodyFramesIDs.size() << endl;
    for (const auto& id : rigidBodyFramesIDs) {
        cout << "Rigid Body Frame ID: " << id << endl;
    }

    auto jointFramesIDs = armModel->GetJointFrameIDs();
    cout << "Joint Frame IDs size: " << jointFramesIDs.size() << endl;
    for (const auto& id : jointFramesIDs) {
        cout << "Joint Frame ID: " << id << endl;
    }

    auto jacobianFramesIDs = armModel->GetJacobianFrameIDs();
    cout << "Jacobian Frame IDs size: " << jacobianFramesIDs.size() << endl;
    for (const auto& id : jacobianFramesIDs) {
        cout << "Jacobian Frame ID: " << id << endl;
    }

    numJoints = armModel->NumJoints();
    std::cout << "Number of Joints: " << numJoints << std::endl;

    // Control Vectors Initialization
    qfb_single = qdotcontrol_single = q_single_final = Eigen::VectorXd::Zero(numJoints);

    // YouBot Q_unfolded
    init_q_single = { { 0.011, 0.11, -1.4, -0.11, 1.57 } };

    // Save File
    std::string DataLogPathStr = futils::get_selfpath() + "/robot_sim_log_" + futils::GetCurrentDateFormatted() + ".txt";
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
    armModel->JointsPosition(qfb_single);
    armModel->JointsVelocity(qdotcontrol_single);

    /** Generating a goal 0.1 meters forward in the x direction w.r.t to the EE */
    bTt = armModel->TransformationMatrix(armModel->GetEndEffectorFrameID());
    Eigen::TransformationMatrix bTg = bTt;
    bTg.TranslationVector(bTg.TranslationVector() + Eigen::Vector3d(0.1, 0.0, 0.0));

    Eigen::Vector6d goal = bTg.ToVector();
    cout << "goal Pose: ";
    fileLog << "goal Pose: ";
    for (int i = 0; i < 6; i++) {
        cout << goal[i] << " ";
        fileLog << goal[i] << " ";
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

    auto rhs_wrapper = [](const state_type& q, state_type& dqdt, double t) { single_arm_sim(q, dqdt, t); };
    auto obs = [](const state_type& q, const double t) { write_single_arm_sim(q, t); };

    // boost::numeric::odeint::integrate( std::bind(&simulator_class::single_arm_sim, std::ref(*this), PH::_1,PH::_2,PH::_3 ),
    //                                      init_q_single , start_time ,end_time , step_time , write_single_arm_sim );
    
    boost::numeric::odeint::integrate(rhs_wrapper, init_q_single, start_time, end_time, step_time, obs);

    return 0;
}
