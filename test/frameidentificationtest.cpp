#include "rml_internal/Futils.h"
#include "test/twolinks_armmodel.h"
#include <iostream>
#include <memory>
#include <rml/RML.h>
#include <string>


int main()
{
    ////ROBOT MODEL TEST
    std::string arm_id = "armID";
    std::string vehicle_id = "vehicleID";
    std::string joint_one_frame = arm_id + rml::FrameID::Joint + "1"; //armID+rml::FrameID::Body +rigid_body_frame_id;

    Eigen::TransfMatrix vTb;
    vTb(0, 0) = -1;
    vTb(0, 1) = 0;
    vTb(0, 2) = 0;
    vTb(0, 3) = 0.85;
    vTb(1, 0) = 0;
    vTb(1, 1) = -1;
    vTb(1, 2) = 0;
    vTb(1, 3) = 0;
    vTb(2, 0) = 0;
    vTb(2, 1) = 0;
    vTb(2, 2) = 1;
    vTb(2, 3) = 0.42;
    vTb(3, 0) = 0;
    vTb(3, 1) = 0;
    vTb(3, 2) = 0;
    vTb(3, 3) = 1;

    auto arm_model = std::make_shared<rml::TwoLinksArmModel>(rml::TwoLinksArmModel(arm_id));
    auto vehicle_model = std::make_shared<rml::VehicleModel>(rml::VehicleModel(vehicle_id));
    vehicle_model->SetJacobian(Eigen::MatrixXd::Identity(6, 6));
    auto robot_model = std::make_shared<rml::RobotModel>(rml::RobotModel());
    robot_model->LoadArm(arm_model, vTb);
    robot_model->LoadVehicle(vehicle_model);

    Eigen::VectorXd initial_tool_pos(2);
    initial_tool_pos << 0.011, 0.11; // -1.4, -0.11, 1.57;//, -1.4, -0.11, 1.57;
    robot_model->GetArm(arm_id)->SetJointsPosition(initial_tool_pos);


    std::cout << "frame ID: " << joint_one_frame << std::endl;
    std::cout << "Trasformata wTj\n" << robot_model->GetTransformation(joint_one_frame) << std::endl;
    std::cout << "J\n" << robot_model->GetCartesianJacobian(joint_one_frame, rml::VehicleFrame )<< std::endl;
    std::cout << "q\n" << robot_model->GetSystemPositionVector().transpose() << std::endl;

    return 0;
}
