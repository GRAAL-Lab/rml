#include "test/youbot_armmodel.h"
#include "test/twolinks_armmodel.h"
#include <iostream>
#include <memory>
#include "RML.h"
#include "rml_internal/Futils.h"
#include <string>

int main()
{
    //std::string ID="TestArm";
    //auto arm_model = std::make_shared<rml::YouBotArmModel>(rml::YouBotArmModel(ID));
    //Eigen::VectorXd initialFrameID::Jointpos(arm_model->GetNumJoints());
    //
    ////TEST TRANSFORMATION MATRICES AND JACOBIANS FOR JOINTS AND ARM
    //
    ////TRANSF MATRIX
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID+FrameID::Tool) << std::endl;
    ////std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    //
    //for (int i =0; i< arm_model->GetNumJoints(); i++)
    //{
    //    std::string matrixName=ID+FrameID::Joint+std::to_string(i);
    //    std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
    //    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    //}
    //
    ////JACOBIANS
    //std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID+FrameID::Tool) << std::endl;
    ////std::cout << "Jacobian via method\n " << arm_model->GetBase2ToolJacobian()<< std::endl;
    //
    //for (int i =0; i<arm_model->GetNumJoints(); i++)
    //
    //{
    //
    //    std::string matrixName=ID+FrameID::Joint+std::to_string(i);
    //    std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
    //    //std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    //}
    //
    //initialFrameID::Jointpos << 0.011, 0.11, -1.4, -0.11, 1.57;
    //arm_model->SetJointsPosition(initialFrameID::Jointpos);
    //
    //
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID+FrameID::Tool) << std::endl;
    ////std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    //for (int i =0; i< arm_model->GetNumJoints(); i++)
    //{
    //    std::string matrixName=ID+FrameID::Joint+std::to_string(i);
    //    std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
    //    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    //
    //}
    //
    ////JACOBIANS
    //std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID+FrameID::Tool) << std::endl;
    ////std::cout << "Jacobian via method \n" << arm_model->GetBase2ToolJacobian()<< std::endl;
    //
    //for (int i =0; i<arm_model->GetNumJoints(); i++)
    //
    //{
    //
    //    std::string matrixName=ID+FrameID::Joint+std::to_string(i);
    //    std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
    //    //std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    //}

    //TEST ATTCHED BODY FRAME TRANSF AND JACOBIANS
    //std::string rigid_body_frame_id("cameraFrame");
    //Eigen::TransfMatrix T;
    //T.SetTransl(Eigen::Vector3d(0.2,0.0,0.2));
    //arm_model->AddRigidBodyFrame(rigid_body_frame_id,2,T);
    //std::cout << "TransformationMatrix \n" << arm_model->GetTransformationMatrix(rigid_body_frame_id) << std::endl;
    ////std::cout << "TransfMatrix with method\n " << arm_model->GetAttachedBodyTransf(rigid_body_frame_id) << std::endl;
    //std::cout << "Jacobian\n" << arm_model->GetJacobian(rigid_body_frame_id) << std::endl;
    ////std::cout << "Jacobian with method \n " << arm_model->GetAttachedBodyJacobian(rigid_body_frame_id) << std::endl;
    //initialFrameID::Jointpos << 0.011, 0.11, -1.4, -0.11, 1.57;
    //arm_model->SetJointsPosition(initialFrameID::Jointpos);
    //std::cout << "TransformationMatrix \n" << arm_model->GetTransformationMatrix(rigid_body_frame_id) << std::endl;
    ////std::cout << "TransfMatrix with method\n " << arm_model->GetAttachedBodyTransf(rigid_body_frame_id) << std::endl;
    //std::cout << "Jacobian\n" << arm_model->GetJacobian(rigid_body_frame_id) << std::endl;
    //std::cout << "Jacobian with method \n " << arm_model->GetAttachedBodyJacobian(rigid_body_frame_id) << std::endl;

    //TEST FOR VEHICLE JACOBIAN, TRANSFORMATION MATRIX and ATTACHED BODY TRANSFORMATION
    //std::string vehicle_id("vehicleTest");
    //auto vehicle_model = std::make_shared<rml::VehicleModel>(rml::VehicleModel(vehicle_id));
    //vehicle_model->SetJacobian(Eigen::MatrixXd::Identity(6, 6));
    //Eigen::Vector6d vehicle_position;
    //vehicle_model->SetPositionOnInertial(vehicle_position.setZero());
    //std::cout << "Jacobians " << vehicle_model->GetJacobian(vehicle_id) << std::endl;
    ////std::cout << "Jacobian via method " << vehicle_model->GetvJv() << std::endl;
    //std::cout << "Transformation " << vehicle_model->GetTransfMatrix(vehicle_id) << std::endl;
    ////std::cout << "Transfromation via merthod " << vehicle_model->GetwTv() << std::endl;
    //vehicle_position.SetSecondVect3(Eigen::Vector3d(0.1,0.1,0.1));
    //vehicle_position.SetFirstVect3(Eigen::Vector3d(0.0,0.0,0.0));
    //vehicle_model->SetPositionOnInertial(vehicle_position);
    //std::cout << "Jacobians " << vehicle_model->GetJacobian(vehicle_id) << std::endl;
    ////std::cout << "Jacobian via method " << vehicle_model->GetvJv() << std::endl;
    //std::cout << "Transformation " << vehicle_model->GetTransfMatrix(vehicle_id) << std::endl;
    ////std::cout << "Transfromation via merthod " << vehicle_model->GetwTv() << std::endl;

    ////TEST ATTCHED BODY FRAME TRANSF AND JACOBIANS
    //std::string rigid_body_frame_id("cameraFrame");
    //std::string rigid_body_frame_id_for_look_up=vehicle_id+rigid_body_frame_id;
    //Eigen::TransfMatrix T;
    //T.SetTransl(Eigen::Vector3d(0.2,0.0,0.2));
    //vehicle_model->AddRigidBodyFrame(rigid_body_frame_id,T);
    //std::cout << "TransformationMatrix \n" << vehicle_model->GetTransfMatrix(rigid_body_frame_id_for_look_up) << std::endl;
    //std::cout << "TransfMatrix with method\n " << vehicle_model->GetAttachedBodyTransf(rigid_body_frame_id_for_look_up) << std::endl;
    //std::cout << "Jacobian\n" << vehicle_model->GetJacobian(rigid_body_frame_id_for_look_up) << std::endl;
    //std::cout << "Jacobian with method \n " << vehicle_model->GetAttachedBodyJacobian(rigid_body_frame_id_for_look_up) << std::endl;
    //vehicle_position.SetSecondVect3(Eigen::Vector3d(0.1,0.0,0.0));
    //vehicle_position.SetFirstVect3(Eigen::Vector3d(0.0,0.0,0.0));
    //vehicle_model->SetPositionOnInertial(vehicle_position);
    //std::cout << "TransformationMatrix \n" << vehicle_model->GetTransfMatrix(rigid_body_frame_id_for_look_up) << std::endl;
    //std::cout << "TransfMatrix with method\n " << vehicle_model->GetAttachedBodyTransf(rigid_body_frame_id_for_look_up) << std::endl;
    //std::cout << "Jacobian\n" << vehicle_model->GetJacobian(rigid_body_frame_id_for_look_up) << std::endl;
    //std::cout << "Jacobian with method \n " << vehicle_model->GetAttachedBodyJacobian(rigid_body_frame_id_for_look_up) << std::endl;

    ////ROBOT MODEL TEST
    std::string arm_id = "TestArm";
    auto arm_model = std::make_shared<rml::TwoLinksArmModel>(rml::TwoLinksArmModel(arm_id));
    std::string vehicle_id("vehicleTest");
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
    auto vehicle_model = std::make_shared<rml::VehicleModel>(rml::VehicleModel(vehicle_id));
    vehicle_model->SetJacobian(Eigen::MatrixXd::Identity(6, 6));
    auto robot_model = std::make_shared<rml::RobotModel>(rml::RobotModel());
    robot_model->LoadVehicle(vehicle_model);
    robot_model->LoadArm(arm_model, vTb);
    try {
        robot_model->LoadArm(arm_model,vTb);
    } catch (rml::ExceptionWithID&e) {
        std::cerr<<e.what()<<std::endl;
        std::cerr<<e.who()<<std::endl;
    }
    Eigen::VectorXd initial_tool_pos(arm_model->GetNumJoints());
    initial_tool_pos << 0.011, 0.11;//, -1.4, -0.11, 1.57;
    arm_model->SetJointsPosition(initial_tool_pos);
    std::string rigid_body_frame_id("CameraFrame");
    Eigen::TransfMatrix T;
    T.SetTransl(Eigen::Vector3d(0.2, 0.0, 0.2));
    arm_model->AddRigidBodyFrame(rigid_body_frame_id, arm_model->GetNumJoints() - 1, T);
    vehicle_model->AddRigidBodyFrame(rigid_body_frame_id, T);
    std::string joint_one_frame =arm_model->GetID() + rml::FrameID::Joint + "1";
    std::string tool_frame = arm_model->GetID() + rml::FrameID::Tool;
    std::string vehicle_frame = vehicle_model->GetID();
    std::string rigid_body_arm_frame =  arm_model->GetID() + rml::FrameID::Body + rigid_body_frame_id;
    std::string rigid_body_vehicle_frame = vehicle_model->GetID() + rml::FrameID::Body + rigid_body_frame_id;
    rml::JacobianObserver obsVehicle=rml::VehicleFrame;
    rml::JacobianObserver obsInertial=rml::InertialFrame;
    std::cout << "-- FRAMES ADDED --" << std::endl;

    Eigen::VectorXd control;
    control.setZero(8);
    robot_model->SetRobotControl(control);
    futils::PrettyPrint(robot_model->GetRobotControl(vehicle_id), "TEST GET ROBOT CONTROL VEHICLE");
    futils::PrettyPrint(robot_model->GetRobotControl(arm_id), "TEST GET ROBOT CONTROL ARM");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(joint_one_frame,obsVehicle), "JACOBIAN JOINT FRAME VEHICLE OBSERVER");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(joint_one_frame,obsInertial), "JACOBIAN JOINT FRAME INERTIAL OBSERVER");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(tool_frame,obsVehicle), "JACOBIAN TOOL FRAME VEHICLE OBSERVER");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(tool_frame,obsInertial), "JACOBIAN TOOL FRAME INERTIAL OBSERVER");
    futils::PrettyPrint(robot_model->GetJacobian_JointSpace(arm_id), "JACOBIAN JOINT SPACE");
    futils::PrettyPrint(robot_model->GetJacobian_Manipulability(arm_id), "JACOBIAN MANIPULABILITY ");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(vehicle_frame,obsInertial), "JACOBIAN VEHICLE");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(rigid_body_arm_frame,obsVehicle), "JACOBIAN ARM RIGID BODY FRAME JACOBIAN OBS VEHICLE");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(rigid_body_arm_frame,obsInertial), "JACOBIAN ARM RIGID BODY FRAME JACOBIAN OBS INERTIAL");
    futils::PrettyPrint(robot_model->GetJacobian_Frame(rigid_body_vehicle_frame,obsInertial), "JACOBIAN VEHICLE RIGID BODY FRAME JACOBIAN");
    futils::PrettyPrint(robot_model->GetTransformation(joint_one_frame), "JOINT FRAME TRANSFORMATION ");
    futils::PrettyPrint(robot_model->GetTransformation(tool_frame), "TOOL FRAME TRANSFORMATION");
    futils::PrettyPrint(robot_model->GetTransformation(vehicle_frame), "VEHICLE FRAME TRANSFORMATION");
    futils::PrettyPrint(robot_model->GetTransformation(rigid_body_arm_frame), "RIGID BODY ARM FRAME TRANSFORMATION");
    futils::PrettyPrint(robot_model->GetTransformation(rigid_body_vehicle_frame), "RIGID BODY VEHICLE FRAME TRANSFORMATION");
    ///MAP TESTING
    //std::map<std::string, Eigen::MatrixXd> testMap;
    //std::string ID1="ID1";
    //std::string ID2="ID2";
    //Eigen::MatrixXd matrix1;
    //matrix1.setIdentity(6,6);
    //Eigen::MatrixXd matrix2;
    //matrix2=2*matrix1;
    //Eigen::MatrixXd matrix3;
    //matrix3=3*matrix1;
    //testMap.insert(std::make_pair(ID1,matrix1));
    //testMap.insert(std::make_pair(ID2,matrix3));
    //std::cout << "\e[1mFIRST MAP \e[0m"  << std::endl;
    //for(std::map<std::string, Eigen::MatrixXd >::iterator iter = testMap.begin(); iter != testMap.end();
    //    ++iter)
    //{
    //    std::cout <<  iter->first << std::endl;
    //    std::cout << "Matrix\n" << iter->second << std::endl;
    //}
    //testMap.find(ID1)->second=matrix2;
    //std::cout << "\e[1m SECOND MAP with EMPLACE\e[0m"  << std::endl;
    //for(std::map<std::string, Eigen::MatrixXd >::iterator iter = testMap.begin(); iter != testMap.end();
    //    ++iter)
    //{
    //    std::cout  << iter->first << std::endl;
    //    std::cout << "Matrix\n" << iter->second << std::endl;
    //}

    return 0;
}
