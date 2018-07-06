#include "test/youbot_armmodel.h"
#include "test/twolinks_armmodel.h"
#include <iostream>
#include <memory>
#include <rml/RML.h>
#include "rml_internal/Futils.h"
#include <string>

int main()
{
    //std::string ID="TestArm";
    //auto arm_model = std::make_shared<rml::YouBotArmModel>(rml::YouBotArmModel(ID));
    //Eigen::VectorXd initial_joint_pos(arm_model->GetNumJoints());
    //
    ////TEST TRANSFORMATION MATRICES AND JACOBIANS FOR JOINTS AND ARM
    //
    ////TRANSF MATRIX
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID+"_Tool") << std::endl;
    ////std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    //
    //for (int i =0; i< arm_model->GetNumJoints(); i++)
    //{
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
    //    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    //}
    //
    ////JACOBIANS
    //std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID+"_Tool") << std::endl;
    ////std::cout << "Jacobian via method\n " << arm_model->GetBase2ToolJacobian()<< std::endl;
    //
    //for (int i =0; i<arm_model->GetNumJoints(); i++)
    //
    //{
    //
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
    //    //std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    //}
    //
    //initial_joint_pos << 0.011, 0.11, -1.4, -0.11, 1.57;
    //arm_model->SetJointsPosition(initial_joint_pos);
    //
    //
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID+"_Tool") << std::endl;
    ////std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    //for (int i =0; i< arm_model->GetNumJoints(); i++)
    //{
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
    //    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    //
    //}
    //
    ////JACOBIANS
    //std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID+"_Tool") << std::endl;
    ////std::cout << "Jacobian via method \n" << arm_model->GetBase2ToolJacobian()<< std::endl;
    //
    //for (int i =0; i<arm_model->GetNumJoints(); i++)
    //
    //{
    //
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
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
    //initial_joint_pos << 0.011, 0.11, -1.4, -0.11, 1.57;
    //arm_model->SetJointsPosition(initial_joint_pos);
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
    Eigen::VectorXd initial_joint_pos(arm_model->GetNumJoints());
    initial_joint_pos << 0.011, 0.11;//, -1.4, -0.11, 1.57;
    arm_model->SetJointsPosition(initial_joint_pos);
    std::string rigid_body_frame_id("CameraFrame");
    Eigen::TransfMatrix T;
    T.SetTransl(Eigen::Vector3d(0.2, 0.0, 0.2));
    arm_model->AddRigidBodyFrame(rigid_body_frame_id, arm_model->GetNumJoints() - 1, T);
    vehicle_model->AddRigidBodyFrame(rigid_body_frame_id, T);
    std::string testJointArm = "Frame_" + arm_model->GetID() + "_Joint_3";
    std::string testJointVehicle = "Frame_" + vehicle_model->GetID() + "_" + arm_model->GetID() + "_Joint_3";
    std::string test_Arm_Tool = "Frame_" + arm_model->GetID() + "_Tool";
    std::string test_Vehcle_Tool = "Frame_" + vehicle_model->GetID() + "_" + arm_model->GetID() + "_Tool";
    std::string test_Identitity = "Identity_" + arm_model->GetID() + "_Identity";
    std::string test_Manipulability = "Manipulability_" + arm_model->GetID() + "_Manipulability";
    std::string test_Vehicle_Jacobian = "Vehicle_" + vehicle_model->GetID();
    std::string test_Rigid_Body_Arm = "Frame_" + arm_model->GetID() + "_Body_" + rigid_body_frame_id;
    std::string test_Rigid_Body_Vehicle = "Vehicle_" + vehicle_model->GetID() + "_Body_" + rigid_body_frame_id;

    std::cout << "-- FRAMES ADDED --" << std::endl;

    futils::PrettyPrint(robot_model->GetJacobian(testJointArm), "TEST JOINT ARM");
    futils::PrettyPrint(robot_model->GetJacobian(testJointVehicle), "TEST JOINT VEHICLE");
    futils::PrettyPrint(robot_model->GetJacobian(test_Arm_Tool), "TEST TOOL ARM");
    futils::PrettyPrint(robot_model->GetJacobian(test_Vehcle_Tool), "TEST GET VEHICLE TOOL");
    futils::PrettyPrint(robot_model->GetJacobian(test_Identitity), "TEST IDENTITY ARM");
    futils::PrettyPrint(robot_model->GetJacobian(test_Manipulability), "TEST MANIPULABILITY ARM");
    futils::PrettyPrint(robot_model->GetJacobian(test_Vehicle_Jacobian), "TEST VEHICLE JACOBIAN");
    futils::PrettyPrint(robot_model->GetJacobian(test_Rigid_Body_Arm), "TEST ARM RIGID BODY JACOBIAN");
    futils::PrettyPrint(robot_model->GetJacobian(test_Rigid_Body_Vehicle), "TEST VEHICLE RIGID BODY JACOBIAN");

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
