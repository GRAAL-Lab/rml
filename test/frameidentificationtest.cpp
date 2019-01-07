#include "rml_internal/Futils.h"
#include "test/twolinks_armmodel.h"
#include <iostream>
#include <memory>
#include <rml/RML.h>
#include <string>

int main()
{
    ////ROBOT MODEL TEST
    //std::string arm_id = "armID";
    //std::string robot_frame_id = "robotID";
    //std::string joint_one_frame = arm_id + rml::FrameID::Joint + "1";
    //std::string rigid_body_frame_id = robot_frame_id + rml::FrameID::Body + "cameraFrame";
    //std::string frame_ID = "cameraFrame";
    //std::string rigid_body_frame_id_arm= arm_id+ rml::FrameID::Body+frame_ID;
    //Eigen::TransfMatrix T;
    //T.SetTransl(Eigen::Vector3d(0.2, 0.0, 0.2));
    //Eigen::TransfMatrix TrobotFrame;
    //TrobotFrame.SetTransl(Eigen::Vector3d(1, 0.2, 0));
    //Eigen::TransfMatrix vTb;
    //vTb(0, 0) = -1;
    //vTb(0, 1) = 0;
    //vTb(0, 2) = 0;
    //vTb(0, 3) = 0.85;
    //vTb(1, 0) = 0;
    //vTb(1, 1) = -1;
    //vTb(1, 2) = 0;
    //vTb(1, 3) = 0;
    //vTb(2, 0) = 0;
    //vTb(2, 1) = 0;
    //vTb(2, 2) = 1;
    //vTb(2, 3) = 0.42;
    //vTb(3, 0) = 0;
    //vTb(3, 1) = 0;
    //vTb(3, 2) = 0;
    //vTb(3, 3) = 1;
    //std::cout << "INITIALIZATION" << std::endl;
    //auto arm_model = std::make_shared<rml::TwoLinksArmModel>(rml::TwoLinksArmModel(arm_id));
    //auto robot_model = std::make_shared<rml::RobotModel>(rml::RobotModel(Eigen::TransfMatrix(), robot_frame_id));
    //auto robot_model_with_vehicle = std::make_shared<rml::RobotModel>(rml::RobotModel(Eigen::TransfMatrix(), robot_frame_id, Eigen::MatrixXd::Identity(6, 6)));
    //robot_model->LoadArm(arm_model, vTb);
    //robot_model_with_vehicle->LoadArm(arm_model, vTb);
    //Eigen::VectorXd initial_tool_pos(2);
    //initial_tool_pos << 0.011, 0.11; // -1.4, -0.11, 1.57;//, -1.4, -0.11, 1.57;
    //robot_model->GetArm(arm_id)->SetJointsPosition(initial_tool_pos);
    //robot_model_with_vehicle->GetArm(arm_id)->SetJointsPosition(initial_tool_pos);
    //robot_model->SetAttachedRigidBodyFrame(frame_ID,T,robot_frame_id);
    //robot_model->SetAttachedRigidBodyFrame(frame_ID, T, joint_one_frame);
    //robot_model_with_vehicle->SetAttachedRigidBodyFrame(frame_ID,T,robot_frame_id);
    //robot_model_with_vehicle->SetAttachedRigidBodyFrame(frame_ID,T,joint_one_frame);
    //robot_model->SetBodyFramePosition(TrobotFrame);
    //robot_model_with_vehicle->SetBodyFramePosition(TrobotFrame);
    //Eigen::Vector2d control_no_vehicle(0.1, 0.2);
    //robot_model->SetRobotControl(control_no_vehicle);
    //Eigen::VectorXd control(8);
    //control << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8;
    //robot_model_with_vehicle->SetRobotControl(control);
    //
    //futils::PrettyPrint(robot_model_with_vehicle->GetRobotControl(arm_id), "robot control arm with");
    //futils::PrettyPrint(robot_model_with_vehicle->GetRobotControl(robot_frame_id), "robot control arm  with vehicle ");
    //futils::PrettyPrint(robot_model_with_vehicle->GetCartesianJacobian(joint_one_frame), "JOINT ONE CARTESIAN JACOBIAN WITH VEHICLE");
    //futils::PrettyPrint(robot_model_with_vehicle->GetTransformation(robot_frame_id), "ROBOT FRAME WITH WITH VEHICLE");
    //futils::PrettyPrint(robot_model_with_vehicle->GetCartesianJacobian(robot_frame_id), "ROBOT FRAME JACOBIAN WITH VEHICLE");
    //
    //futils::PrettyPrint(robot_model_with_vehicle->GetCartesianJacobian(rigid_body_frame_id), "RIGID BODY JACOBIAN WITH VEHICLE");
    //futils::PrettyPrint(robot_model_with_vehicle->GetCartesianJacobian(rigid_body_frame_id_arm),"RIGID BODY FRAME ID WITH VEHICLE ARM ");
    //futils::PrettyPrint(robot_model->GetRobotControl(arm_id), "robot control arm ");
    ////futils::PrettyPrint(robot_model->GetRobotControl(robot_frame_id),"robot control arm ");
    //futils::PrettyPrint(robot_model->GetCartesianJacobian(joint_one_frame), "JOINT ONE CARTESIAN JACOBIAN WITH NO VEHICLE");
    //futils::PrettyPrint(robot_model->GetTransformation(robot_frame_id), "ROBOT FRAME WITH NO VEHICLE");
    //futils::PrettyPrint(robot_model->GetCartesianJacobian(robot_frame_id), "ROBOT FRAME JACOBIAN WITH NO VEHICLE");
    //futils::PrettyPrint(robot_model->GetCartesianJacobian(rigid_body_frame_id), "RIGID BODY JACOBIAN WITH WITH VEHICLE");
    //futils::PrettyPrint(robot_model->GetCartesianJacobian(rigid_body_frame_id_arm),"RIGID BODY FRAME ID NO VEHICLE ");
    ////Test change jacobian observer  from inertial to vehicle for joint 5
    //Eigen::Vector3d Error= rml::CartesianError(Eigen::TransfMatrix(),robot_model_with_vehicle->GetTransformation(joint_one_frame)).GetFirstVect3();
    //Eigen::MatrixXd J = robot_model_with_vehicle->GetCartesianJacobian(joint_one_frame).block(0,0,3,robot_model_with_vehicle->GetTotalDOFs());
    //Eigen::MatrixXd Jobs= robot_model_with_vehicle->GetCartesianJacobian(robot_frame_id).block(0,0,3,robot_model_with_vehicle->GetTotalDOFs());
    //futils::PrettyPrint(J, "JACOBIAN BEFORE CHANGING OBSERVER") ;
    //futils::PrettyPrint(rml::ChangeJacobianObserver(J,Jobs,Error), "CHANGE OF OBSERVER ");

    Eigen::Vector3d a;
    a<< 0, 0, 1;
    Eigen::Vector3d b;
    b<< 0, 0, -1;
    std::cout<<"reduced versor lemma"<<std::endl;
    Eigen::Vector3d c = rml::ReducedVersorLemma(a,b);
    std::cout<< "Reduced Versor Lemma in btw "<<a.transpose()<<" and "<<b.transpose()<<" = "<< c.norm()<<"vector="<<c.normalized().transpose()<<std::endl;
    return 0;
}
