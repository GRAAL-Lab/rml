#include <rml/RML.h>
#include "test/youbot_armmodel.h"
#include <iostream>
#include <string>
#include <memory>
int main ()
{
    std::string ID="TestArm";
    auto arm_model = std::make_shared<rml::YouBotArmModel>(rml::YouBotArmModel(ID));
    Eigen::VectorXd initial_joint_pos(arm_model->GetNumJoints());


    ///TEST TRANSFORMATION MATRICES AND JACOBIANS FOR JOINTS AND ARM
    //
    ////TRANSF MATRIX
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID) << std::endl;
    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    //
    //for (int i =0; i< arm_model->GetNumJoints(); i++)
    //{
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
    //    std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    //}
    //
    ////JACOBIANS
    //std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID) << std::endl;
    //std::cout << "Jacobian via method\n " << arm_model->GetBase2ToolJacobian()<< std::endl;
    //
    //for (int i =0; i<arm_model->GetNumJoints(); i++)
    //
    //{
    //
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
    //    std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    //}
    //
    //initial_joint_pos << 0.011, 0.11, -1.4, -0.11, 1.57;
    //arm_model->SetJointsPosition(initial_joint_pos);
    //
    //
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID) << std::endl;
    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    //for (int i =0; i< arm_model->GetNumJoints(); i++)
    //{
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
    //    std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    //
    //}
    //
    ////JACOBIANS
    //std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID) << std::endl;
    //std::cout << "Jacobian via method \n" << arm_model->GetBase2ToolJacobian()<< std::endl;
    //
    //for (int i =0; i<arm_model->GetNumJoints(); i++)
    //
    //{
    //
    //    std::string matrixName=ID+"_Joint_"+std::to_string(i);
    //    std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
    //    std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    //}
    //

   ////TEST ATTCHED BODY FRAME TRANSF AND JACOBIANS
   // std::string rigid_body_frame_id("cameraFrame");
   // Eigen::TransfMatrix T;
   // T.SetTransl(Eigen::Vector3d(0.2,0.0,0.2));
   // arm_model->AddRigidBodyFrame(rigid_body_frame_id,2,T);
   // std::cout << "TransformationMatrix \n" << arm_model->GetTransformationMatrix(rigid_body_frame_id) << std::endl;
   // std::cout << "TransfMatrix with method\n " << arm_model->GetAttachedBodyTransf(rigid_body_frame_id) << std::endl;
   // std::cout << "Jacobian\n" << arm_model->GetJacobian(rigid_body_frame_id) << std::endl;
   // std::cout << "Jacobian with method \n " << arm_model->GetAttachedBodyJacobian(rigid_body_frame_id) << std::endl;
   // initial_joint_pos << 0.011, 0.11, -1.4, -0.11, 1.57;
   // arm_model->SetJointsPosition(initial_joint_pos);
   // std::cout << "TransformationMatrix \n" << arm_model->GetTransformationMatrix(rigid_body_frame_id) << std::endl;
   // std::cout << "TransfMatrix with method\n " << arm_model->GetAttachedBodyTransf(rigid_body_frame_id) << std::endl;
   // std::cout << "Jacobian\n" << arm_model->GetJacobian(rigid_body_frame_id) << std::endl;
   // std::cout << "Jacobian with method \n " << arm_model->GetAttachedBodyJacobian(rigid_body_frame_id) << std::endl;

    //TEST FOR VEHICLE JACOBIAN, TRANSFORMATION MATRIX and ATTACHED BODY TRANSFORMATION
    std::string vehicle_id("vehicleTest");
    auto vehicle_model = std::make_shared<rml::VehicleModel>(rml::VehicleModel(vehicle_id));
    vehicle_model->SetJacobian(Eigen::MatrixXd::Identity(6, 6));
    Eigen::Vector6d vehicle_position;
    //std::cout << "Jacobians " << vehicle_model->GetJacobian(vehicle_id) << std::endl;
    //std::cout << "Jacobian via method " << vehicle_model->GetvJv() << std::endl;
    //std::cout << "Transformation " << vehicle_model->GetTransfMatrix(vehicle_id) << std::endl;
    //std::cout << "Transfromation via merthod " << vehicle_model->GetwTv() << std::endl;
    //vehicle_position.SetSecondVect3(Eigen::Vector3d(0.1,0.0,0.0));
    //vehicle_position.SetFirstVect3(Eigen::Vector3d(0.0,0.0,0.0));
    //vehicle_model->SetPositionOnInertial(vehicle_position);
    //std::cout << "Jacobians " << vehicle_model->GetJacobian(vehicle_id) << std::endl;
    //std::cout << "Jacobian via method " << vehicle_model->GetvJv() << std::endl;
    //std::cout << "Transformation " << vehicle_model->GetTransfMatrix(vehicle_id) << std::endl;
    //std::cout << "Transfromation via merthod " << vehicle_model->GetwTv() << std::endl;

    ////TEST ATTCHED BODY FRAME TRANSF AND JACOBIANS
    std::string rigid_body_frame_id("cameraFrame");
    std::string rigid_body_frame_id_for_look_up=vehicle_id+rigid_body_frame_id;
    Eigen::TransfMatrix T;
    T.SetTransl(Eigen::Vector3d(0.2,0.0,0.2));
    vehicle_model->AddRigidBodyFrame(rigid_body_frame_id,T);
    std::cout << "TransformationMatrix \n" << vehicle_model->GetTransfMatrix(rigid_body_frame_id_for_look_up) << std::endl;
    std::cout << "TransfMatrix with method\n " << vehicle_model->GetAttachedBodyTransf(rigid_body_frame_id_for_look_up) << std::endl;
    std::cout << "Jacobian\n" << vehicle_model->GetJacobian(rigid_body_frame_id_for_look_up) << std::endl;
    std::cout << "Jacobian with method \n " << vehicle_model->GetAttachedBodyJacobian(rigid_body_frame_id_for_look_up) << std::endl;
    vehicle_position.SetSecondVect3(Eigen::Vector3d(0.1,0.0,0.0));
    vehicle_position.SetFirstVect3(Eigen::Vector3d(0.0,0.0,0.0));
    vehicle_model->SetPositionOnInertial(vehicle_position);
    std::cout << "TransformationMatrix \n" << vehicle_model->GetTransfMatrix(rigid_body_frame_id_for_look_up) << std::endl;
    std::cout << "TransfMatrix with method\n " << vehicle_model->GetAttachedBodyTransf(rigid_body_frame_id_for_look_up) << std::endl;
    std::cout << "Jacobian\n" << vehicle_model->GetJacobian(rigid_body_frame_id_for_look_up) << std::endl;
    std::cout << "Jacobian with method \n " << vehicle_model->GetAttachedBodyJacobian(rigid_body_frame_id_for_look_up) << std::endl;


    return 0;

}
