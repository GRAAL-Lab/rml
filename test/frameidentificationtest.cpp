#include "rml_internal/Futils.h"
#include "test/twolinks_armmodel.h"
#include <iostream>
#include <memory>
#include <rml/RML.h>
#include <string>

int main()
{
    //ROBOT MODEL TEST
    std::string robotName = "RobotBodyFrame";
    std::string cameraFrameName = "CameraFrame";
    std::string armName = "ArmName";

    Eigen::TransformationMatrix T_AUV = Eigen::TransformationMatrix::Zero();
    Eigen::TransformationMatrix body_T_camera = Eigen::TransformationMatrix::Zero();
    Eigen::MatrixXd J_AUV = Eigen::MatrixXd::Zero(6, 6);

    // create a robot whose name is robotName (which is the name of the body frame)
    auto robotModel_ = std::make_shared<rml::RobotModel>(rml::RobotModel(T_AUV, robotName, J_AUV));
    // add a new frame in the rigid space of the body, called "CameraFrame", described by the transformation matrix body_T_camera
    robotModel_->AttachRigidBodyFrame(cameraFrameName, robotName, body_T_camera);
    // we can get its Cartesian Jacobian by appending the name of the frame to the body frame name
    auto Jcam = robotModel_->CartesianJacobian(robotName + "_" + cameraFrameName);
    std::cout << "Jcam: " << std::endl;
    std::cout << Jcam << std::endl;

    // create an arm model called "ArmName"
    auto armModel = std::make_shared<rml::ArmModel>(rml::ArmModel(armName));
    // add an arm with two joint and a tool attached on the second joint
    // baseF_T_joint0 is the transformation matrix between the the base and the current joint at q=0
    // each joint will get a frame called armName+rml::FrameID::Joint+Number, where number starts from 0
    // so "ArmName"+rml::FrameID::Joint+"0"
    Eigen::TransformationMatrix baseF_T_joint0 = Eigen::TransformationMatrix::Zero();
    double minJoint0 = 0.0, maxJoint0 = 0.0;
    std::cout << "AddJointLink" << std::endl;
    armModel->AddJointLink(rml::JointType::Revolute, Eigen::Vector3d::UnitZ(), baseF_T_joint0, minJoint0, maxJoint0);

    Eigen::TransformationMatrix joint0_T_joint1 = Eigen::TransformationMatrix::Zero();
    double minJoint1 = 0.0, maxJoint1 = 0.0;
    std::cout << "AddJointLink" << std::endl;
    armModel->AddJointLink(rml::JointType::Prismatic, Eigen::Vector3d::UnitZ(), joint0_T_joint1, minJoint1, maxJoint1);

    //add a tool to the second joint. In this case the tool get a frame "ArmName" + "_" + "toolID"
    Eigen::TransformationMatrix joint1_T_toolF = Eigen::TransformationMatrix::Zero();
    armModel->AttachRigidBodyFrame("toolID", armName + rml::FrameID::Joint + "1", joint1_T_toolF);

    Eigen::TransformationMatrix bodyF_T_baseF = Eigen::TransformationMatrix::Zero();
    // add the arm to the robot model, specifying where the base of the arm is with respect to the body frame of the vehicle
    robotModel_->LoadArm(armModel, bodyF_T_baseF);

    // if I want the Cartesian jacobian, I have
    auto Jlink1 = robotModel_->CartesianJacobian(armName + rml::FrameID::Joint + "0");
    std::cout << "Jlink1: " << std::endl;
    std::cout << Jlink1 << std::endl;
    // transformation w_T_j1 (by default its with respect to world frame)
    auto w_T_j1 = robotModel_->TransformationMatrix(armName + rml::FrameID::Joint + "0");
    std::cout << "w_T_j1: " << std::endl;
    std::cout << w_T_j1 << std::endl;
    // or explicit
    //    auto w_T_j1 = robotModel_->TransformationMatrix(rml::FrameID::WorldFrame, armName + rml::FrameID::Joint + "0");
    // Joint 1 with respect to camera frame
    auto c_T_j1 = robotModel_->TransformationMatrix(robotName + "_" + cameraFrameName, armName + rml::FrameID::Joint + "0");
    std::cout << "c_T_j1: " << std::endl;
    std::cout << c_T_j1 << std::endl;

    auto w_T_tool = robotModel_->TransformationMatrix(armName + "_" + "toolID");
    std::cout << "w_T_tool: " << std::endl;
    std::cout << w_T_tool << std::endl;

    auto armBaseF_T_joint = armModel->TransformationMatrix(armName + "_" + "toolID");
    std::cout << "armBaseF_T_joint: " << std::endl;
    std::cout << armBaseF_T_joint << std::endl;
    return 0;
}
