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

    //TRANSF MATRIX
    //std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID) << std::endl;
    //std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    /*
    for (int i =0; i< arm_model->GetNumJoints(); i++)
    {
        std::string matrixName=ID+"_Joint_"+std::to_string(i);
        std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
        std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;
    }
    */
    //JACOBIANS
    std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID) << std::endl;
    std::cout << "Jacobian via method\n " << arm_model->GetBase2ToolJacobian()<< std::endl;

    for (int i =0; i<arm_model->GetNumJoints(); i++)

    {

        std::string matrixName=ID+"_Joint_"+std::to_string(i);
        std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
        std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    }

    initial_joint_pos << 0.011, 0.11, -1.4, -0.11, 1.57;
    arm_model->SetJointsPosition(initial_joint_pos);

    /*
    std::cout <<" TransfMatrix Tool\n  " << arm_model->GetTransformationMatrix(ID) << std::endl;
    std::cout<<"with base to joint transf\n"<< arm_model->GetBase2ToolTransf()<<std::endl;
    for (int i =0; i< arm_model->GetNumJoints(); i++)
    {
        std::string matrixName=ID+"_Joint_"+std::to_string(i);
        std::cout << matrixName+" TransfMatrix\n  " << arm_model->GetTransformationMatrix(matrixName) << std::endl;
        std::cout<<"with base to joint transf\n"<< arm_model->GetBase2JointTransf(i)<<std::endl;

    }
    */
    //JACOBIANS
    std::cout << "Jacobian Tool\n" << arm_model->GetJacobian(ID) << std::endl;
    std::cout << "Jacobian via method \n" << arm_model->GetBase2ToolJacobian()<< std::endl;

    for (int i =0; i<arm_model->GetNumJoints(); i++)

    {

        std::string matrixName=ID+"_Joint_"+std::to_string(i);
        std::cout << matrixName+" Jacobians\n  " << arm_model->GetJacobian(matrixName) << std::endl;
        std::cout<<"with via method \n"<< arm_model->EvaluateBase2JointJacobian(i)<<std::endl;
    }

    return 0;

}
