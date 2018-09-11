/**
 * @file baxterLeft_armmodel.cc
 *
 *  Created on: Mar 10, 2016
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */


#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

#include "test/baxterLeft_armmodel.h"

#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

//#define DBG_PRINT

using std::cout;
using std::endl;

namespace rml {

BaxterLeftArmModel::BaxterLeftArmModel(std::string id) : ArmModel(id)
{
  //SetArmJoints(7);
	InitMatrix();
}

BaxterLeftArmModel::~BaxterLeftArmModel()
{
    // TODO Auto-generated destructor stub
}

void BaxterLeftArmModel::InitMatrix()
{
    /*ArmModel::InitMatrix();

    baseTb0_(0, 0) = 0.7081;   baseTb0_(0, 1) = -0.7061;  baseTb0_(0, 2) = -0.0037;    baseTb0_(0, 3) = 0.024;
    baseTb0_(1, 0) = 0.7061;   baseTb0_(1, 1) = 0.7081;   baseTb0_(1, 2) = -0.0015;    baseTb0_(1, 3) = 0.22;
    baseTb0_(2, 0) = 0.0037;   baseTb0_(2, 1) = -0.0015;  baseTb0_(2, 2) = 1;          baseTb0_(2, 3) = 0.108;
    baseTb0_(3, 0) = 0;        baseTb0_(3, 1) = 0;        baseTb0_(3, 2) = 0;          baseTb0_(3, 3) = 1;

	biTri_[0](0,0) = 1;      biTri_[0](0,1) = 0.002;  biTri_[0](0,2) = 0; biTri_[0](0,3) = 0.056;
	biTri_[0](1,0) = -0.002; biTri_[0](1,1) = 1;      biTri_[0](1,2) = 0; biTri_[0](1,3) = 0;
	biTri_[0](2,0) = 0;      biTri_[0](2,1) = 0;      biTri_[0](2,2) = 1; biTri_[0](2,3) = 0.011;
	biTri_[0](3,0) = 0;      biTri_[0](3,1) = 0;      biTri_[0](3,2) = 0; biTri_[0](3,3) = 1;

	biTri_[1](0,0) = 1;      biTri_[1](0,1) = 0.0028; biTri_[1](0,2) = 0; biTri_[1](0,3) = 0.069;
	biTri_[1](1,0) = 0;      biTri_[1](1,1) = 0;      biTri_[1](1,2) = 1; biTri_[1](1,3) = 0;
	biTri_[1](2,0) = 0.0028; biTri_[1](2,1) = -1;     biTri_[1](2,2) = 0; biTri_[1](2,3) = 0.27;
	biTri_[1](3,0) = 0;      biTri_[1](3,1) = 0;      biTri_[1](3,2) = 0; biTri_[1](3,3) = 1;

	biTri_[2](0,0) = 0;      biTri_[2](0,1) = 0;      biTri_[2](0,2) = 1; biTri_[2](0,3) = 0.102;
	biTri_[2](1,0) = 1;      biTri_[2](1,1) = 0;      biTri_[2](1,2) = 0; biTri_[2](1,3) = 0;
	biTri_[2](2,0) = 0;      biTri_[2](2,1) = 1;      biTri_[2](2,2) = 0; biTri_[2](2,3) = 0;
	biTri_[2](3,0) = 0;      biTri_[2](3,1) = 0;      biTri_[2](3,2) = 0; biTri_[2](3,3) = 1;

	biTri_[3](0,0) = 0;      biTri_[3](0,1) = 1;      biTri_[3](0,2) = 0; biTri_[3](0,3) = 0.069;
	biTri_[3](1,0) = 0;      biTri_[3](1,1) = 0;      biTri_[3](1,2) = 1; biTri_[3](1,3) = 0;
	biTri_[3](2,0) = 1;      biTri_[3](2,1) = 0;      biTri_[3](2,2) = 0; biTri_[3](2,3) = 0.262;
	biTri_[3](3,0) = 0;      biTri_[3](3,1) = 0;      biTri_[3](3,2) = 0; biTri_[3](3,3) = 1;

	biTri_[4](0,0) = 0;      biTri_[4](0,1) = 0;      biTri_[4](0,2) = 1; biTri_[4](0,3) = 0.104;
	biTri_[4](1,0) = 1;      biTri_[4](1,1) = 0;      biTri_[4](1,2) = 0; biTri_[4](1,3) = 0;
	biTri_[4](2,0) = 0;      biTri_[4](2,1) = 1;      biTri_[4](2,2) = 0; biTri_[4](2,3) = 0;
	biTri_[4](3,0) = 0;      biTri_[4](3,1) = 0;      biTri_[4](3,2) = 0; biTri_[4](3,3) = 1;

	biTri_[5](0,0) = 0;      biTri_[5](0,1) = 1;      biTri_[5](0,2) = 0; biTri_[5](0,3) = 0.01;
	biTri_[5](1,0) = 0;      biTri_[5](1,1) = 0;      biTri_[5](1,2) = 1; biTri_[5](1,3) = 0;
	biTri_[5](2,0) = 1;      biTri_[5](2,1) = 0;      biTri_[5](2,2) = 0; biTri_[5](2,3) = 0.271;
	biTri_[5](3,0) = 0;      biTri_[5](3,1) = 0;      biTri_[5](3,2) = 0; biTri_[5](3,3) = 1;

	biTri_[6](0,0) = 0;      biTri_[6](0,1) = 0;      biTri_[6](0,2) = 1; biTri_[6](0,3) = 0.116;
	biTri_[6](1,0) = 1;      biTri_[6](1,1) = 0;      biTri_[6](1,2) = 0; biTri_[6](1,3) = 0;
	biTri_[6](2,0) = 0;      biTri_[6](2,1) = 1;      biTri_[6](2,2) = 0; biTri_[6](2,3) = 0;
	biTri_[6](3,0) = 0;      biTri_[6](3,1) = 0;      biTri_[6](3,2) = 0; biTri_[6](3,3) = 1;

	eTt_(0,0) = 1; eTt_(0,1) = 0;  eTt_(0,2) = 0; eTt_(0,3) = 0;
	eTt_(1,0) = 0; eTt_(1,1) = 1;  eTt_(1,2) = 0; eTt_(1,3) = 0;
	eTt_(2,0) = 0; eTt_(2,1) = 0;  eTt_(2,2) = 1; eTt_(2,3) = 0.27125; //Original 0.114;
	eTt_(3,0) = 0; eTt_(3,1) = 0;  eTt_(3,2) = 0; eTt_(3,3) = 1;
*/
}


}

