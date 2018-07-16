/**
 * @file youbot_armmodel.cc
 *
 *  Created on: Mar 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Universit� Degli Studi di Genova
 */


#include "test/youbot_armmodel.h"

using std::cout;
using std::endl;

namespace rml {

YouBotArmModel::YouBotArmModel(std::string id): ArmModel(id)
{
	int numJoints = 5;
	std::vector<Eigen::TransfMatrix> biTri(numJoints);


	double min[] = { -2.94, -1.13, -2.616, -1.788, -2.68 };
	double max[] = { +2.94, +1.57, +2.54, +1.788, +2.923 };

	biTri.at(0)(0,0) = 1;      biTri.at(0)(0,1) = 0;      biTri.at(0)(0,2) = 0;  biTri.at(0)(0,3) = 0;
	biTri.at(0)(1,0) = 0;      biTri.at(0)(1,1) = -1;     biTri.at(0)(1,2) = 0;  biTri.at(0)(1,3) = 0;
	biTri.at(0)(2,0) = 0;      biTri.at(0)(2,1) = 0;      biTri.at(0)(2,2) = -1; biTri.at(0)(2,3) = 0.072;
	biTri.at(0)(3,0) = 0;      biTri.at(0)(3,1) = 0;      biTri.at(0)(3,2) = 0;  biTri.at(0)(3,3) = 1;

	biTri.at(1)(0,0) = 1;      biTri.at(1)(0,1) = 0;      biTri.at(1)(0,2) = 0;  biTri.at(1)(0,3) = 0;
	biTri.at(1)(1,0) = 0;      biTri.at(1)(1,1) = 0;      biTri.at(1)(1,2) = 1;  biTri.at(1)(1,3) = 0;
	biTri.at(1)(2,0) = 0;      biTri.at(1)(2,1) = -1;     biTri.at(1)(2,2) = 0;  biTri.at(1)(2,3) = -0.075;
	biTri.at(1)(3,0) = 0;      biTri.at(1)(3,1) = 0;      biTri.at(1)(3,2) = 0;  biTri.at(1)(3,3) = 1;

	biTri.at(2)(0,0) = 1;      biTri.at(2)(0,1) = 0;      biTri.at(2)(0,2) = 0;  biTri.at(2)(0,3) = 0;
	biTri.at(2)(1,0) = 0;      biTri.at(2)(1,1) = 1;      biTri.at(2)(1,2) = 0;  biTri.at(2)(1,3) = 0.155;
	biTri.at(2)(2,0) = 0;      biTri.at(2)(2,1) = 0;      biTri.at(2)(2,2) = 1;  biTri.at(2)(2,3) = 0;
	biTri.at(2)(3,0) = 0;      biTri.at(2)(3,1) = 0;      biTri.at(2)(3,2) = 0;  biTri.at(2)(3,3) = 1;

	biTri.at(3)(0,0) = 1;      biTri.at(3)(0,1) = 0;      biTri.at(3)(0,2) = 0;  biTri.at(3)(0,3) = 0;
	biTri.at(3)(1,0) = 0;      biTri.at(3)(1,1) = 1;      biTri.at(3)(1,2) = 0;  biTri.at(3)(1,3) = 0.135;
	biTri.at(3)(2,0) = 0;      biTri.at(3)(2,1) = 0;      biTri.at(3)(2,2) = 1;  biTri.at(3)(2,3) = 0;
	biTri.at(3)(3,0) = 0;      biTri.at(3)(3,1) = 0;      biTri.at(3)(3,2) = 0;  biTri.at(3)(3,3) = 1;

	biTri.at(4)(0,0) = 1;      biTri.at(4)(0,1) = 0;      biTri.at(4)(0,2) = 0;  biTri.at(4)(0,3) = 0;
	biTri.at(4)(1,0) = 0;      biTri.at(4)(1,1) = 0;      biTri.at(4)(1,2) = -1; biTri.at(4)(1,3) = 0.113;
	biTri.at(4)(2,0) = 0;      biTri.at(4)(2,1) = 1;      biTri.at(4)(2,2) = 0;  biTri.at(4)(2,3) = 0;
	biTri.at(4)(3,0) = 0;      biTri.at(4)(3,1) = 0;      biTri.at(4)(3,2) = 0;  biTri.at(4)(3,3) = 1;

	for (int i = 0; i < numJoints; ++i) {
		//std::cout << "Adding link " << i << std::endl;
		AddJointLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), biTri.at(i), min[i], max[i]);
	}

    /*eTt_(0,0) =  1;   eTt_(0,1) =  0;   eTt_(0,2) =  0;   eTt_(0,3) = 0;
	eTt_(1,0) =  0;   eTt_(1,1) =  1;   eTt_(1,2) =  0;   eTt_(1,3) = 0;
	eTt_(2,0) =  0;   eTt_(2,1) =  0;   eTt_(2,2) =  1;   eTt_(2,3) = 0;
	eTt_(3,0) =  0;   eTt_(3,1) =  0;   eTt_(3,2) =  0;   eTt_(3,3) = 1;
*/
}

YouBotArmModel::~YouBotArmModel()
{
	// TODO Auto-generated destructor stub
}

}

