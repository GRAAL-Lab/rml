/**
 * @file twolinks_armmodel.cc
 *
 *  Created on: Mar 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Universit� Degli Studi di Genova
 */


#include "test/twolinks_armmodel.h"

using std::cout;
using std::endl;

namespace rml {

TwoLinksArmModel::TwoLinksArmModel(std::string id) : ArmModel(id)
{
	int numJoints = 2;
	std::vector<Eigen::TransfMatrix> biTri(numJoints);

    double min[] = { -2.94, -1.13 };//, -2.616, -1.788, -2.68 };
    double max[] = { +2.94, +1.57 };//, +2.54, +1.788, +2.923 };

	biTri.at(0)(0,0) = 1; biTri.at(0)(0,1) = 0; biTri.at(0)(0,2) = 0; biTri.at(0)(0,3) = 0;
	biTri.at(0)(1,0) = 0; biTri.at(0)(1,1) = 1; biTri.at(0)(1,2) = 0; biTri.at(0)(1,3) = 0;
	biTri.at(0)(2,0) = 0; biTri.at(0)(2,1) = 0; biTri.at(0)(2,2) = 1; biTri.at(0)(2,3) = 0.175;
	biTri.at(0)(3,0) = 0; biTri.at(0)(3,1) = 0; biTri.at(0)(3,2) = 0; biTri.at(0)(3,3) = 1;

	biTri.at(1)(0,0) = 1; biTri.at(1)(0,1) = 0; biTri.at(1)(0,2) = 0; biTri.at(1)(0,3) = 0;
	biTri.at(1)(1,0) = 0; biTri.at(1)(1,1) = 0; biTri.at(1)(1,2) = 1; biTri.at(1)(1,3) = 0;
	biTri.at(1)(2,0) = 0; biTri.at(1)(2,1) = -1;biTri.at(1)(2,2) = 0; biTri.at(1)(2,3) = 0.108;
	biTri.at(1)(3,0) = 0; biTri.at(1)(3,1) = 0; biTri.at(1)(3,2) = 0; biTri.at(1)(3,3) = 1;

	for (int i = 0; i < numJoints; ++i) {
		//std::cout << "Adding link " << i << std::endl;
		AddLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), biTri.at(i), min[i], max[i]);
	}
/*
	eTt_(0,0) = 1; eTt_(0,1) = 0;  eTt_(0,2) = 0;  eTt_(0,3) = 0;
	eTt_(1,0) = 0; eTt_(1,1) = 0;  eTt_(1,2) = -1; eTt_(1,3) = -0.105;
	eTt_(2,0) = 0; eTt_(2,1) = 1;  eTt_(2,2) = 0;  eTt_(2,3) = 0;
	eTt_(3,0) = 0; eTt_(3,1) = 0;  eTt_(3,2) = 0;  eTt_(3,3) = 1;
*/
}

TwoLinksArmModel::~TwoLinksArmModel()
{
	// TODO Auto-generated destructor stub
}

}

