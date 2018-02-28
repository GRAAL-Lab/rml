/*
 * RobotLink.cpp
 *
 *  Created on: Jul 8, 2015
 *      Author: francescow
 */

#include <iostream>
#include "rml/RobotLink.h"


using std::cout;
using std::endl;

namespace rml {

RobotLink::RobotLink() :
				type_(JointType::Fixed), mass_(0), jointLimitsMin_(0), jointLimitsMAX_(0)
{
	InitVectors();
}

RobotLink::RobotLink(const JointType type, const Eigen::TransfMatrix& baseTransf): type_(type), baseTransf_(baseTransf){
	InitVectors();
}

RobotLink::~RobotLink()
{
	// TODO Auto-generated destructor stub
}

void RobotLink::InitVectors() {
	lengthVec_.setZero();
	CoM_.setZero();
	Inertia_.setZero();
	self_f.setZero();
	self_n.setZero();
	inter_f.setZero();
	inter_n.setZero();
}

void RobotLink::SetPhysicalProperties(const double mass, const Eigen::Vector3d& lengthVec, const Eigen::Vector3d& CoM, const Eigen::MatrixXd &Inertia)
{
	mass_ = mass;
	lengthVec_ = lengthVec;
	CoM_ = CoM;
	Inertia_ = Inertia;

	//InitVectors();

}
}


