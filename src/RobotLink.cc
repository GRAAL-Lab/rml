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

RobotLink::RobotLink(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double joinLimMax):
		type_(type), axis_(axis), baseTransf_(baseTransf), jointLimitsMin_(jointLimMin), jointLimitsMAX_(joinLimMax)
{
	InitVectors();
}

RobotLink::~RobotLink()
{
	// TODO Auto-generated destructor stub
}

void RobotLink::InitVectors() {
	axis_.setZero();
	sizeVect_.setZero();
	CoM_.setZero();
	Inertia_.setZero();
	self_f_.setZero();
	self_n_.setZero();
	inter_f_.setZero();
	inter_n_.setZero();

}

void RobotLink::SetPhysicalProperties(double mass, const Eigen::Vector3d& sizeVect, const Eigen::Vector3d& CoM, const Eigen::Matrix3d &Inertia)
{
	mass_ = mass;
	sizeVect_ = sizeVect;
	CoM_ = CoM;
	Inertia_ = Inertia;

	//InitVectors();
}

}


