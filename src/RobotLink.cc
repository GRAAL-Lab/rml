/*
 * RobotLink.cpp
 *
 *  Created on: Jul 8, 2015
 *      Author: francescow
 */

#include <iostream>
#include "RobotLink.h"

using std::cout;
using std::endl;

namespace rml
{

RobotLink::RobotLink() :
		type_(JointType::Fixed), mass_(0), jointLimitMin_(0), jointLimitMAX_(0)
{
	axis_.setZero();
	sizeVect_.setZero();
	CoM_.setZero();
	Inertia_.setZero();
}

RobotLink::RobotLink(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf,
		double jointLimMin, double jointLimMax) :
		type_(type), axis_(axis), baseTransf_(baseTransf), jointLimitMin_(jointLimMin), jointLimitMAX_(jointLimMax)
{
	SetDynamicProperties(0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero());
}

RobotLink::~RobotLink()
{
	// TODO Auto-generated destructor stub
}

void RobotLink::SetKinematicProperties(const JointType type, const Eigen::Vector3d& axis,
		const Eigen::TransfMatrix& baseTransf, double jointLimMin, double jointLimMax)
{
	type_ = type;
	axis_ = axis;
	baseTransf_ = baseTransf;
	jointLimitMin_ = jointLimMin;
	jointLimitMAX_ = jointLimMax;
}

void RobotLink::SetDynamicProperties(double mass, const Eigen::Vector3d& sizeVect, const Eigen::Vector3d& CoM,
		const Eigen::Matrix3d &Inertia)
{
	mass_ = mass;
	sizeVect_ = sizeVect;
	CoM_ = CoM;
	Inertia_ = Inertia;
}

}

