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
				mass_(1), linkIndex_(0)
{
	InitVectors();
}

rml::RobotLink::RobotLink(const RobotLink& other) : mass_(other.mass_), jointLimitsMin_(other.jointLimitsMin_),
		jointLimitsMAX_(other.jointLimitsMAX_), linkIndex_(other.linkIndex_), CoM_(other.CoM_), Inertia_(other.Inertia_),
		lengthVec_(other.lengthVec_)
{}

RobotLink& rml::RobotLink::operator =(RobotLink other) {
	swap(*this, other);
	return *this;
}

RobotLink::RobotLink(const double mass, const Eigen::Vector3d& lengthVec, const Eigen::Vector3d& CoM, const Eigen::MatrixXd& Inertia) :
				mass_(mass), lengthVec_(lengthVec), CoM_(CoM), Inertia_(Inertia), linkIndex_(0)
{
	InitVectors();
}

void RobotLink::Initialize(const double mass, const Eigen::Vector3d& lengthVec, const Eigen::Vector3d& CoM, const Eigen::MatrixXd &Inertia)
{
	mass_ = mass;
	lengthVec_ = lengthVec;
	CoM_ = CoM;
	Inertia_ = Inertia;

	InitVectors();
	//cout << "Link " << linkIndex_ << " Initialized!" << endl;
}


RobotLink::~RobotLink()
{
	// TODO Auto-generated destructor stub
}

int RobotLink::GetIndex()
{
	return linkIndex_;
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

RobotLink RobotLink::Generator(const double mass, const Eigen::Vector3d& lengthVec, const Eigen::Vector3d& CoM, const Eigen::MatrixXd& Inertia)
{
	RobotLink tempLink(mass, lengthVec, CoM, Inertia);
	return tempLink;

}

void swap(rml::RobotLink& first, rml::RobotLink& second) {

	using std::swap;
	swap(first.mass_,second.mass_);
	swap(first.jointLimitsMin_,second.jointLimitsMin_);
    swap(first.jointLimitsMAX_,second.jointLimitsMAX_);
    swap(first.linkIndex_,second.linkIndex_);
    swap(first.CoM_,second.CoM_);
    swap(first.Inertia_,second.Inertia_);
    swap(first.mass_,second.mass_);
    swap(first.lengthVec_,second.lengthVec_);
}

}


