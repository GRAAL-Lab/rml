/*
 * Defines.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "rml/Types.h"


namespace rml{

EulerYPR::EulerYPR() : yaw_(0.0), pitch_(0.0), roll_(0.0) {}
EulerYPR::EulerYPR(double yaw, double pitch, double roll) :  yaw_(yaw), pitch_(pitch), roll_(roll) {}
EulerYPR::EulerYPR(Eigen::Vector3d vec3) {
	yaw_ = vec3(0);
	pitch_= vec3(1);
	roll_ = vec3(2);
}

double EulerYPR::GetYaw() const {
	return yaw_;
}

void EulerYPR::SetYaw(double yaw) {
	this->yaw_ = yaw;
}

double EulerYPR::GetPitch() const {
	return pitch_;
}

void EulerYPR::SetPitch(double pitch) {
	this->pitch_ = pitch;
}

double EulerYPR::GetRoll() const {
	return roll_;
}

void EulerYPR::SetRoll(double roll) {
	this->roll_ = roll;
}

void EulerYPR::SetYPR(double yaw, double pitch, double roll){
	*this = EulerYPR(yaw, pitch, roll);
}

void EulerYPR::SetYPR(Eigen::Vector3d& vec3){
	*this = EulerYPR(vec3);
}

Eigen::Vector3d EulerYPR::ToVect3() const{
	return Eigen::Vector3d(yaw_, pitch_, roll_);
}


/**
 * R = Rz(yaw)*Ry(pitch)*Rx(roll)
 */
Eigen::RotMatrix EulerYPR::ToRotMatrix() const {

	Eigen::Matrix3d n =
	n = Eigen::AngleAxisd(this->GetYaw(), Eigen::Vector3d::UnitZ())
		*Eigen::AngleAxisd(this->GetPitch(), Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(this->GetRoll(), Eigen::Vector3d::UnitX());
	return n;
}

}
