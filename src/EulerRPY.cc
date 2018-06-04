/*
 * Defines.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "EulerRPY.h"
#include "PseudoInverse.h"

namespace rml{

EulerRPY::EulerRPY() : roll_(0.0), pitch_(0.0), yaw_(0.0) {}
EulerRPY::EulerRPY(double roll, double pitch, double yaw) : roll_(roll), pitch_(pitch), yaw_(yaw) {}
EulerRPY::EulerRPY(Eigen::Vector3d vec3) {
	roll_ = vec3(0);
	pitch_= vec3(1);
	yaw_ = vec3(2);
}

EulerRPY::EulerRPY(Eigen::Quaterniond q) {
	*this = Eigen::RotMatrix(q.toRotationMatrix()).ToEulerRPY();
}


double EulerRPY::GetRoll() const {
	return roll_;
}

void EulerRPY::SetRoll(double roll) {
	this->roll_ = roll;
}

double EulerRPY::GetPitch() const {
	return pitch_;
}

void EulerRPY::SetPitch(double pitch) {
	this->pitch_ = pitch;
}

double EulerRPY::GetYaw() const {
	return yaw_;
}

void EulerRPY::SetYaw(double yaw) {
	this->yaw_ = yaw;
}

void EulerRPY::SetRPY(double roll, double pitch, double yaw){
	*this = EulerRPY(roll, pitch, yaw);
}

void EulerRPY::SetRPY(Eigen::Vector3d& vec3){
	*this = EulerRPY(vec3);
}

Eigen::Vector3d EulerRPY::ToVect3() const{
	return Eigen::Vector3d(yaw_, pitch_, roll_);
}

Eigen::Quaterniond EulerRPY::ToQuaternion() const
{
	return this->ToRotMatrix().ToQuaternion();
}

/**
 * R = Rz(yaw)*Ry(pitch)*Rx(roll)
 */
Eigen::RotMatrix EulerRPY::ToRotMatrix() const {

	Eigen::Matrix3d n;
	n = Eigen::AngleAxisd(this->GetYaw(), Eigen::Vector3d::UnitZ())
		*Eigen::AngleAxisd(this->GetPitch(), Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(this->GetRoll(), Eigen::Vector3d::UnitX());
	return n;
}

Eigen::Vector3d EulerRPY::GetDerivative(Eigen::Vector3d omega) const throw (std::exception) {
	Eigen::Matrix3d S;
  S << cos(yaw_)*cos(pitch_), -sin(yaw_),    0,
       sin(yaw_)*cos(pitch_),  cos(yaw_),    0,
      -sin(pitch_)		  ,       0 		  ,	   1;
	RegularizationData mySvd;
	return RegularizedPseudoInverse(S, mySvd) * this->ToRotMatrix() * omega;
}



} // namespace rml

