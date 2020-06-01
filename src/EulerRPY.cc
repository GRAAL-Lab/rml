/*
 * Defines.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "EulerRPY.h"
#include "PseudoInverse.h"

namespace rml {

EulerRPY::EulerRPY()
    : roll_(0.0)
    , pitch_(0.0)
    , yaw_(0.0)
{
}
EulerRPY::EulerRPY(double roll, double pitch, double yaw)
    : roll_(roll)
    , pitch_(pitch)
    , yaw_(yaw)
{
}
EulerRPY::EulerRPY(Eigen::Vector3d vec3)
{
    roll_ = vec3(0);
    pitch_ = vec3(1);
    yaw_ = vec3(2);
}

EulerRPY::EulerRPY(Eigen::Quaterniond q) { *this = Eigen::RotMatrix(q.toRotationMatrix()).ToEulerRPY(); }

EulerRPY::~EulerRPY() {}

/**
 * R = Rz(yaw)*Ry(pitch)*Rx(roll)
 */
Eigen::RotMatrix EulerRPY::ToRotMatrix() const
{
    return Eigen::AngleAxisd(this->Yaw(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(this->Pitch(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(this->Roll(), Eigen::Vector3d::UnitX());
}

Eigen::Vector3d EulerRPY::Derivative(const Eigen::Vector3d& omega) const
{
    Eigen::Matrix3d S;
    S << cos(yaw_) * cos(pitch_), -sin(yaw_), 0,
        sin(yaw_) * cos(pitch_), cos(yaw_), 0,
        -sin(pitch_), 0, 1;
    RegularizationData mySvd;
    return RegularizedPseudoInverse(S, mySvd) * this->ToRotMatrix() * omega;
}

Eigen::Vector3d EulerRPY::Omega(const Eigen::Vector3d& rpyDerivarives) const
{
    Eigen::Matrix3d S;
    S << cos(yaw_) * cos(pitch_), -sin(yaw_), 0,
        sin(yaw_) * cos(pitch_), cos(yaw_), 0,
        -sin(pitch_), 0, 1;
    return this->ToRotMatrix().transpose() * S * rpyDerivarives;
}
Eigen::Quaterniond EulerRPY::ToQuaternion() const
{
    return this->ToRotMatrix().ToQuaternion();
}

} // namespace rml
