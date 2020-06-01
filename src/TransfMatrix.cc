/*
 * TransfMatrix.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "TransfMatrix.h"

namespace Eigen {

TransfMatrix::TransfMatrix(void)
    : Eigen::Matrix4d()
{
    *this = Eigen::Matrix4d::Identity();
}

Vector6d TransfMatrix::XYZ_RPY() const
{
    return Eigen::Vector6d{ this->Transl(), this->RotationMatrix().ToEulerRPY().ToVector() };
}

TransfMatrix TransfMatrix::Integral(const Vector6d& vin, double dt) const
{
    TransfMatrix temp = *this;
    temp.RotationMatrix(RotationMatrix().StrapDown(vin.AngularVector(), dt));
    temp.Transl(vin.LinearVector() * dt + Transl());
    return temp;
}
void TransfMatrix::RotationMatrix(const Eigen::RotMatrix rot)
{
    if (rot.cols() == 3 && rot.rows() == 3) {
        this->block(0, 0, 3, 3) = rot;
    } else {
        std::cout << "[TransfMatrix::SetRotMatrix()] WARNING: Size is not 3x3" << std::endl;
    }
}

Eigen::RotMatrix TransfMatrix::RotationMatrix() const
{
    return this->block(0, 0, 3, 3);
}
} // namespace Eigen
