/*
 * TransformationMatrix.cc
 *
 *  Created on: Feb 23, 2018
 *      Author: fraw
 */

#include "TransfMatrix.h"

namespace Eigen {

TransformationMatrix::TransformationMatrix(void)
    : Eigen::Matrix4d()
{
    *this = Eigen::Matrix4d::Identity();
}

Vector6d TransformationMatrix::ToVector() const
{
    return Eigen::Vector6d{ this->TranslationVector(), this->RotationMatrix().ToEulerRPY().ToVector() };
}

TransformationMatrix TransformationMatrix::Integral(const Vector6d& InputVelocities, double dt) const
{
    TransformationMatrix temp = *this;
    temp.RotationMatrix(RotationMatrix().StrapDown(InputVelocities.AngularVector(), dt));
    temp.TranslationVector(InputVelocities.LinearVector() * dt + TranslationVector());
    return temp;
}
void TransformationMatrix::RotationMatrix(const Eigen::RotationMatrix rotationMatrix)
{
    if (rotationMatrix.cols() == 3 && rotationMatrix.rows() == 3) {
        this->block(0, 0, 3, 3) = rotationMatrix;
    } else {
        std::cout << "[TransformationMatrix::RotationMatrix()] WARNING: Size is not 3x3" << std::endl;
    }
}

Eigen::RotationMatrix TransformationMatrix::RotationMatrix() const
{
    return this->block(0, 0, 3, 3);
}
} // namespace Eigen
