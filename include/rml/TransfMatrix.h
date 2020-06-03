/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_TransformationMatrix_H_
#define INCLUDE_TransformationMatrix_H_

#include "Types.h"

namespace Eigen {

/**
 * \class TransformationMatrix
 *
 * \brief This class extends the Eigen::Matrix4d
 *
 * \details The Eigen::TransformationMatrix represents an homogeneus transformation matrix an extension of the
 *   Matrix4d class that defaults the constructor to an identity matrix, with the addition of member
 *   functions to convert do different representaions, set and extract rotation and translational
 *   parts of it separately (rot and transl parts).
 *
 */
class TransformationMatrix : public Eigen::Matrix4d {
public:
    TransformationMatrix(void);

    // This constructor allows you to construct TransformationMatrix from Eigen expressions
    template <typename OtherDerived>
    TransformationMatrix(const Eigen::MatrixBase<OtherDerived>& other)
        : Eigen::Matrix4d(other)
    {
    }
    // This method allows you to assign Eigen expressions to TransformationMatrix
    template <typename OtherDerived>
    TransformationMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Eigen::Matrix4d::operator=(other);
        return *this;
    }

    Eigen::RotationMatrix RotationMatrix() const;

    void RotationMatrix(const Eigen::RotationMatrix rotationMatrix);

    auto TranslationVector() const -> const Vector3d { return this->block(0, 3, 3, 1); }

    auto TranslationVector(const Vector3d translationVector) -> void { this->block(0, 3, 3, 1) = translationVector; }

    Vector6d ToVector() const;

    TransformationMatrix Integral(const Vector6d& inputVelocities, double dt) const;
};
}

#endif /* INCLUDE_TransformationMatrix_H_ */
