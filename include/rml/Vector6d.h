/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_VECTOR6D_H_
#define INCLUDE_VECTOR6D_H_

#include "Types.h"

namespace Eigen {

/**
 * @brief A 6d vector generally used for containing pose [x y z r p y] or velocity [vx vy vz wx wy wz] vectors
 */
class Vector6d : public Eigen::Vector6dBase {
public:
    Vector6d();
    // This constructor allows you to construct TransfMatrix from Eigen expressions
    template <typename OtherDerived>
    Vector6d(const Eigen::MatrixBase<OtherDerived>& other)
        : Eigen::Vector6dBase(other)
    {
    }

    // This method allows you to assign Eigen expressions to TransfMatrix
    template <typename OtherDerived>
    Vector6d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Eigen::Vector6dBase::operator=(other);
        return *this;
    }

    template <typename OtherDerived>
    void LinearVector(const Eigen::MatrixBase<OtherDerived>& linear)
    {
        this->block(0, 0, 3, 1) = linear;
    }

    template <typename OtherDerived>
    void AngularVector(const Eigen::MatrixBase<OtherDerived>& angular)
    {
        this->block(3, 0, 3, 1) = angular;
    }

    Vector6d(const Vector3d linear, const Vector3d angular);

    auto LinearVector() const -> Vector3d { return this->block(0, 0, 3, 1); }

    auto AngularVector() const -> Vector3d { return this->block(3, 0, 3, 1); }
};
}

#endif /* INCLUDE_VECTOR6D_H_ */
