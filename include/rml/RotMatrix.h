/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_ROTMATRIX_H_
#define INCLUDE_ROTMATRIX_H_

#include "Types.h"

namespace Eigen {

/**
 * \class RotationMatrix
 *
 * \brief This class extends the Eigen::Matrix3d
 *
 * \details The Eigen::RotMatrix represents a cartesian rotation matrix. Is an extension of
 * the Matrix3d class that defaults the constructor to an identity matrix, with the addition
 * of member functions to convert to different representations such as: Cartesian 6x6 rotation
 * matrix, rml::EulerRPY and Eigen::Quaterniond reprensentation.
 *
 */
class RotationMatrix : public Eigen::Matrix<double,3,3,Eigen::DontAlign> {
public:
    RotationMatrix();
    RotationMatrix(Eigen::Quaterniond q);

    // This constructor allows you to construct RotMatrix from Eigen expressions
    template <typename OtherDerived>
    RotationMatrix(const Eigen::MatrixBase<OtherDerived>& other)
        : Eigen::Matrix<double,3,3,Eigen::DontAlign>(other)
    {
    }
    // This method allows you to assign Eigen expressions to RotMatrix
    template <typename OtherDerived>
    RotationMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Eigen::Matrix<double,3,3,Eigen::DontAlign>::operator=(other);
        return *this;
    }

    Eigen::Matrix6d CartesianRotationMatrix() const;

    rml::EulerRPY ToEulerRPY() const;

    Eigen::Quaterniond ToQuaternion() const;

    //Computes the integral of a rotation matrix (Out = e^[wdt^] * Rin )
    RotationMatrix StrapDown(const Vector3d& w, double dt) const;
};
}

#endif /* INCLUDE_ROTMATRIX_H_ */
