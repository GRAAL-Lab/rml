/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_EULER_RPY_H_
#define INCLUDE_EULER_RPY_H_

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "Types.h"

namespace rml {

/**
 * @brief Euler RPY angle representation.
 *
 * This class stores the orientation of a rigid body in terms of the roll, pitch and yaw
 * rotation. The order in which the rotation are applied follows the z-y-x convention:
 * rotate around yaw, then pitch, then roll.
 */
class EulerRPY {
public:
    EulerRPY();
    EulerRPY(double roll, double pitch, double yaw);
    EulerRPY(Eigen::Vector3d vec3);
    EulerRPY(Eigen::Quaterniond q);

    virtual ~EulerRPY();

    friend std::ostream& operator<<(std::ostream& os, EulerRPY const& a)
    {
        return os << a.roll_ << "\t" << a.pitch_ << "\t" << a.yaw_ << "\t";
    }

    auto Yaw() const -> double { return this->yaw_; }
    auto Yaw(double yaw) -> void { this->yaw_ = yaw; }

    auto Pitch() const -> double { return this->pitch_; }
    auto Pitch(double pitch) -> void { this->pitch_ = pitch; }

    auto Roll() const -> double { return this->roll_; }
    auto Roll(double roll) -> void { this->roll_ = roll; }

    auto RPY(double roll, double pitch, double yaw) -> void
    {
        this->roll_ = roll;
        this->pitch_ = pitch;
        this->yaw_ = yaw;
    }

    auto RPY(Eigen::Vector3d rpy) -> void
    {
        this->roll_ = rpy[0];
        this->pitch_ = rpy[1];
        this->yaw_ = rpy[2];
    }

    auto ToVector() const -> Eigen::Vector3d { return Eigen::Vector3d(roll_, pitch_, yaw_); }

    Eigen::RotMatrix ToRotMatrix() const;

    /**
   * @brief GetDerivative computes the euler rates (derivative) of 'this', given the angular
   * velocity omega of the frame. Omega is interpreted as projected on the frame itself (body)
   * and not the inertial frame.
   * @param[in] omega
   * @return the derivative of this
   */
    Eigen::Vector3d Derivative(const Eigen::Vector3d& omega) const;
    /**
   * @brief GetOmega computes the angular velocity of the frame given euler rates (derivative) of 'this',
   * Omega is projected on the frame itself (body)  and not the inertial frame.
   * @param[in] rpyDerivatives of this
   * @return omega
   */
    Eigen::Vector3d Omega(const Eigen::Vector3d& rpyDerivarives) const;
    Eigen::Quaterniond ToQuaternion() const;

private:
    double roll_, pitch_, yaw_;
};
}

#endif /* INCLUDE_EULER_RPY_H_ */
