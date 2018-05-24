/**
 * \file
 *
 * \date 	Feb 20, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_EULER_RPY_H_
#define INCLUDE_EULER_RPY_H_

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "Types.h"

namespace rml{

/**
 * @brief Euler RPY angle representation.
 *
 * This class stores the orientation of a rigid body in terms of the roll, pitch and yaw
 * rotation. The order in which the rotation are applied follows the z-y-x convention:
 * rotate around yaw, then pitch, then roll.
 */
class EulerRPY
{
public:
	EulerRPY();
	EulerRPY(double yaw, double pitch, double roll);
	EulerRPY(Eigen::Vector3d vec3);
	EulerRPY(Eigen::Quaterniond q);

	virtual ~EulerRPY() {}

	friend std::ostream& operator <<(std::ostream& os, EulerRPY const& a)
	{
		return os << a.roll_ <<"\t" << a.pitch_ << "\t" <<  a.yaw_ << "\t";
	}

	double GetYaw() const;
	void SetYaw(double yaw);

	double GetPitch() const;
	void SetPitch(double pitch);

	double GetRoll() const;
	void SetRoll(double roll);

	void SetRPY(double roll, double pitch, double yaw);
	void SetRPY(Eigen::Vector3d& vec3);

	Eigen::Vector3d ToVect3() const;
	Eigen::RotMatrix ToRotMatrix() const;
	Eigen::Vector3d GetDerivative(Eigen::Vector3d omega) const throw (std::exception);
	Eigen::Quaterniond ToQuaternion() const;

private:
	double roll_, pitch_, yaw_ ;
};
}




#endif /* INCLUDE_EULER_RPY_H_ */
