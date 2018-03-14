/*
 * RobotLink.h
 *
 *  Created on: Jul 8, 2015
 *      Author: francescow
 */

#ifndef SRC_ROBOTLINK_H_
#define SRC_ROBOTLINK_H_

#include <rml/Types.h>

namespace rml {

enum class JointType : uint8_t { Fixed, Revolute, Prismatic };

class RobotLink
{
    double mass_;
    Eigen::Vector3d lengthVec_;
    Eigen::Vector3d CoM_;
    Eigen::MatrixXd Inertia_;
	double jointLimitsMin_;
	double jointLimitsMAX_;

    void SetPhysicalProperties(const double mass, const Eigen::Vector3d& sizes, const Eigen::Vector3d& CoM, const Eigen::MatrixXd& Inertia);
    void InitVectors();

public:
    /**
     * The interaction forces and moments associated with the link "i" are relative to the one between
     * "i" and "i-1". While the "self" forces are the one relative to the acceleration of the CoM_i.
     */
    Eigen::Vector3d self_f, self_n;
    Eigen::Vector3d inter_f, inter_n;

	JointType type_;
	Eigen::Vector3d axis_;
	Eigen::TransfMatrix baseTransf_;

    RobotLink();
    RobotLink(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double joinLimMax);

    const float Mass() 				const { return mass_; }
    const Eigen::Vector3d& LenghtVec() 	const { return lengthVec_; }
    const Eigen::Vector3d& CoM() 		const { return CoM_; }
    const Eigen::MatrixXd& Inertia() 	const { return Inertia_; }

    int GetIndex();

    virtual ~RobotLink();

	double GetJointLimitsMax() const {
		return jointLimitsMAX_;
	}

	/*void SetJointLimitsMax(double jointLimitsMax) {
		jointLimitsMAX_ = jointLimitsMax;
	}*/

	double GetJointLimitsMin() const {
		return jointLimitsMin_;
	}

	/*void SetJointLimitsMin(double jointLimitsMin) {
		jointLimitsMin_ = jointLimitsMin;
	}*/
};

}

#endif /* SRC_ROBOTLINK_H_ */
