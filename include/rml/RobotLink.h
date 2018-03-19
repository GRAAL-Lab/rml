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
	// Kinematic properties
	JointType type_;
	Eigen::Vector3d axis_;
	Eigen::TransfMatrix baseTransf_;
	double jointLimitsMin_;
	double jointLimitsMAX_;

	// Dynamic properties
    double mass_;
    Eigen::Vector3d sizeVect_;
    Eigen::Vector3d CoM_;
    Eigen::Matrix3d Inertia_;

    void InitVectors();

public:
    /**
     * The interaction forces and moments associated with the link "i" are relative to the one between
     * "i" and "i-1". While the "self" forces are the one relative to the acceleration of the CoM_i.
     */
    Eigen::Vector3d self_f_, self_n_;
    Eigen::Vector3d inter_f_, inter_n_;

    RobotLink();
    RobotLink(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double joinLimMax);
    virtual ~RobotLink();

    void SetPhysicalProperties(double mass, const Eigen::Vector3d& sizes, const Eigen::Vector3d& CoM, const Eigen::Matrix3d& Inertia);

    JointType Type()    					const {	return type_;}
    const Eigen::Vector3d Axis()    		const {	return axis_;}
    const Eigen::TransfMatrix BaseTransf() 	const {	return baseTransf_; }
    double JointLimitsMax()    				const {	return jointLimitsMAX_; }
	double JointLimitsMin() 				const {	return jointLimitsMin_;	}

    float Mass() 				   			const { return mass_; }
    const Eigen::Vector3d& Sizes() 			const { return sizeVect_; }
    const Eigen::Vector3d& CoM() 			const { return CoM_; }
    const Eigen::Matrix3d& Inertia()		const { return Inertia_; }
};

}

#endif /* SRC_ROBOTLINK_H_ */
