/**
 * \file
 *
 * \date 	July 08, 2015
 * \author 	Francesco Wanderlingh
 */

#ifndef SRC_ROBOTLINK_H_
#define SRC_ROBOTLINK_H_

#include <map>
#include <string>

#include "Types.h"

namespace rml {

/**
 * \brief Used to describe the type of joint
 */
enum class JointType : uint8_t { Fixed, Revolute, Prismatic };

const std::map<JointType, std::string> JointType2String = {
		{ JointType::Fixed, "Fixed" },
		{ JointType::Revolute, "Revolute" },
		{ JointType::Prismatic, "Prismatic" }
};

/**
 * \brief Basic element of an rml::ArmModel
 */
class RobotLink
{
	// Kinematic properties
	JointType type_;
	Eigen::Vector3d axis_;
	Eigen::TransfMatrix baseTransf_;
	double jointLimitMin_;
	double jointLimitMAX_;

	// Dynamic properties
    double mass_;
    Eigen::Vector3d sizeVect_;
    Eigen::Vector3d CoM_;
    Eigen::Matrix3d Inertia_;

public:

    RobotLink();
    RobotLink(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double jointLimMax);
    virtual ~RobotLink();

    void SetKinematicProperties(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double jointLimMax);
    void SetDynamicProperties(double mass, const Eigen::Vector3d& sizes, const Eigen::Vector3d& CoM, const Eigen::Matrix3d& Inertia);

    JointType Type()    					const {	return type_;}
    const Eigen::Vector3d& Axis()    		const {	return axis_;}
    const Eigen::TransfMatrix& BaseTransf()	const {	return baseTransf_; }
    double JointLimitMax()    				const {	return jointLimitMAX_; }
	double JointLimitMin() 					const {	return jointLimitMin_;	}

    float Mass() 				   			const { return mass_; }
    const Eigen::Vector3d& Sizes() 			const { return sizeVect_; }
    const Eigen::Vector3d& CoM() 			const { return CoM_; }
    const Eigen::Matrix3d& Inertia()		const { return Inertia_; }
};

}

#endif /* SRC_ROBOTLINK_H_ */
