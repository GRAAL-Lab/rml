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
 * \brief Basic element of an ArmModel
 *
 * \details RobotLink collects all the properties for the single links that compose
 * a kinematic chain. It is the input for the ArmModel::AddLink() function. If you
 * use the empty constructor you must then call the function SetKinematicProperties()
 * to get meaningful results since by default everything is set to zero. Otherwise you
 * can use the full constructor where you have to specify all the necessary variables.
 * In both cases you need to provide the following parameters (in this order):
 *   1. The ::JointType
 *   2. Its rotation/translation axis
 *   3. The transformation matrix from the previous link
 *   4. The minimum value for the joint excursion
 *   5. The maximum value for the joint excursion
 *
 * If you want to use the RobotModel for dynamic purposes (such as pass it to the
 * NewtonEuler class) you need also to call the function SetDynamicProperties()
 * where you need to provide the link's:
 *   1. Mass
 *   2. The x-y-z sizes
 *   3. The link center of mass
 *   4. The link inertia matrix
 *
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

    /**
     * \brief Sets the kinematic properties of the link
     *
     * @param[in] type
     * @param[in] axis
     * @param[in] baseTransf
     * @param[in] jointLimMin
     * @param[in] jointLimMax
     */
    void SetKinematicProperties(const JointType type, const Eigen::Vector3d& axis, const Eigen::TransfMatrix& baseTransf, double jointLimMin, double jointLimMax);

    /**
     * \brief Sets the dynamic properties of the link
     *
     * @param[in] mass
     * @param[in] sizes
     * @param[in] CoM
     * @param[in] Inertia
     */
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
