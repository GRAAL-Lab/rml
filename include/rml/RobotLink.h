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

class RobotLink
{
    double mass_;
    Eigen::Vector3d lengthVec_;
    Eigen::Vector3d CoM_;
    Eigen::MatrixXd Inertia_;
	double jointLimitsMin_;
	double jointLimitsMAX_;

    /// For debugging purposes
    int linkIndex_;
    //static int globalLinkIndex;

    void Initialize(const double mass, const Eigen::Vector3d& dims, const Eigen::Vector3d& CoM, const Eigen::MatrixXd& Inertia);
    void InitVectors();

public:
    /**
     * The interaction forces and moments associated with the link "i" are relative to the one between
     * "i" and "i-1". While the "self" forces are the one relative to the acceleration of the CoM_i.
     */
    Eigen::Vector3d self_f, self_n;
    Eigen::Vector3d inter_f, inter_n;

    RobotLink();
	/**
	 * @brief Copy constructor
	 */
    RobotLink(const RobotLink& other);

	/**
	 * @brief Specialized swap() to implement the copy-assignment-operator by reusing the copy-constructor
	 */
	friend void swap(RobotLink& first, RobotLink& second);

	/**
	 * @brief Copy Assignment Operator
	 */
	RobotLink& operator=(RobotLink other);

    RobotLink(const double mass, const Eigen::Vector3d& dims, const Eigen::Vector3d& CoM, const Eigen::MatrixXd& Inertia);

    static RobotLink Generator(const double mass, const Eigen::Vector3d& dims, const Eigen::Vector3d& CoM, const Eigen::MatrixXd& Inertia);

    const float Mass() 				const { return mass_; }
    const Eigen::Vector3d& LenghtVec() 	const { return lengthVec_; }
    const Eigen::Vector3d& CoM() 		const { return CoM_; }
    const Eigen::MatrixXd& Inertia() 	const { return Inertia_; }

    int GetIndex();

    virtual ~RobotLink();
};

}

#endif /* SRC_ROBOTLINK_H_ */
