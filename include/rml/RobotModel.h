/*
 * RobotModel.h
 *
 *  Created on: Feb 27, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_ROBOTMODEL_H_
#define INCLUDE_RML_ROBOTMODEL_H_

#include <rml/VehicleModel.h>
#include <rml/ArmModel.h>

/**
 * \class RobotModel
 *
 * \ingroup RML
 *
 * \brief This class provides a container for storing multi-arm mobile manipulators,
 * including a series of model related functions.
 *
 * Detailed description TODO.
 *
 *
 * \author (last to touch it) fw
 *
 * \date 2018/02/28 12:06:20
 *
 * Contact: francesco.wanderlingh@dibris.unige.it
 *
 * Created on: Tue Feb 27 10:22:30 2018
 *
 *
 */

namespace rml {

class RobotModel {

	VehicleModel vehicle_;
	std::vector<ArmModel> arms_;

public:
	RobotModel();
	virtual ~RobotModel();

	const ArmModel& GetArm(int index) const {
		return arms_.at(index);
	}

	const VehicleModel& GetVehicle() const {
		return vehicle_;
	}
};

} /* namespace rml */

#endif /* INCLUDE_RML_ROBOTMODEL_H_ */
