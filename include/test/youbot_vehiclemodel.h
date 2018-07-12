/**
 * @file youbot_basemodel.h
 *
 *  Created on: May 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */

#ifndef __YOUBOT_BASEMODEL_H__
#define __YOUBOT_BASEMODEL_H__


#include <vector>
#include <algorithm>	// for std::copy

#include "rml/RML.h"

namespace rml {

class YouBotVehicleModel : public VehicleModel
{

public:
	YouBotVehicleModel(const std::string id);
	virtual ~YouBotVehicleModel();

};

}

#endif /* __YOUBOT_BASEMODEL_H__ */
