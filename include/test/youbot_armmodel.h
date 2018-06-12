/**
 * @file youbot_armmodel.h
 *
 *  Created on: Mar 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */

#ifndef __YOUBOT_ARMMODEL_H__
#define __YOUBOT_ARMMODEL_H__


#include <vector>
#include <algorithm>	// for std::copy
#include <rml/RML.h>

namespace rml {

class YouBotArmModel : public ArmModel
{

public:
    YouBotArmModel(std::string id);
	virtual ~YouBotArmModel();
};

}

#endif /* __YOUBOT_ARMMODEL_H__ */
