/**
 * @file twolinks_armmodel.h
 *
 *  Created on: Mar 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */

#ifndef __TWOLINKS_ARMMODEL_H__
#define __TWOLINKS_ARMMODEL_H__


#include <vector>
#include <algorithm>	// for std::copy
#include <rml/RML.h>

namespace rml {

class TwoLinksArmModel : public ArmModel
{

public:
    TwoLinksArmModel(std::string id);
	virtual ~TwoLinksArmModel();
};

}

#endif /* __TWOLINKS_ARMMODEL_H__ */
