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
#include <ortos/ortos.h>
#include <cmat/cmat.h>

#include "ctrl_defines.h"
#include "ctrl_armmodel.h"

namespace CTRL {

class YouBotArmModel : public ArmModel_CRTP<YouBotArmModel>
{
	//friend class ::ArmCtrl;

public:
	YouBotArmModel();
	virtual ~YouBotArmModel();

    virtual void InitMatrix();

    virtual void EvaluatedJdq(CMAT::Matrix* dJdq);

private:
    virtual void EvaluatedJdq(double* q, double* out1, double* out2, double* out3, double* out4, double* out5);

};

}

#endif /* __YOUBOT_ARMMODEL_H__ */
