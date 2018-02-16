/**
 * @file baxterLeft_armmodel.h
 *
 *  Created on: Mar 10, 2016
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */

#ifndef __BAXTERLEFT_ARMMODEL_H__
#define __BAXTERLEFT_ARMMODEL_H__


#include <vector>
#include <algorithm>	// for std::copy
#include <ortos/ortos.h>
#include <cmat/cmat.h>

#include "ctrl_defines.h"
#include "ctrl_armmodel.h"

namespace CTRL {

class BaxterLeftArmModel : public ArmModel_CRTP<BaxterLeftArmModel>
{

public:
	BaxterLeftArmModel();
	virtual ~BaxterLeftArmModel();

    virtual void InitMatrix();

    virtual void EvaluatedJdq(CMAT::Matrix* dJdq);

private:
    virtual void EvaluatedJdq(double* q, double* out1, double* out2, double* out3, double* out4, double* out5, double* out6, double* out7);

};

}

#endif /* __BAXTERLEFT_ARMMODEL_H__ */
