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
#include "rml/RML.h"

namespace rml {

class BaxterLeftArmModel : public ArmModel
{

public:
  BaxterLeftArmModel(std::string id);
	virtual ~BaxterLeftArmModel();

    virtual void InitMatrix();

//    virtual void EvaluatedJdq(Eigen::Matrix* dJdq);
//
//private:
//    virtual void EvaluatedJdq(double* q, double* out1, double* out2, double* out3, double* out4, double* out5, double* out6, double* out7);

};

}

#endif /* __BAXTERLEFT_ARMMODEL_H__ */
