/**
 * @file youbot_basemodel.cc
 *
 *  Created on: May 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */

#include "test/youbot_vehiclemodel.h"

using std::cout;
using std::endl;

namespace rml {

YouBotVehicleModel::YouBotVehicleModel(const std::string id)
    : VehicleModel(id)
{
    Eigen::Matrix6d vJac;
    vJac.setZero();
    /**
	 * Giving that the velocity is standardized to the form of wx wy wz x y z,
	 *
	 */
    vJac(0, 3) = 0.0;
    vJac(0, 4) = 0.0;
    vJac(0, 5) = 0.0;
    vJac(1, 3) = 0.0;
    vJac(1, 4) = 0.0;
    vJac(1, 5) = 0.0;
    vJac(2, 3) = 1.0;
    vJac(2, 4) = 0.0;
    vJac(2, 5) = 0.0;
    vJac(3, 3) = 0.0;
    vJac(3, 4) = 1.0;
    vJac(3, 5) = 0.0;
    vJac(4, 3) = 0.0;
    vJac(4, 4) = 0.0;
    vJac(4, 5) = 1.0;
    vJac(5, 3) = 0.0;
    vJac(5, 4) = 0.0;
    vJac(5, 5) = 0.0;
    Jacobian(vJac);

    //	vJv_.setZero();
    //	vJv_(2,3) = 1.0;
    //	vJv_(3,4) = 1.0;
    //	vJv_(4,5) = 1.0;

    //	vTb_(1,1) =  1.0;   vTb_(1,2) =  0.0;   vTb_(1,3) =  0.0;   vTb_(1,4) = -0.143;
    //	vTb_(2,1) =  0.0;   vTb_(2,2) =  1.0;   vTb_(2,3) =  0.0;   vTb_(2,4) = 0.0;
    //	vTb_(3,1) =  0.0;   vTb_(3,2) =  0.0;   vTb_(3,3) =  1.0;   vTb_(3,4) = 0.0;
    //	vTb_(4,1) =  0.0;   vTb_(4,2) =  0.0;   vTb_(4,3) =  0.0;   vTb_(4,4) = 1.0;

    //	T_(1,1) =  1;	T_(1,2) =  0;	T_(1,3) =  0;	T_(1,4) = 0;
    //	T_(2,1) =  0;	T_(2,2) =  1;	T_(2,3) =  0;	T_(2,4) = -0.35;
    //	T_(3,1) =  0;	T_(3,2) =  0;	T_(3,3) =  1;	T_(3,4) = 0;
    //	T_(4,1) =  0;	T_(4,2) =  0;	T_(4,3) =  0;	T_(4,4) = 1;
}

YouBotVehicleModel::~YouBotVehicleModel()
{
    // TODO Auto-generated destructor stub
}

//void YouBotVehicleModel::GetPosition(CMAT::Vect6& vehiclePos) {
//	CMAT::Vect6 v6;
//	v6(1) = fbkPosition_(1);   // yaw
//	v6(2) = 0.0;     // pitch
//	v6(3) = 0.0;     // roll
//	v6(4) = fbkPosition_(2);   // x
//	v6(5) = fbkPosition_(3);   // y
//	v6(6) = 0.0;     // z
//	vehiclePos = v6;
//}
//
//void YouBotVehicleModel::Get6DVelocity(CMAT::Vect6& vehicleVel) {
//	CMAT::Vect6 v6;
//	v6(1) = 0;          // w_x
//	v6(2) = 0;          // w_y
//	v6(3) = qdot_(1);   // w_z
//	v6(4) = qdot_(2);   // v_x
//	v6(5) = qdot_(3);   // v_y
//	v6(6) = 0;          // v_z
//
//	CMAT::TransfMatrix wTv;
//	EvaluatewTv(wTv);
//
//	vehicleVel = wTv.GetRotMatrix().Transpose().GetCartesianRotationMatrix() * v6;
//}

}
