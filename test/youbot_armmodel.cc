/**
 * @file youbot_armmodel.cc
 *
 *  Created on: Mar 10, 2017
 *     @author: Francesco Wanderlingh
 *      		GRAAL Lab (DIBRIS)
 *      		Università Degli Studi di Genova
 */


#ifndef CMAT_STANDALONE
#define CMAT_STANDALONE
#endif

#include "test/youbot_armmodel.h"

#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

//#define DBG_PRINT

using std::cout;
using std::endl;

namespace rml {

YouBotArmModel::YouBotArmModel()
{
	int numJoints = 5;
	std::vector<Eigen::TransfMatrix> biTri(numJoints);

	biTri.at(0)(0,0) = 1;      biTri.at(0)(0,1) = 0;      biTri.at(0)(0,2) = 0;  biTri.at(0)(0,3) = 0;
	biTri.at(0)(1,0) = 0;      biTri.at(0)(1,1) = -1;     biTri.at(0)(1,2) = 0;  biTri.at(0)(1,3) = 0;
	biTri.at(0)(2,0) = 0;      biTri.at(0)(2,1) = 0;      biTri.at(0)(2,2) = -1; biTri.at(0)(2,3) = 0.072;
	biTri.at(0)(3,0) = 0;      biTri.at(0)(3,1) = 0;      biTri.at(0)(3,2) = 0;  biTri.at(0)(3,3) = 1;

	biTri.at(1)(0,0) = 1;      biTri.at(1)(0,1) = 0;      biTri.at(1)(0,2) = 0;  biTri.at(1)(0,3) = 0;
	biTri.at(1)(1,0) = 0;      biTri.at(1)(1,1) = 0;      biTri.at(1)(1,2) = 1;  biTri.at(1)(1,3) = 0;
	biTri.at(1)(2,0) = 0;      biTri.at(1)(2,1) = -1;     biTri.at(1)(2,2) = 0;  biTri.at(1)(2,3) = -0.075;
	biTri.at(1)(3,0) = 0;      biTri.at(1)(3,1) = 0;      biTri.at(1)(3,2) = 0;  biTri.at(1)(3,3) = 1;

	biTri.at(2)(0,0) = 1;      biTri.at(2)(0,1) = 0;      biTri.at(2)(0,2) = 0;  biTri.at(2)(0,3) = 0;
	biTri.at(2)(1,0) = 0;      biTri.at(2)(1,1) = 1;      biTri.at(2)(1,2) = 0;  biTri.at(2)(1,3) = 0.155;
	biTri.at(2)(2,0) = 0;      biTri.at(2)(2,1) = 0;      biTri.at(2)(2,2) = 1;  biTri.at(2)(2,3) = 0;
	biTri.at(2)(3,0) = 0;      biTri.at(2)(3,1) = 0;      biTri.at(2)(3,2) = 0;  biTri.at(2)(3,3) = 1;

	biTri.at(3)(0,0) = 1;      biTri.at(3)(0,1) = 0;      biTri.at(3)(0,2) = 0;  biTri.at(3)(0,3) = 0;
	biTri.at(3)(1,0) = 0;      biTri.at(3)(1,1) = 1;      biTri.at(3)(1,2) = 0;  biTri.at(3)(1,3) = 0.135;
	biTri.at(3)(2,0) = 0;      biTri.at(3)(2,1) = 0;      biTri.at(3)(2,2) = 1;  biTri.at(3)(2,3) = 0;
	biTri.at(3)(3,0) = 0;      biTri.at(3)(3,1) = 0;      biTri.at(3)(3,2) = 0;  biTri.at(3)(3,3) = 1;

	biTri.at(4)(0,0) = 1;      biTri.at(4)(0,1) = 0;      biTri.at(4)(0,2) = 0;  biTri.at(4)(0,3) = 0;
	biTri.at(4)(1,0) = 0;      biTri.at(4)(1,1) = 0;      biTri.at(4)(1,2) = -1; biTri.at(4)(1,3) = 0.113;
	biTri.at(4)(2,0) = 0;      biTri.at(4)(2,1) = 1;      biTri.at(4)(2,2) = 0;  biTri.at(4)(2,3) = 0;
	biTri.at(4)(3,0) = 0;      biTri.at(4)(3,1) = 0;      biTri.at(4)(3,2) = 0;  biTri.at(4)(3,3) = 1;

	for (int i = 0; i < numJoints; ++i) {
		//std::cout << "Adding link " << i << std::endl;
		AddLink(JointType::Revolute, Eigen::Vector3d::UnitZ(), biTri.at(i));
	}

	hasBeenInitialized_ = true;

	eTt_(0,0) =  1;   eTt_(0,1) =  0;   eTt_(0,2) =  0;   eTt_(0,3) = 0;
	eTt_(1,0) =  0;   eTt_(1,1) =  1;   eTt_(1,2) =  0;   eTt_(1,3) = 0;
	eTt_(2,0) =  0;   eTt_(2,1) =  0;   eTt_(2,2) =  1;   eTt_(2,3) = 0;
	eTt_(3,0) =  0;   eTt_(3,1) =  0;   eTt_(3,2) =  0;   eTt_(3,3) = 1;

}

YouBotArmModel::~YouBotArmModel()
{
	// TODO Auto-generated destructor stub
}


/*
void YouBotArmModel::EvaluatedJdq(CMAT::Matrix* dJdq)
{
	double q[5], out1[30], out2[30], out3[30], out4[30], out5[30];

	for (int i = 0; i < 5; i++)
	{
		q[i] = q_(i + 1);
	}

	EvaluatedJdq(q, out1, out2, out3, out4, out5);

	dJdq[0] = CMAT::Matrix(6, 5, out1);
	dJdq[1] = CMAT::Matrix(6, 5, out2);
	dJdq[2] = CMAT::Matrix(6, 5, out3);
	dJdq[3] = CMAT::Matrix(6, 5, out4);
	dJdq[4] = CMAT::Matrix(6, 5, out5);

}

void YouBotArmModel::EvaluatedJdq(double* q, double* out1, double* out2, double* out3, double* out4, double* out5)
{
	double cos_q1, sin_q1;
	double cos_q2, sin_q2;
	double cos_q3, sin_q3;
	double cos_q4, sin_q4;
	double cos_q5, sin_q5;
	cos_q1 = cos(q[0]);
	sin_q1 = sin(q[0]);
	cos_q2 = cos(q[1]);
	sin_q2 = sin(q[1]);
	cos_q3 = cos(q[2]);
	sin_q3 = sin(q[2]);
	cos_q4 = cos(q[3]);
	sin_q4 = sin(q[3]);
	cos_q5 = cos(q[4]);
	sin_q5 = sin(q[4]);
	out1[0] = 0.0;
	out1[1] = 0.0;
	out1[2] = 0.0;
	out1[3] = 0.155*cos_q1*sin_q2 + 0.113*sin_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*sin_q3 + 0.135*cos_q1*cos_q3*sin_q2;
	out1[4] = 0.113*sin_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) - 0.155*sin_q1*sin_q2 - 0.113*cos_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - 0.135*cos_q2*sin_q1*sin_q3 - 0.135*cos_q3*sin_q1*sin_q2;
	out1[5] = 0.0;
	out1[6] = -1.0*cos_q1;
	out1[7] = sin_q1;
	out1[8] = 0.0;
	out1[9] = sin_q1*(0.155*cos_q2 + 0.135*cos_q2*cos_q3 - 0.135*sin_q2*sin_q3 + 0.113*cos_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out1[10] = cos_q1*(0.155*cos_q2 + 0.135*cos_q2*cos_q3 - 0.135*sin_q2*sin_q3 + 0.113*cos_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out1[11] = 0.0;
	out1[12] = -1.0*cos_q1;
	out1[13] = sin_q1;
	out1[14] = 0.0;
	out1[15] = sin_q1*(0.135*cos_q2*cos_q3 - 0.135*sin_q2*sin_q3 + 0.113*cos_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out1[16] = cos_q1*(0.135*cos_q2*cos_q3 - 0.135*sin_q2*sin_q3 + 0.113*cos_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out1[17] = 0.0;
	out1[18] = -1.0*cos_q1;
	out1[19] = sin_q1;
	out1[20] = 0.0;
	out1[21] = sin_q1*(0.113*cos_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out1[22] = cos_q1*(0.113*cos_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out1[23] = 0.0;
	out1[24] = sin_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) - 1.0*cos_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2);
	out1[25] = - 1.0*sin_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 1.0*cos_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2);
	out1[26] = 0.0;
	out1[27] = 0.0;
	out1[28] = 0.0;
	out1[29] = 0.0;
	out2[0] = 0.0;
	out2[1] = 0.0;
	out2[2] = 0.0;
	out2[3] = 0.155*cos_q2*sin_q1 - 0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) - 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - 0.135*sin_q1*sin_q2*sin_q3 + 0.135*cos_q2*cos_q3*sin_q1;
	out2[4] = 0.155*cos_q1*cos_q2 + 0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*cos_q3 - 0.135*cos_q1*sin_q2*sin_q3;
	out2[5] = 0.0;
	out2[6] = 0.0;
	out2[7] = 0.0;
	out2[8] = 0.0;
	out2[9] = cos_q1*(0.155*sin_q2 + 0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out2[10] = -1.0*sin_q1*(0.155*sin_q2 + 0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out2[11] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) - 0.155*cos_q2*sin_q1 + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) + 0.135*sin_q1*sin_q2*sin_q3 - 0.135*cos_q2*cos_q3*sin_q1) - 1.0*cos_q1*(0.155*cos_q1*cos_q2 + 0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*cos_q3 - 0.135*cos_q1*sin_q2*sin_q3);
	out2[12] = 0.0;
	out2[13] = 0.0;
	out2[14] = 0.0;
	out2[15] = cos_q1*(0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out2[16] = -1.0*sin_q1*(0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out2[17] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) + 0.135*sin_q1*sin_q2*sin_q3 - 0.135*cos_q2*cos_q3*sin_q1) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*cos_q3 - 0.135*cos_q1*sin_q2*sin_q3);
	out2[18] = 0.0;
	out2[19] = 0.0;
	out2[20] = 0.0;
	out2[21] = cos_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out2[22] = -1.0*sin_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out2[23] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2)) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2));
	out2[24] = cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 1.0*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2);
	out2[25] = cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2);
	out2[26] = sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2);
	out2[27] = 0.0;
	out2[28] = 0.0;
	out2[29] = 0.0;
	out3[0] = 0.0;
	out3[1] = 0.0;
	out3[2] = 0.0;
	out3[3] = 0.135*cos_q2*cos_q3*sin_q1 - 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) - 0.135*sin_q1*sin_q2*sin_q3 - 0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1);
	out3[4] = 0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*cos_q3 - 0.135*cos_q1*sin_q2*sin_q3;
	out3[5] = 0.0;
	out3[6] = 0.0;
	out3[7] = 0.0;
	out3[8] = 0.0;
	out3[9] = cos_q1*(0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out3[10] = -1.0*sin_q1*(0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out3[11] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) + 0.135*sin_q1*sin_q2*sin_q3 - 0.135*cos_q2*cos_q3*sin_q1) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*cos_q3 - 0.135*cos_q1*sin_q2*sin_q3);
	out3[12] = 0.0;
	out3[13] = 0.0;
	out3[14] = 0.0;
	out3[15] = cos_q1*(0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out3[16] = -1.0*sin_q1*(0.135*cos_q2*sin_q3 + 0.135*cos_q3*sin_q2 + 0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out3[17] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2) + 0.135*sin_q1*sin_q2*sin_q3 - 0.135*cos_q2*cos_q3*sin_q1) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2) + 0.135*cos_q1*cos_q2*cos_q3 - 0.135*cos_q1*sin_q2*sin_q3);
	out3[18] = 0.0;
	out3[19] = 0.0;
	out3[20] = 0.0;
	out3[21] = cos_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out3[22] = -1.0*sin_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out3[23] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2)) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2));
	out3[24] = cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 1.0*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2);
	out3[25] = cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2);
	out3[26] = sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2);
	out3[27] = 0.0;
	out3[28] = 0.0;
	out3[29] = 0.0;
	out4[0] = 0.0;
	out4[1] = 0.0;
	out4[2] = 0.0;
	out4[3] = - 0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) - 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2);
	out4[4] = 0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2);
	out4[5] = 0.0;
	out4[6] = 0.0;
	out4[7] = 0.0;
	out4[8] = 0.0;
	out4[9] = cos_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out4[10] = -1.0*sin_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out4[11] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2)) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2));
	out4[12] = 0.0;
	out4[13] = 0.0;
	out4[14] = 0.0;
	out4[15] = cos_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out4[16] = -1.0*sin_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out4[17] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2)) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2));
	out4[18] = 0.0;
	out4[19] = 0.0;
	out4[20] = 0.0;
	out4[21] = cos_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out4[22] = -1.0*sin_q1*(0.113*sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + 0.113*cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2));
	out4[23] = sin_q1*(0.113*cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + 0.113*sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2)) - 1.0*cos_q1*(0.113*cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 0.113*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2));
	out4[24] = cos_q4*(cos_q1*cos_q2*cos_q3 - 1.0*cos_q1*sin_q2*sin_q3) - 1.0*sin_q4*(cos_q1*cos_q2*sin_q3 + cos_q1*cos_q3*sin_q2);
	out4[25] = cos_q4*(sin_q1*sin_q2*sin_q3 - 1.0*cos_q2*cos_q3*sin_q1) + sin_q4*(cos_q2*sin_q1*sin_q3 + cos_q3*sin_q1*sin_q2);
	out4[26] = sin_q4*(cos_q2*cos_q3 - 1.0*sin_q2*sin_q3) + cos_q4*(cos_q2*sin_q3 + cos_q3*sin_q2);
	out4[27] = 0.0;
	out4[28] = 0.0;
	out4[29] = 0.0;
	out5[0] = 0.0;
	out5[1] = 0.0;
	out5[2] = 0.0;
	out5[3] = 0.0;
	out5[4] = 0.0;
	out5[5] = 0.0;
	out5[6] = 0.0;
	out5[7] = 0.0;
	out5[8] = 0.0;
	out5[9] = 0.0;
	out5[10] = 0.0;
	out5[11] = 0.0;
	out5[12] = 0.0;
	out5[13] = 0.0;
	out5[14] = 0.0;
	out5[15] = 0.0;
	out5[16] = 0.0;
	out5[17] = 0.0;
	out5[18] = 0.0;
	out5[19] = 0.0;
	out5[20] = 0.0;
	out5[21] = 0.0;
	out5[22] = 0.0;
	out5[23] = 0.0;
	out5[24] = 0.0;
	out5[25] = 0.0;
	out5[26] = 0.0;
	out5[27] = 0.0;
	out5[28] = 0.0;
	out5[29] = 0.0;
}
*/

}

