/*
 * Functions.cc
 *
 *  Created on: Feb 21, 2018
 *      Author: fraw
 */

#include <vector>
#include <algorithm>
#include <iostream>
#include <vector>
#include "rml/Functions.h"

namespace rml {

template<typename T, template<class, class ...> class C, class... Args>
std::ostream& operator <<(std::ostream& os, const C<T,Args...>& objs)
{
	os << __PRETTY_FUNCTION__ << '\n';
	for (auto const& obj : objs)
	os << obj << ' ';
	return os;
}


Eigen::Vector3d GreatestNormVector(Eigen::Vector3d& vect3...) {

	std::vector<Eigen::Vector3d> vecs = {vect3...};
	return std::max_element(vecs.begin(), vecs.end(), vect3_norm_compare);

}

Eigen::Vector3d VersorLemma(const Eigen::RotMatrix& r1, const Eigen::RotMatrix& r2) {
	//Init
	double costh = 0, sinth = 0, theta = 0;

	//Each element of r1 for each element of r2
	//Matrix h(r1.ElementProd(r2));
	Eigen::RotMatrix h = r1;

	h = r1.cwiseProduct(r2);
	/*
	 r1=[i1|j1|k1] & r2=[i2|j2|k2]
	 costh=(i1^i2+j1^j2+k1^k2-1)/2
	 where ^ represents the scalar prod
	 */
	/*for (int i=0; i<h.cols()*h.rows(); i++) {

	 costh += h.M_[i];
	 }*/
	costh = h.sum();

	costh = (costh - 1) * 0.5;

	//Vect3 ro*sin(theta)

	Eigen::Vector3d rosinth;	// = CMAT::Vect3::GetVect3Buffer(r1.tid_);
	Eigen::Vector3d tmp1; //= CMAT::Vect3::GetVect3Buffer(r1.tid_);
	Eigen::Vector3d tmp2; // = CMAT::Vect3::GetVect3Buffer(r1.tid_);

	//Vect3 rosinth;
	/*for(int i=0;i<3;i++) {
	 rosinth.M_[i] = 0;
	 }*/
	rosinth.setZero();

	//ro*sin(th)=(i1 x i2 + j1 x j2 + k1 x k2)/2
	//x represents the cross prod
	for (int i = 0; i < 3; i++) {
		//Get each column of r1
		tmp1 = r1.col(i);	//block(1,i,3,1);//r1.GetSubMatrix(1,i,3,i);
		tmp2 = r2.col(i);	//block(1,i,3,1);//r2.GetSubMatrix(1,i,3,i);
		rosinth += (tmp1.cross(tmp2));
	}

	rosinth *= 0.5;

	//|ro*sin(th)|=|ro|*|sinth|=sinth
	sinth = rosinth.norm();

	Eigen::Vector3d out;	// = CMAT::Vect3::GetVect3Buffer(r1.tid_);

	//Vect3 out;

	if (sinth > rml::VersorLemmaThreshold) {
		// 0 < theta < 180 degrees
		theta = atan2(sinth, costh);
		out = (rosinth * (theta / sinth));
	} else {
		if (costh > 0) {
			// theta = 0 degrees
			out.setZero();
			/*for (int i = 0; i < 3; i++) {
			 out.M_[i] = 0;
			 }*/
		} else {
			// theta = 180 degrees
			h = (r1 + r2);

			//Get each column of h

			Eigen::Vector3d temp1;	// = CMAT::Vect3::GetVect3Buffer(r1.tid_);
			Eigen::Vector3d temp2;	// = CMAT::Vect3::GetVect3Buffer(r1.tid_);
			Eigen::Vector3d temp3;	// = CMAT::Vect3::GetVect3Buffer(r1.tid_);

			/*Vect3 temp1(h.GetSubMatrix(1,1,3,1));
			 Vect3 temp2(h.GetSubMatrix(1,2,3,2));
			 Vect3 temp3(h.GetSubMatrix(1,3,3,3));*/

			temp1 = h.col(1);	//block(1,1,3,1);//GetSubMatrix(1,1,3,1);
			temp2 = h.col(2);
			;	//block(1,2,3,1);//GetSubMatrix(1,2,3,2);
			temp3 = h.col(3);
			;	//block(1,3,3,1);//GetSubMatrix(1,3,3,3);

			temp1 = GreatestNormVector(temp1, temp2, temp3);

			if (temp1.norm() != 0) {
				out = (temp1 * (M_PI / temp1.norm()));
			} else {
				out.setZero();
				/*for (int i = 0; i < 3; i++) {
				 out.M_[i] = 0;
				 }*/
			}
		}
	}

	return out;
}

// Cartesian 3-components vector (Roll, -Pitch, Yaw) to Rotation Matrix
Eigen::RotMatrix& Vect2RPY(const Eigen::Vector3d& vec)
{
	Eigen::RotMatrix R;// = CMAT::RotMatrix::GetRotMatrixBuffer(tid_);

	double cr, sr, cp, sp, cy, sy;

	//computer the cos & sin of roll -pitch yaw
	cr = cos(vec[0]);
	sr = sin(vec[0]);
	cp = cos(vec[1]);
	sp = sin(vec[1]);
	cy = cos(vec[2]);
	sy = sin(vec[2]);

	/*
	 Create the resulting matrix

						   [cr*cp   -sr*cy+cr*sp*sy   sr*sy+cr*sp*cy]
	 R=Rroll*R-pitch*Ryaw= [sr*cp    cr*cy+sr*sp*sy  -cr*sy+sr*sp*cy]
						   [-sp       cp*sy            cp*cy        ]
	 */

	R(1, 1) = cr * cp;
	R(2, 1) = sr * cp;
	R(3, 1) = -sp;

	R(1, 2) = -sr * cy + cr * sp * sy;
	R(2, 2) = cr * cy + sr * sp * sy;
	R(3, 2) = cp * sy;

	R(1, 3) = sr * sy + cr * sp * cy;
	R(2, 3) = -cr * sy + sr * sp * cy;
	R(3, 3) = cp * cy;

	//return the resulting matrix
	return R;

}


// Computes 3-components Cartesian error with 2 6-components vectors as input
Eigen::Vector3d& VersorLemma(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
	return VersorLemma(Vect2RPY(v1), Vect2RPY(v2));
}
/*
// Computes 6-components Cartesian error with 2 Transformation matrices as input
Eigen::Vector6d& CartError(const Eigen::TransfMatrix& in1, const Eigen::TransfMatrix& in2) {

	Vect6& temp = CMAT::Vect6::GetVect6Buffer(in1.tid_);
	Vect3& temp3 = CMAT::Vect3::GetVect3Buffer(in1.tid_);

	//KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "Vect3", "before setfirst");
	temp3 = (VersorLemma(in1.GetRotMatrix(), in2.GetRotMatrix())) * (-1);
	//temp3 *= -1;
	temp.SetFirstVect3(temp3);
	//KAL::DebugConsole::Write(LOG_LEVEL_ERROR, "Vect3", "before setsecond");
	temp3 = in1.GetTrasl() - in2.GetTrasl();
	temp.SetSecondVect3(temp3);

	return temp;
}

// Computes 6-components Cartesian error with 2 6-components vectors as input
Vect6& CartError(const Vect6& v1, const Vect6& v2) {

	Vect6& temp = CMAT::Vect6::GetVect6Buffer(v1.tid_);

	temp.SetFirstVect3(VersorLemma(v1.GetFirstVect3().Vect2RPY(), v2.GetFirstVect3().Vect2RPY()) * (-1));
	temp.SetSecondVect3(v1.GetSecondVect3() - v2.GetSecondVect3());

	return temp;
}
*/
double DecreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x) {
	if (x <= xmin) {
		return ymax;
	}
	if (x >= xmax) {
		return ymin;
	}

	double cosarg;
	cosarg = (x - xmin) * M_PI / (xmax - xmin);
	//std::cout << "cosarg " << cosarg << " ";
	return (ymax - ymin) * (0.5 * cos(cosarg) + 0.5) + ymin;
}

double IncreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x) {
	if (x <= xmin) {
		return ymin;
	}
	if (x >= xmax) {
		return ymax;
	}

	double cosarg;
	cosarg = (x - xmin) * M_PI / (xmax - xmin) + M_PI;
	return (ymax - ymin) * (0.5 * cos(cosarg) + 0.5) + ymin;
}

}
