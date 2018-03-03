/*
 * Functions.cc
 *
 *  Created on: Feb 21, 2018
 *      Author: fraw
 */

#include <initializer_list>
#include "rml/Functions.h"
#include "rml/MatrixOperations.h"

namespace rml {

const double VersorLemmaThreshold = 1E-9;

Eigen::Vector3d VersorLemma(const Eigen::RotMatrix& r1, const Eigen::RotMatrix& r2) {

	double costh = 0, sinth = 0, theta = 0;

	//Each element of r1 for each element of r2
	Eigen::RotMatrix h = r1.cwiseProduct(r2);
	/*
	 r1=[i1|j1|k1] & r2=[i2|j2|k2]
	 costh=(i1^i2+j1^j2+k1^k2-1)/2
	 where ^ represents the scalar prod
	 */
	costh = h.sum();
	costh = (costh - 1) * 0.5;

	//Vect3 ro*sin(theta)

	Eigen::Vector3d rosinth;
	rosinth.setZero();

	//ro*sin(th)=(i1 x i2 + j1 x j2 + k1 x k2)/2
	//x represents the cross prod
	for (int i = 0; i < 3; i++) {
		//Get each column of r1
		rosinth += (r1.col(i).cross(r2.col(i)));
	}

	rosinth *= 0.5;

	//|ro*sin(th)|=|ro|*|sinth|=sinth
	sinth = rosinth.norm();

	Eigen::Vector3d out;

	if (sinth > rml::VersorLemmaThreshold) {
		// 0 < theta < 180 degrees
		theta = atan2(sinth, costh);
		out = (rosinth * (theta / sinth));
	} else {
		if (costh > 0) {
			// theta = 0 degrees
			out.setZero();
		} else {
			// theta = 180 degrees
			h = (r1 + r2);

			//Get each column of h
			Eigen::Vector3d temp1;
			Eigen::Vector3d temp2;
			Eigen::Vector3d temp3;

			temp1 = h.col(1);
			temp2 = h.col(2);
			temp3 = h.col(3);

			temp1 = GreatestNormElement(temp1, temp2, temp3);

			if (temp1.norm() != 0) {
				out = (temp1 * (M_PI / temp1.norm()));
			} else {
				out.setZero();
			}
		}
	}

	return out;
}

// Computes 3-components Cartesian error with 2 EulerYPR elements as imput
Eigen::Vector3d VersorLemma(const EulerYPR& v1, const EulerYPR& v2) {
	return VersorLemma(v1.ToRotMatrix(), v2.ToRotMatrix());
}


// Computes 6-components Cartesian error with 2 Transformation matrices as input
Eigen::Vector6d CartesianError(const Eigen::TransfMatrix& in1, const Eigen::TransfMatrix& in2) {

	Eigen::Vector3d angular = VersorLemma(in1.GetRotMatrix(), in2.GetRotMatrix());
	Eigen::Vector3d linear = in2.GetTransl() - in1.GetTransl();

	return Eigen::Vector6d(angular, linear);
}

// Computes 6-components Cartesian error with 2 6-components vectors as input
Eigen::Vector6d CartesianError(const Eigen::Vector6d& v1, const Eigen::Vector6d& v2) {
	Eigen::Vector3d angular = VersorLemma(EulerYPR(v1.GetFirstVect3()),
			EulerYPR(v2.GetFirstVect3()));
	Eigen::Vector3d linear = v2.GetSecondVect3() - v1.GetSecondVect3();

	return Eigen::Vector6d(angular, linear);
}

double DecreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x) {
	if (x <= xmin) {
		return ymax;
	}
	if (x >= xmax) {
		return ymin;
	}

	double cosarg;
	cosarg = (x - xmin) * M_PI / (xmax - xmin);
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

void SaturateVector(const int vecSize, const double sat, Eigen::VectorXd &vect)
{
	double curr_max = 0.0;
	for (int i = 0; i < vecSize; i++) {
		if (curr_max < std::fabs(vect(i))) {
			curr_max = std::fabs(vect(i));
		}
	}
	if (curr_max > sat) {
		for (int i = 0; i < vecSize; i++) {
			vect(i + 1) = vect(i + 1) * (sat / curr_max);
		}
	}
}

void SaturateScalar(double sat, double& value)
{
	if (value > sat)
		value = sat;
	else if (value < -sat)
		value = -sat;
}

double DistancePointToPlane(const Eigen::Vector3d& point, const PlaneParameters &planeParams)
{
	double numerator = std::fabs(planeParams.A * point(1) + planeParams.B * point(2) + planeParams.C * point(3) + planeParams.D);
	double denominator = std::sqrt(planeParams.A*planeParams.A + planeParams.B*planeParams.B + planeParams.C*planeParams.C);

	//cout << "n d: " << numerator << " " << denominator << "\n";
	return (numerator/denominator);
}

Eigen::Vector3d ClosestPointOnPlane(const Eigen::Vector3d& point, const PlaneParameters &planeParams)
{
	Eigen::Vector3d resultingPoint = Eigen::Vector3d::Zero();
	double alpha_num, alpha_den, alpha;
	alpha_num = -(planeParams.D + planeParams.A * point(1) + planeParams.B * point(2) + planeParams.C * point(3));
	alpha_den = planeParams.A*planeParams.A + planeParams.B*planeParams.B + planeParams.C*planeParams.C;
	alpha = alpha_num/alpha_den;
	resultingPoint(1) = point(1) + alpha * planeParams.A;
	resultingPoint(2) = point(2) + alpha * planeParams.B;
	resultingPoint(3) = point(3) + alpha * planeParams.C;

	return resultingPoint;
}

Eigen::Matrix3d Vect3ToSkew(const Eigen::Vector3d& t)
{
	Eigen::Matrix3d t_hat;
	t_hat << 0,    -t(2),  t(1),
			 t(2),    0,  -t(0),
			-t(1),  t(0),    0;
	return t_hat;
}

Eigen::Matrix6d GetRigidBodyMatrix(const Eigen::Vector3d& transl)
{
	Eigen::Matrix6d S;
	S.block(0,0,3,3) = S.block(3,3,3,3) = Eigen::Matrix3d::Identity();
	S.block(0,3,3,3) = Eigen::Matrix3d::Zero();
	S.block(3,0,3,3) = -1.0 * Vect3ToSkew(transl);
	return S;
}

}
