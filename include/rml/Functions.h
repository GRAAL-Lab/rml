/*
 * Functions.h
 *
 *  Created on: Feb 21, 2018
 *      Author: fraw
 */

#ifndef INCLUDE_RML_FUNCTIONS_H_
#define INCLUDE_RML_FUNCTIONS_H_


#include <array>
#include <vector>
#include <algorithm>
#include <iostream>
#include <vector>

#include <rml/Types.h>

namespace rml {

/**
 * Using four parameters represention in the form:
 * Ax + By + Cz + D = 0;
 */
struct PlaneParameters
{
    double A, B, C, D;

    PlaneParameters() : A(0), B(0), C(0), D(0) {}
};

template <class MatT>
static bool norm_compare(MatT& a, MatT& b) {
	return (a.norm() < b.norm());
}

template<class MatT>
MatT GreatestNormElement(const MatT& vect1, const MatT& vect2, const MatT& vect3){

	std::vector<MatT> vecs = {vect1,vect2,vect3};
	return *std::max_element(vecs.begin(), vecs.end(), norm_compare<MatT>);
}

/**
 * @brief Compute the versor lemma between the two rotation matrices.
 *
 * Computes the misalignment vector between the two frames, represented by the two rotation matrices
 * The vector is the axis around which the first frame should rotate in order to become equal to the second (angle-axis representation)
 *
 * @param[in] r1 the first rotation matrix
 * @param[in] r2 the second rotation matrix
 *
 * @return the Vect3 representing the axis around which r1 should rotate to reach r2, where its modulus is the angle
 *
 * @note the two rotation matrix should be w.r.t a common frame, i.e. r1 = cRa, r2 = cRb then the versor lemma gives the rotation vector
 *       that brings frame \<a\> over frame \<b\> projected on \<c\>
 */
Eigen::Vector3d VersorLemma(const Eigen::RotMatrix& r1, const Eigen::RotMatrix& r2);


/**
 * @brief Compute the versor lemma between the two rotation matrices.
 *
 * Computes the misalignment vector between the two frames, represented by the two rotation matrices expressed as their RPY representation
 * In particular, both v1 and v2 are assumed to be in the form: [yaw -pitch roll] giving the following matrix
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * The vector is the axis around which the first frame should rotate in order to become equal to the second (angle-axis representation).
 *
 * @param[in] v1 the RPY representation of the first rotation matrix
 * @param[in] v2 the RPY representation of the second rotation matrix
 *
 * @return the Vect3 representing the axis around which v1 should rotate to reach v2, where its modulus is the angle
 *
 * @note the two vector should represent rotation matrices computed w.r.t a common frame, i.e. r1 = cRa, r2 = cRb then the versor lemma gives the rotation vector
 *       that brings frame \<a\> over frame \<b\> projected on \<c\>
 */
Eigen::Vector3d VersorLemma(const EulerYPR& v1, const EulerYPR& v2);

/**
 * @brief Compute the Cartesian error between two transformation matrices.
 *
 * The method computes the misalignment error and the position error between two transformation matrices
 * It uses the VersorLemma to compute the misalignment, and a simple difference to compute the linear distance
 * The result is the error that brings in1 towards in2
 *
 * @param[in] in1 the initial transformation matrix
 * @param[in] in2 the goal transformation matrix
 *
 * @return the Vect6 representing the axis around which in2 should rotate to reach in1 and the linear distance
 *
 * @note the two transformation matrix should have a common base frame, i.e. CartError(wTg, wTt) brings the tool
 * 		 frame \<t\> towards a goal frame \<g\>, and returns the error projected on frame \<w\>
 */
Eigen::Vector6d CartesianError(const Eigen::TransfMatrix&  in1, const Eigen::TransfMatrix&  in2);

/**
 * @brief Compute the Cartesian error between two transformation matrices
 *
 * The method computes the misalignment error and the position error between two transformation matrices expressed as their 6 parameters representation
 * The method assumes that the two Vect6 are a 6 parameter representation of the type
 * v = [yaw -pitch roll x y z] leading to the following rotational part
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * It uses the VersorLemma to compute the misalignment, and a simple difference to compute the linear distance
 * The result is the error that brings in1 towards in2
 *
 * @param[in] v1 the initial transformation matrix expressed as its 6 parameter representation
 * @param[in] v2 the goal transformation matrix expressed as its 6 parameter representation
 *
 * @return the Vect6 representing the axis around which in2 should rotate to reach in1 and the linear distance
 *
 * @note the two vector should represent transformation matrix computed w.r.t a common frame, i.e. v1 = wTt, v2 = wTg then CartError(v1, v2)
 *       brings the tool frame \<t\> towards a goal frame \<g\>, and returns the error projected on frame \<w\>
 */
Eigen::Vector6d CartesianError(const Eigen::Vector6d& v1, const Eigen::Vector6d& v2);

/**
 * @brief A decreasing bell shaped (sigmoid) function.
 *
 * The output of this function has a smooth behavior, starting from the value (xmin, ymax) to (xmax, ymin)
 * For x < xmin,  y = ymax
 * For x > xmax, y = ymin
 *
 * @param[in] xmin the lower extreme value where the transition begins
 * @param[in] xmax the higher extreme value where the transition stops
 * @param[in] ymin the lowest value of the function
 * @param[in] ymax the highest value of the function
 * @param[in] x the value where the function is evaluated
 *
 * @return the value of the function
 */
double DecreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x);

/**
 * @brief An increasing bell shaped (sigmoid) function.
 *
 * @details The output of this function has a smooth behavior, starting from the value (xmin, ymin) to (xmax, ymax)\n
 *     For \f$ x < x_{min},  y = y_{min}\f$ \n
 *     For \f$ x > x_{max}, y = y_{max}\f$ \n
 *
 *
 * @param[in] xmin the lower extreme value where the transition begins
 * @param[in] xmax the higher extreme value where the transition stops
 * @param[in] ymin the lowest value of the function
 * @param[in] ymax the highest value of the function
 * @param[in] x the value where the function is evaluated
 *
 * @return the value of the function
 */
double IncreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x);

/**
 * @brief Saturate the scalar to a given maximum value
 * @param[in] sat 			the value of the saturation
 * @param[in,out] value 	the value which is saturated to the maximum value
 */
void SaturateScalar(double sat, double& value);

/**
 * @brief Saturate the vector to a given maximum value applying normalization for preserving the vector direction
 *
 * @param[in] vecSize		size of vector, to avoid unnecessary size identifications
 * @param[in] sat 			the value of the saturation
 * @param[in,out] vector 	the vector which is saturated to the maximum value
 */
void SaturateVector(const int vecSize, const double sat, Eigen::VectorXd& vector);

/**
 * @brief Evaluates the norm of the shortest distance vector among a point and a given plane
 *
 * @param[in]  point		The point from where calculate the distance
 * @param[in] planeParams	The target plane
 * @return The distance norm
 */
double DistancePointToPlane(const Eigen::Vector3d& point, const PlaneParameters& planeParams);

/**
 * @brief Given a point and a plane evaluates the coordinates of the closest point on plane w.r.t the input one.
 *
 * @param[in] point 		The point from where calculate the distance
 * @param[in] planeParams	The target plane
 * @return	The 3d coordinates of the closest point laying on the plane
 */
Eigen::Vector3d ClosestPointOnPlane(const Eigen::Vector3d& point, const PlaneParameters& planeParams);

}



#endif /* INCLUDE_RML_FUNCTIONS_H_ */
