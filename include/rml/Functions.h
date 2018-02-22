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
#include <rml/Defines.h>

namespace rml {

/*template <typename Derived>
static bool vect3_norm_compare(Eigen::MatrixBase<Derived>& a, Eigen::MatrixBase<Derived>& b) {
	return (a.norm() < b.norm());
}*/

/*template <typename Derived>
Eigen::MatrixBase<Derived> GreatestNormVector(const Eigen::MatrixBase<Derived>& vect1, const Eigen::MatrixBase<Derived>& vect2, const Eigen::MatrixBase<Derived>& vect3) {

	std::vector<Eigen::MatrixBase<Derived>> vecs = {vect1, vect2, vect3};
	//Eigen::Vector3d max = ;
	return *std::max_element(vecs.begin(), vecs.end(), [](Eigen::MatrixBase<Derived> a, Eigen::MatrixBase<Derived> b){ return a.norm() < b.norm(); });
}*/


/*template <typename Derived>
 void print_inv_cond(const Eigen::MatrixBase<Derived>& a)
 {
 const typename Eigen::JacobiSVD<typename Derived::PlainObject>::SingularValuesType&
 sing_vals = a.jacobiSvd().singularValues();
 std::cout << "inv cond: " << sing_vals(sing_vals.size()-1) / sing_vals(0) << std::endl;
 }*/

//Eigen::Vector6d GreatestNormVector(Eigen::Vector6d& vect1, Eigen::Vector6d& vect2, Eigen::Vector6d& vect3);

Eigen::Vector3d GreatestNormVector(Eigen::Vector3d& vect1, Eigen::Vector3d& vect2, Eigen::Vector3d& vect3);


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
 *       that brings frame <a> over frame <b> projected on <c>
 */
Eigen::Vector3d VersorLemma(const Eigen::RotMatrix& r1, const Eigen::RotMatrix& r2);


Eigen::RotMatrix Vect2RPY(const Eigen::Vector3d& vec);

/**
 * @brief Compute the versor lemma between the two rotation matrices.
 *
 * Computes the misalignment vector between the two frames, represented by the two rotation matrices expressed as their RPY representation
 * In particular, both v1 and v2 are assumed to be in the form: [yaw -pitch roll] giving the following matrix
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * The vector is the axis around which the first frame should rotate in order to become equal to the second (angle-axis representation).
 *
 * @param[in] r1 the RPY representation of the first rotation matrix
 * @param[in] r2 the RPY representation of the second rotation matrix
 *
 * @return the Vect3 representing the axis around which v1 should rotate to reach v2, where its modulus is the angle
 *
 * @note the two vector should represent rotation matrices computed w.r.t a common frame, i.e. r1 = cRa, r2 = cRb then the versor lemma gives the rotation vector
 *       that brings frame <a> over frame <b> projected on <c>
 */
Eigen::Vector3d VersorLemma(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

/**
 * @brief Compute the Cartesian error between two transformation matrices.
 *
 * The method computes the misalignment error and the position error between two transformation matrices
 * It uses the VersorLemma to compute the misalignment, and a simple difference to compute the linear distance
 * The result is the error that brings in2 towards in1
 *
 * @note for historical reasons this method brings the second frame towards the first, as opposed to what the implementation of VersorLemma does
 *
 * @param[in] in1 the goal transformation matrix
 * @param[in] in2 the initial transformation matrix
 *
 * @return the Vect6 representing the axis around which in2 should rotate to reach in1 and the linear distance
 *
 * @note the two transformation matrix should have a common base frame, i.e. CartError(wTg, wTt) brings the tool frame <t> towards a goal frame <g>, and returns the error projected on frame <w>
 */
Eigen::Vector6d CartError(const Eigen::TransfMatrix&  in1, const Eigen::TransfMatrix&  in2);

/**
 * @brief Compute the Cartesian error between two transformation matrices
 *
 * The method computes the misalignment error and the position error between two transformation matrices expressed as their 6 parameters representation
 * The method assumes that the two Vect6 are a 6 parameter representation of the type
 * v = [yaw -pitch roll x y z] leading to the following rotational part
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * It uses the VersorLemma to compute the misalignment, and a simple difference to compute the linear distance
 * The result is the error that brings in2 towards in1
 *
 * @note for historical reasons this method brings the second frame towards the first, as opposed to what the implementation of VersorLemma does
 *
 * @param[in] in1 the goal transformation matrix expressed as its 6 parameter representation
 * @param[in] in2 the initial transformation matrix expressed as its 6 parameter representation
 *
 * @return the Vect6 representing the axis around which in2 should rotate to reach in1 and the linear distance
 *
 * @note the two vector should represent transformation matrix computed w.r.t a common frame, i.e. v1 = wTg, v2 = wTt then CartError(v1, v2)
 *       brings the tool frame <t> towards a goal frame <g>, and returns the error projected on frame <w>
 */
//Eigen::Vector6d& CartError(const Eigen::Vector6d& v1, const Eigen::Vector6d& v2);

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
 * The output of this function has a smooth behavior, starting from the value (xmin, ymin) to (xmax, ymax)
 * For x < xmin,  y = ymin
 * For x > xmax, y = ymax
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



}



#endif /* INCLUDE_RML_FUNCTIONS_H_ */
