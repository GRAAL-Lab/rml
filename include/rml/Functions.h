/**
 * \file
 *
 * \date 	Feb 21, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_FUNCTIONS_H_
#define INCLUDE_RML_FUNCTIONS_H_

#include "EulerRPY.h"
#include "RMLExceptions.h"
#include "Types.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

namespace rml {
/**
 * @brief Using four parameters represention in the form:
 * Ax + By + Cz + D = 0;
 */
struct PlaneParameters {
    double A, B, C, D;

    PlaneParameters()
        : A(0)
        , B(0)
        , C(0)
        , D(0)
    {
    }
};
/**
 * @brief Compute the misalignment error between two vectors. The result is the vector around which v1 has to rotate to reach v2.
 *
 * @param[in] v1 first orientation vector
 * @param[in] v2 second orientation vector
 * @return the axis around which v1 has to rotate to reach v2
 */
Eigen::Vector3d ReducedVersorLemma(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

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
Eigen::Vector3d VersorLemma(const Eigen::RotationMatrix& r1, const Eigen::RotationMatrix& r2);
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
Eigen::Vector3d VersorLemma(const EulerRPY& v1, const EulerRPY& v2);
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
 * @note the two transformation matrix should have a common base frame, i.e. CartesianError(wTt, wTg) brings the tool
 * 		 frame \<t\> towards a goal frame \<g\>, and returns the error projected on frame \<w\>
 */
Eigen::Vector6d CartesianError(const Eigen::TransformationMatrix& in1, const Eigen::TransformationMatrix& in2);
/**
 * @brief Compute the Cartesian error between two transformation matrices
 *
 * The method computes the misalignment error and the position error between two transformation matrices expressed as their 6 parameters representation
 * The method assumes that the two Vect6 are a 6 parameter representation of the type
 * v = [yaw pitch roll x y z] leading to the following rotational part
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * It uses the VersorLemma to compute the misalignment, and a simple difference to compute the linear distance
 * The result is the error that brings in1 towards in2
 *
 * @param[in] v1 the initial transformation matrix expressed as its 6 parameter representation
 * @param[in] v2 the goal transformation matrix expressed as its 6 parameter representation
 *
 * @return the Vect6 representing the axis around which in2 should rotate to reach in1 and the linear distance
 *
 * @note the two vector should represent transformation matrix computed w.r.t a common frame, i.e. v1 = wTt, v2 = wTg then CartesianError(v1, v2)
 *       brings the tool frame \<t\> towards a goal frame \<g\>, and returns the error projected on frame \<w\>
 */
Eigen::Vector6d CartesianError(const Eigen::Vector6d& v1, const Eigen::Vector6d& v2);
/**
 * @brief A decreasing bell shaped (sigmoid) function.
 *
 * The output of this function has a smooth behavior, starting from the value (xmin, ymax) to (xmax, ymin)
 * For \f$ x < x_{min}, y = y_{max}\f$ \n
 * For \f$ x > x_{max}, y = y_{min}\f$ \n
 *
 * @param[in] xmin the lower extreme value where the transition begins
 * @param[in] xmax the higher extreme value where the transition stops
 * @param[in] ymin the lowest value of the function
 * @param[in] ymax the highest value of the function
 * @param[in] x the value where the function is evaluated
 *
 * @return the value of the function
 */
double DecreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x) noexcept(false);
/**
 * @brief An increasing bell shaped (sigmoid) function.
 *
 * @details The output of this function has a smooth behavior, starting from the value (xmin, ymin) to (xmax, ymax)\n
 *     For \f$ x < x_{min}, y = y_{min}\f$ \n
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
double IncreasingBellShapedFunction(double xmin, double xmax, double ymin, double ymax, double x) noexcept(false);
/**
 * @brief Saturate the scalar to a given maximum value
 * @param[in] sat 			the value of the saturation
 * @param[in,out] value 	the value which is saturated to the maximum value
 */
void SaturateScalar(double sat, double& value);
/**
 * @brief Saturate the vector to a given maximum value applying normalization for preserving the vector direction
 *
 * @param[in] sat 			the value of the saturation
 * @param[in,out] vector 	the vector which is saturated to the maximum value
 */
void SaturateVector(const double sat, Eigen::VectorXd& vector);

/**
 * @brief Saturate the vector within the specified limits.
 *
 * This function ensures that all components of a given vector remain within the bounds
 * defined by the provided upper and lower limits. If any component of the vector exceeds
 * its corresponding upper limit or falls below its lower limit, the entire vector is scaled
 * proportionally to keep all components within the valid range.
 *
 * The scaling process ensures that the direction of the vector is preserved while bringing
 * all components within bounds. The scaling factor is calculated based on the strictest limit
 * violation across all components.
 *
 * @param[in] upper_limits  The upper limits for each component of the vector.
 *                          This must be the same size as the input vector.
 * @param[in] lower_limits  The lower limits for each component of the vector.
 *                          This must be the same size as the input vector.
 * @param[in,out] vect      The vector to be saturated. This vector is updated in place
 *                          to ensure all its components lie within the specified bounds.
 *
 * @note If the size of the `upper_limits` or `lower_limits` vectors does not match the size
 *       of the input `vect`, the function will throw a `std::invalid_argument` exception.
 *
 * @throws std::invalid_argument If the size of the `upper_limits` or `lower_limits`
 *                               does not match the size of `vect`.
 */
void SaturateVector(const Eigen::VectorXd& upper_limits, const Eigen::VectorXd& lower_limits, Eigen::VectorXd& vect);


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
/**
 * \brief Computes the skew symmetric matrix form for a vector
 *
 * The output is the following:\n
 * \f$ t\wedge = \left| \begin{array}{ccc}
				0 & -z  & y \\
				z &  0  & -x \\
			   -y &  x  & 0
				\end{array} \right| \f$
 *
 * @param t input vector
 * @return the skew symmetric matrix
 */
Eigen::Matrix3d Vect3ToSkew(const Eigen::Vector3d& t);
/**
 * @brief Compute a rigid body matrix.
 *
 * This method assumes that the three values stored in the Vect3 correspond to a translation r between two frames
 * Then it computes the rigid body transformation matrix defined as\n
 *
 * \f$

\left| \begin{array}{c}
		\omega  \\
		   v
		\end{array} \right|
		=
 \left| \begin{array}{cc}
		 I_{3 \times 3} &  0_{3 \times 3}  \\
	    -r\wedge        &  I_{3 \times 3}  \\
		\end{array} \right|
\left| \begin{array}{c}
		\omega  \\
		   v
		\end{array} \right|
		 \f$
 *
 *
 *
 * @return the rigid body matrix
 */
Eigen::Matrix6d RigidBodyMatrix(const Eigen::Vector3d& transl);
/**
 * @brief ChangeJacobianObserver Method implementing change of observer for a cartesian Jacobian. \n
 * @details \f$ J^{v}_{error}=J^{o}_{error}-S_{e/o}J^{o}_{v}  \f$
 * @param J error jacobian wrt to the inertial frame
 * @param Jobserver jacobian of the observer wrt to the inertial frame
 * @param CartesianError error
 * @return Jacobian of the error observed by the input observer
 */
Eigen::MatrixXd ChangeJacobianObserver(Eigen::MatrixXd J, Eigen::MatrixXd Jobserver, Eigen::Vector3d CartesianError);
/**
 * @brief A norm comparing function (is "a < b" ?) to be binded to STL algorithms.
 *
 * @param a		lhs of less than comparison
 * @param b		rhs of less than comparison
 * @return		true if a is smaller than b, false otherwise
 */
template <class MatT>
static bool eigen_norm_compare(MatT& a, MatT& b)
{
    return (a.norm() < b.norm());
}
/**
 * @brief An utility to find the vector with the greatest norm among three.
 *
 * @param vect1
 * @param vect2
 * @param vect3
 * @return The vector with the greatest norm among the three
 */
template <class MatT>
MatT GreatestNormElement(const MatT& vect1, const MatT& vect2, const MatT& vect3)
{

    std::vector<MatT> vecs = { vect1, vect2, vect3 };
    return *std::max_element(vecs.begin(), vecs.end(), eigen_norm_compare<MatT>);
}
template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}
}

#endif /* INCLUDE_RML_FUNCTIONS_H_ */
