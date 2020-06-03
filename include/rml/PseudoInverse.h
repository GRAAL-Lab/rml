/**
 * \file
 *
 * \date 	Feb 15, 2018
 * \author 	Francesco Wanderlingh
 */

#ifndef INCLUDE_RML_PSEUDOINVERSE_H_
#define INCLUDE_RML_PSEUDOINVERSE_H_

#include <eigen3/Eigen/Dense>
#include <libconfig.h++>

#include "SVD.h"
#include "Types.h"

namespace rml {

struct RegularizationParameters {
    double threshold; //!< The value above which the raised cosine becomes 0
    double lambda; //!< The maximum value of the raised cosine

    RegularizationParameters()
        : threshold(0.01)
        , lambda(0.01)
    {
    }
};

struct RegularizationResults {
    double mu; //!< Product of singular values
    int flag; //!< The number of time the regularization parameter was not zero

    RegularizationResults()
        : mu(0.0)
        , flag(0)
    {
    }
};

/**
 * @brief Regularization parameters and results container
 *
 * @details The SVDData struct contains input (\p params) and output (\p results) values for the SVD decompostion.
 * In particular \p threshold and \p lambda are inputs, while \p mu and
 * \p flag are outputs.

 * @note The default values assigned to the parameters are: \p threshold=0.01 and \p lamba=0.01 .
 *
 */
struct RegularizationData {
    RegularizationParameters params;
    RegularizationResults results;

    RegularizationData() = default;
};

/**
 * @internal for internal use only
 *
 * @brief Computes the SVD-based regularized matrix pseudoinversion (A = U*S*V')
 */
void GT_RegPinv(const double* J, int m, int n, double* JPInv, double treshold, double lambda, double* prod, int* flag);

/**
 * @brief Computes the SVD-based regularized matrix pseudoinversion (A = U*S*V')
 *
 * The regularization, being \f$ \sigma_i \f$ the i-th singular value,
 * is performed using following formula:\n
 *
 * \f$ \sigma_i = \sigma_i / ({\sigma_i}^2 + Reg) \f$, where, using the raised cosine\n
 *
 * \f$ Reg = \left\{ \begin{array}{ll}
        lambda/2 * (1 + cos((\sigma_i / thresh) * \pi)) & \mbox{if $0 \leq \sigma_i \leq th$}\\
        0 & \mbox{elsewhere}.
        \end{array} \right. \f$
 *
 *
 * @param mat			The matrix to be inverted
 * @param regData		The regularization parameters and results struct
 * @return				The pseudo-inverse matrix of \p mat
 */
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> RegularizedPseudoInverse(
    const MatT& mat, RegularizationData& regData) // choose appropriately
{

    int m = mat.rows(), n = mat.cols();
    double J[m * n]; // NULL pointer
    double JPInv[n * m];

    //Here we convert the input type to a double array which is the type used by the GT_RegPinv
    Eigen::Map<MatT>(J, m, n) = mat;

    GT_RegPinv(J, m, n, JPInv, regData.params.threshold, regData.params.lambda, &(regData.results.mu), &(regData.results.flag));

    //Here the results of the GT_RegPinv algorithm are mapped back to the input type
    MatT eigenPinv = Eigen::Map<MatT>(JPInv, n, m);

    return eigenPinv;
}

} //namespace rml

#endif /* INCLUDE_RML_PSEUDOINVERSE_H_ */
