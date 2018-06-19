/*
 *  rml_test.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: Francesco Wanderlingh
 *
 *  This executable validates the mathematical functionalities of the Robotic Mathematical Library (RML)
 */

#include "test/rml_test_defines.h"

using std::cout;
using std::endl;
using futils::PrettyPrint;

int main(int, char**)
{

    const int m = 4, n = 4;

    Eigen::MatrixXd A(m, n), U(m, m), S(m, n), V(n, n);

    A.setRandom();
    U.setRandom();
    S.setRandom();
    V.setRandom();

    ///////////////////////////////
    ///    MATRIX OPERATIONS    ///
    ///////////////////////////////

    std::cout << std::endl << tc::yellow << "### MATRIX OPERATIONS Test ###" << tc::none << std::endl;

    S = rml::RightJuxtapose(A, U);
    V = rml::UnderJuxtapose(A, U);
    PrettyPrint(A, "A");
    PrettyPrint(U, "U");
    PrettyPrint(S, "rml::RightJuxtapose(A,U)");
    PrettyPrint(V, "rml::UnderJuxtapose(A,U)");
    /*A = rml::RightJuxtapose(A,A);
    PrettyPrint(A, "rml::UnderJuxtapose(A,A)");*/

    Eigen::Vector6d vect6_1;
    Eigen::Vector3d vect3_1, vect3_2, vect3_3;
    vect3_1.setConstant(1);
    vect3_2.setConstant(2);
    vect6_1.setZero();
    PrettyPrint(vect6_1.transpose(), "vect6'");
    PrettyPrint(vect3_1.transpose(), "vect3_1'");
    PrettyPrint(vect3_2.transpose(), "vect3_2'");

    vect6_1.SetFirstVect3(vect3_1);
    PrettyPrint(vect6_1.transpose(), "vect6' after SetFirstVect3()");
    vect6_1.SetSecondVect3(vect3_2);
    PrettyPrint(vect6_1.transpose(), "vect6' after SetSecondVect3()");

    Eigen::TransfMatrix Tmat = A.setRandom(4, 4);
    PrettyPrint(Tmat, "Tmat");
    PrettyPrint(Tmat.GetRotMatrix(), "A.GetRotMatrix()");
    PrettyPrint(Tmat.GetTransl(), "A.GetTrasl()");

    vect3_3.setConstant(3);
    PrettyPrint(vect3_1.transpose(), "vect3_1");
    PrettyPrint(vect3_2.transpose(), "vect3_2");
    PrettyPrint(vect3_3.transpose(), "vect3_3");
    Eigen::Vector3d maxvect3 = rml::GreatestNormElement(vect3_1, vect3_2, vect3_3);
    futils::PrettyPrint(maxvect3.transpose(), "maxvect3");

    rml::EulerRPY rpy;
    rpy.SetRPY(0.0, 0.3, 0.4);
    PrettyPrint(rpy, "rpy cout");
    PrettyPrint(rpy.ToRotMatrix(), "rpy.ToRotMatrix()");
    PrettyPrint(rpy.ToRotMatrix().ToEulerRPY(), "rpy.ToRotMatrix().ToEulerYPR()");

    Eigen::Vector3d transl(1.0, 2.0, 3.0);
    PrettyPrint(rml::GetRigidBodyMatrix(transl), "rml::GetRigidBodyMatrix(transl)");
    // PrettyPrint(transl.GetRigidBodyMatrix(), "transl.GetRigidBodyMatrix()");

    ///////////////////////////////
    //////     PINV TEST     //////
    ///////////////////////////////

    std::cout << std::endl << tc::yellow << "### PINV Test (with timings) ###" << tc::none << std::endl;

    Eigen::MatrixXd Avar, Apinv;
    int iterations = 1000;
    int rowMinSize = 4, colMinSize = 4;
    int rowMaxSize = 12, colMaxSize = 12;
    int step = 4;

    TimeResults timings;
    PinvSpecs pinvSpecs;
    pinvSpecs.SVDdata.params.threshold = 0.01;
    pinvSpecs.SVDdata.params.lambda = 0.0001;

    for (int rows = rowMinSize; rows <= rowMaxSize; rows += step) {
        for (int cols = colMinSize; cols <= colMaxSize; cols += step) {

            pinvSpecs.nRows = rows;
            pinvSpecs.nCols = cols;
            PseudoInverseTest(iterations, Avar, pinvSpecs, Apinv, timings);
        }
    }

    Apinv = rml::RegularizedPseudoInverse(A, pinvSpecs.SVDdata);
    PrettyPrint((A * Apinv), "A * Apinv (4x4)");

    ///////////////////////////////
    //////     SVD TEST      //////
    ///////////////////////////////

    std::cout << std::endl << tc::yellow << "### SVD Test ###" << tc::none << std::endl;

    for (int i = 0; i < A.rows(); i++) {
        for (int j = 0; j < A.cols(); j++) {
            A(i, j) = i * A.cols() + j;
        }
    }

    rml::SVD(A, U, S, V);

    PrettyPrint(A, "A");
    PrettyPrint(U, "U");
    PrettyPrint(S, "S");
    PrettyPrint(V, "V");

    Eigen::MatrixXd A_usv = U * S * V.transpose();
    PrettyPrint(A_usv, "A as the result of: A = U*S*V'");

    Eigen::TransfMatrix wTt, wTg;
    std::cout << "CartesianError() 1: " << (rml::CartesianError(wTt, wTg)).transpose() << std::endl;

    wTg.SetTransl(Eigen::Vector3d(1, 1, 1));
    std::cout << "CartesianError() 2: " << (rml::CartesianError(wTt, wTg)).transpose() << std::endl;

    Eigen::Matrix3d n_rot;
    n_rot = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    // *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
    // *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

    rml::EulerRPY rpy2(0, M_PI, M_PI);
    futils::PrettyPrint(rpy2, "rpy2");

    futils::PrettyPrint(wTg, "wTg");
    futils::PrettyPrint(rpy2.ToRotMatrix(), "rpy2.toRotMatrix");

    wTg.SetRotMatrix(Eigen::Matrix3d::Identity());

    wTg.SetRotMatrix(n_rot);

    wTg.SetRotMatrix(rpy2.ToRotMatrix());

    futils::PrettyPrint(rml::CartesianError(wTt, wTg), "rml::CartesianError(wTt, wTg)");
    return 0;
}
