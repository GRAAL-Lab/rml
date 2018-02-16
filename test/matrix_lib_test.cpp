// g++ test_matrix_lib.cpp -o test_matrix_lib -lCMAT_core -larmadillo -lgomp -fopenmp -march=native -O3

#include <cstdio>
#include <iostream>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cmat/cmat.h>

//#include <CMAT2/core/core.hpp>
#include <eigen3/Eigen/Dense>

#include "test_defines.h"

int iterations = 1000;

using std::vector;
using std::cout;
using std::ofstream;
using std::endl;

double TimeDiff(timeval t1, timeval t2);
void RunCMAT(int size, const vector<double> &values, TimeResults &results);
void RunEigen(int size, const vector<double> &values, TimeResults &results);
void HTMLOutput(ofstream &os, const vector<vector<double> > &results, const vector<std::string> &labels);

int main()
{
    // Check if output directory exist and create it if needed
    struct stat st = { 0 };

    if (stat("./out", &st) == -1)
    {
        mkdir("./out", 0700);
    }

    int start = 4;
    int end = 64;
    timeval t1, t2;

    vector<vector<double> > svd_results;
    vector<vector<double> > mul_results;
    vector<vector<double> > add_results;
    vector<vector<double> > inv_results;
    vector<vector<double> > tran_results;
    vector<std::string> labels;

    for (int i = start; i < end; i = i * 2)
    {
		labels.push_back(std::to_string(i) + "x" + std::to_string(i));

        if (i >= 256)
        {
            iterations = 1;
        }
        else if (i >= 64)
        {
            iterations = 100;
        }
        else
        {
            iterations = 1000;
        }

        vector<double> values;

        // Fill with random values
        for (int j = 0; j < i; j++)
        {
            for (int k = 0; k < i; k++)
            {
                double v = rand() / (1.0 + RAND_MAX);
                values.push_back(v);
            }
        }

        TimeResults r1, r2, r3;

        RunCMAT(i, values, r1);
        RunEigen(i, values, r3);

        vector<double> svd, mul, add, inv, tran;

        svd.push_back(r1.svd / iterations);
        //svd.push_back(r2.svd / ITERATIONS);
        svd.push_back(r3.svd / iterations);
        svd_results.push_back(svd);

        mul.push_back(r1.mul / iterations);
        //mul.push_back(r2.mul / ITERATIONS);
        mul.push_back(r3.mul / iterations);
        mul_results.push_back(mul);

        add.push_back(r1.add / iterations);
        //add.push_back(r2.add / ITERATIONS);
        add.push_back(r3.add / iterations);
        add_results.push_back(add);

        inv.push_back(r1.inv / iterations);
        //inv.push_back(r2.inv / ITERATIONS);
        inv.push_back(r3.inv / iterations);
        inv_results.push_back(inv);

        tran.push_back(r1.tran / iterations);
        //tran.push_back(r2.tran / ITERATIONS);
        tran.push_back(r3.tran / iterations);
        tran_results.push_back(tran);

        cout << endl;
    }

    // Output HTML table
    ofstream output_svd(svd_file.c_str());
    ofstream output_mul(mul_file.c_str());
    ofstream output_add(add_file.c_str());
    ofstream output_inv(inv_file.c_str());
    ofstream output_tran(tran_file.c_str());

    HTMLOutput(output_svd, svd_results, labels);
    HTMLOutput(output_mul, mul_results, labels);
    HTMLOutput(output_add, add_results, labels);
    HTMLOutput(output_inv, inv_results, labels);
    HTMLOutput(output_tran, tran_results, labels);

    return 0;
}

void RunCMAT(int size, const vector<double> &values, TimeResults &results)
{
    CMAT::Matrix A(size, size), U(size, size), S(size, size), V(size, size);
    CMAT::Matrix B(size, size);
    CMAT::Matrix C(size, size);

    //arma::vec S;

    timeval t1, t2;
    double d;

    for (int i = 1; i <= size; i++)
    {
        for (int j = 1; j <= size; j++)
        {
            A(i, j) = values[i * size + j];
        }
    }

    /* The methods computes the singular value decomposition of the given matrix A, defined as
     * A = USV'
     * if A is of size m x n, then U is m x m, S is m x n and V is n x n
     */
    // SVD(const CMAT::Matrix& A, CMAT::Matrix& U, CMAT::Matrix& S, CMAT::Matrix& V);
    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        CMAT::SVD(A, U, S, V);
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.svd = d;
    cout << "CMAT SVD: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = A + B;
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.add = d;
    cout << "CMAT addition: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = A * B;
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.mul = d;
    cout << "CMAT multiply: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    double thresh = 0.01;
    double lambda = 0.0001;
    for (int i = 0; i < iterations; i++)
    {
        C = A.RegPseudoInverse(thresh, lambda);
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.inv = d;
    cout << "CMAT pseudo inverse: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = A.Transpose();
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.tran = d;
    cout << "CMAT transpose: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;
}

/*
 void RunArmadillo(int size, const vector <double> &values, Results &results)
 {
 arma::mat A(size, size), U, V;
 arma::mat B(size, size), C(size,size);
 arma::vec S;

 timeval t1, t2;
 double d;

 for(int i=0; i < size; i++) {
 for(int j=0; j < size; j++) {
 A(i,j) = values[i*size + j];
 B(i,j) = values[i*size + j];
 }
 }

 gettimeofday(&t1, NULL);
 for(int i=0; i < ITERATIONS; i++) {
 arma::svd(U, S, V, A);
 }
 gettimeofday(&t2, NULL);
 d = TimeDiff(t1,t2);
 results.svd = d;
 cout << "Armadillo SVD: " << size << "x" << size << " - " << TimeDiff(t1,t2) << endl;

 gettimeofday(&t1, NULL);
 for(int i=0; i < ITERATIONS; i++) {
 C = A+B;
 }
 gettimeofday(&t2, NULL);
 d = TimeDiff(t1,t2);
 results.add = d;
 cout << "Armadillo addition: " << size << "x" << size << " - " << TimeDiff(t1,t2) << endl;

 gettimeofday(&t1, NULL);
 for(int i=0; i < ITERATIONS; i++) {
 C = A*B;
 }
 gettimeofday(&t2, NULL);
 d = TimeDiff(t1,t2);
 results.mul = d;
 cout << "Armadillo multiply: " << size << "x" << size << " - " << TimeDiff(t1,t2) << endl;

 gettimeofday(&t1, NULL);
 for(int i=0; i < ITERATIONS; i++) {
 C = arma::inv(A);
 }
 gettimeofday(&t2, NULL);
 d = TimeDiff(t1,t2);
 results.inv = d;
 cout << "Armadillo invert: " << size << "x" << size << " - " << TimeDiff(t1,t2) << endl;

 gettimeofday(&t1, NULL);
 for(int i=0; i < ITERATIONS; i++) {
 C = arma::trans(A);
 }
 gettimeofday(&t2, NULL);
 d = TimeDiff(t1,t2);
 results.tran = d;
 cout << "Armadillo transpose: " << size << "x" << size << " - " << TimeDiff(t1,t2) << endl;
 }
 */
void RunEigen(int size, const vector<double> &values, TimeResults &results)
{
    Eigen::MatrixXd A(size, size), U, V;
    Eigen::MatrixXd B(size, size);
    Eigen::MatrixXd C(size, size);

    //arma::vec S;

    timeval t1, t2;
    double d;

    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            A(i, j) = values[i * size + j];
        }
    }

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.svd = d;
    cout << "Eigen SVD: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = A + B;
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.add = d;
    cout << "Eigen addition: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = A * B;
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.mul = d;
    cout << "Eigen multiply: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = EigenEigenPinv(A);
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.inv = d;
    cout << "Eigen pseudo inverse: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;

    gettimeofday(&t1, NULL);
    for (int i = 0; i < iterations; i++)
    {
        C = A.transpose();
    }
    gettimeofday(&t2, NULL);
    d = TimeDiff(t1, t2);
    results.tran = d;
    cout << "Eigen transpose: " << size << "x" << size << " - " << TimeDiff(t1, t2) << endl;
}
