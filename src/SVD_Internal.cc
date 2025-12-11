/*
 * SVD_Internal.cc
 *
 *  Created on: Feb 19, 2018
 *      Author: fraw
 */

#include <cmath>
#include <cstring>
#include <iostream>

namespace rml {

double SmoothFunction(double x, double beta, double lambda)
{
    return lambda / 2 * (1 + tanh(1 / (1 - x / beta) - beta / x));
}

double SmoothTransition(double x, double beta, double xmin, double lambda)
{
    double h;
    if ((x <= xmin))
        h = lambda;
    else if ((x > xmin) && (x < (xmin + beta)))
        h = SmoothFunction(beta + xmin - x, beta, lambda);
    else
        h = 0;

    return h;
}

//  Raised cosine function  (out = lambda*cos[(in/th)*(PI/2)])
double RaisedCosine(double in, double th, double lambda)
{
    if (in < 0)
        in = -in;

    if (0 <= in && in <= th)
        return lambda * (0.5 * cos((in / th) * M_PI) + 0.5);
    else
        return 0;
}

//  Product of two matrices
short MatrixMultiply(const double* A, int m, int n, const double* B, char k, char p, double* output)
{

    int i;
    int j;
    int l;

    if (n != k) {
        return -1;
    }

    for (i = 0; i < m * p; i++) {
        output[i] = 0;
    }

    for (l = 0; l < p; l++) {
        for (i = 0; i < m; i++) {
            for (j = 0; j < n; j++) {
                output[i + l * m] += A[i + m * j] * B[j + l * n];
            }
        }
    }
    return 1;
}

//  Transpose a matrix (NOTE: the input and the output matrices must be different!!!)
void MatrixTranspose(const double* A, int m, int n, double* output)
{

    int i;
    int j;

    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            output[j + n * i] = A[i + m * j];
        }
    }
}
}
